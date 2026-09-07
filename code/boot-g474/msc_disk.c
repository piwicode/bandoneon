/*
 * The BANDOLIBRE drive: a FAT16 volume synthesised on the fly, and the UF2
 * parser behind it.
 *
 * Nothing about the filesystem is stored — tud_msc_read10_cb() builds whatever
 * sector the host asks for from the constants below. The host is free to write
 * anywhere it likes; every sector it writes is examined, those carrying a UF2
 * block for this chip are staged, and everything else (directory entries, FAT
 * updates, the .Spotlight-V100 and System Volume Information debris that
 * macOS and Windows leave on any removable drive) is discarded.
 *
 * That is the whole reason for using UF2 rather than a plain .bin: each block
 * carries its own destination address and "block N of M" counter, so the
 * bootloader never has to parse a directory, follow a cluster chain, or care
 * in what order the host flushes its writes.
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "flash_write.h"
#include "memmap.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// Volume geometry
//--------------------------------------------------------------------+
#define SECTOR_SIZE        512U

#define RESERVED_SECTORS   1U
#define FAT_COUNT          2U
#define SECTORS_PER_FAT    32U
#define ROOT_ENTRIES       256U
#define ROOT_SECTORS       (ROOT_ENTRIES * 32U / SECTOR_SIZE)   /* 16 */
#define CLUSTER_COUNT      8192U                                /* 1 sector each */

#define FAT_START          RESERVED_SECTORS                     /* 1 */
#define ROOT_START         (FAT_START + FAT_COUNT * SECTORS_PER_FAT)  /* 65 */
#define DATA_START         (ROOT_START + ROOT_SECTORS)               /* 81 */
#define TOTAL_SECTORS      (DATA_START + CLUSTER_COUNT)              /* 8273 */

/* FAT16 is only valid at 4085 clusters or more; below that a host reads the
 * volume as FAT12 and the FAT entries here would be misinterpreted. */
_Static_assert(CLUSTER_COUNT >= 4085U, "cluster count too low for FAT16");
_Static_assert(TOTAL_SECTORS <= 65535U, "total sectors must fit the 16-bit BPB field");

#define VOLUME_LABEL       "BANDOLIBRE "   /* exactly 11 bytes, space padded */

/* FAT packs a date as (year - 1980) << 9 | month << 5 | day. 2026-01-01. */
#define FAT_DATE           (((2026U - 1980U) << 9) | (1U << 5) | 1U)

static const char info_txt[] =
    "Bandolibre main board (STM32G474CB)\r\n"
    "UF2 bootloader\r\n"
    "\r\n"
    "Copy main-g474.uf2 onto this drive to update the firmware.\r\n"
    "Build it with 'just dfu' in code/main-g474.\r\n"
    "\r\n"
    "The drive disappears and the instrument restarts when the copy\r\n"
    "finishes. Files copied here are not stored: anything that is not a\r\n"
    "UF2 image for this board is ignored.\r\n";

/* One cluster is one sector here, so the file has to fit in a single one. */
_Static_assert(sizeof(info_txt) - 1U <= SECTOR_SIZE, "INFO_UF2.TXT exceeds one cluster");

#define INFO_CLUSTER       2U
#define INFO_SIZE          (sizeof(info_txt) - 1U)

//--------------------------------------------------------------------+
// UF2 block format (microsoft/uf2)
//--------------------------------------------------------------------+
#define UF2_MAGIC_START0   0x0A324655UL  /* "UF2\n" */
#define UF2_MAGIC_START1   0x9E5D5157UL
#define UF2_MAGIC_END      0x0AB16F30UL

#define UF2_FLAG_NOFLASH   0x00000001UL  /* block carries no data to write */
#define UF2_FLAG_FAMILY_ID 0x00002000UL  /* fileSize field holds a family id */

typedef struct {
  uint32_t magic_start0;
  uint32_t magic_start1;
  uint32_t flags;
  uint32_t target_addr;
  uint32_t payload_size;
  uint32_t block_no;
  uint32_t num_blocks;
  uint32_t family_id;    /* or file size, when UF2_FLAG_FAMILY_ID is clear */
  uint8_t  data[476];
  uint32_t magic_end;
} uf2_block_t;

_Static_assert(sizeof(uf2_block_t) == SECTOR_SIZE, "UF2 block must be one sector");

/* A UF2 payload is 256 bytes by convention, so this is the most blocks an
 * image filling the application region can need. */
#define MAX_BLOCKS         (APP_SIZE / 256U)

//--------------------------------------------------------------------+
// Staging state
//--------------------------------------------------------------------+

/* The whole image is assembled in RAM and only committed once every block has
 * arrived. An interrupted or truncated copy therefore never reaches flash, and
 * the bootloader has the RAM to spare — the application itself uses 8 KB of
 * the 128 KB. */
static uint8_t  image[APP_SIZE];
static uint8_t  block_seen[(MAX_BLOCKS + 7U) / 8U];
static uint32_t blocks_expected;
static uint32_t blocks_received;
static uint32_t image_len;
static bool     commit_ready;

static void staging_reset(uint32_t num_blocks)
{
  printf("uf2: new image, %lu blocks expected\r\n", (unsigned long) num_blocks);
  memset(image, 0xFF, sizeof(image));   /* gaps program as erased flash */
  memset(block_seen, 0, sizeof(block_seen));
  blocks_expected = num_blocks;
  blocks_received = 0;
  image_len = 0;
  commit_ready = false;
}

bool uf2_commit_ready(void)
{
  return commit_ready;
}

bool uf2_receiving(void)
{
  return blocks_received > 0 && !commit_ready;
}

bool uf2_commit(void)
{
  commit_ready = false;
  return flash_write_app(image, image_len);
}

/* Examine one 512-byte sector the host wrote. Returns without doing anything
 * unless it is a UF2 block belonging to a complete image for this board. */
static void uf2_consume(const uint8_t *buffer)
{
  const uf2_block_t *b = (const uf2_block_t *) buffer;

  if (b->magic_start0 != UF2_MAGIC_START0 ||
      b->magic_start1 != UF2_MAGIC_START1 ||
      b->magic_end    != UF2_MAGIC_END) {
    return;   /* not UF2: filesystem bookkeeping, or some other file */
  }

  /* Blocks for a different chip are ignored rather than misprogrammed. */
  if ((b->flags & UF2_FLAG_FAMILY_ID) && b->family_id != UF2_FAMILY_ID) return;
  if (b->flags & UF2_FLAG_NOFLASH) return;

  if (b->num_blocks == 0 || b->num_blocks > MAX_BLOCKS) return;
  if (b->block_no >= b->num_blocks) return;
  if (b->payload_size > sizeof(b->data)) return;

  /* Reject anything outside the application region. This is what stops a UF2
   * linked for 0x08000000 — a Debug image, or firmware for another project —
   * from overwriting the bootloader itself. */
  if (b->target_addr < APP_BASE) return;
  if (b->target_addr + b->payload_size > APP_BASE + APP_SIZE) return;
  if (b->target_addr + b->payload_size < b->target_addr) return;   /* overflow */

  /* A new file (or a retry after a failed one) starts a fresh image. */
  if (b->num_blocks != blocks_expected) staging_reset(b->num_blocks);

  uint32_t const offset = b->target_addr - APP_BASE;
  memcpy(image + offset, b->data, b->payload_size);

  if (offset + b->payload_size > image_len) image_len = offset + b->payload_size;

  /* Count each block once: hosts do re-send sectors. */
  uint8_t const mask = (uint8_t) (1U << (b->block_no % 8U));
  if ((block_seen[b->block_no / 8U] & mask) == 0U) {
    block_seen[b->block_no / 8U] |= mask;
    blocks_received++;
  }

  if (blocks_received == blocks_expected) {
    printf("uf2: all %lu blocks received (image_len=%lu)\r\n",
           (unsigned long) blocks_expected, (unsigned long) image_len);
    commit_ready = true;
  }
}

//--------------------------------------------------------------------+
// TinyUSB MSC callbacks
//--------------------------------------------------------------------+

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16],
                        uint8_t product_rev[4])
{
  (void) lun;
  memcpy(vendor_id,   "Bandolb ", 8);
  memcpy(product_id,  "Firmware Update ", 16);
  memcpy(product_rev, "1.0", 3);
  product_rev[3] = ' ';
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
  (void) lun;
  *block_count = TOTAL_SECTORS;
  *block_size  = SECTOR_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
  (void) lun; (void) power_condition; (void) start; (void) load_eject;
  return true;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

/* Build the requested sector. Everything is derived from the geometry
 * constants; there is no backing store. */
static void read_sector(uint32_t lba, uint8_t *out)
{
  memset(out, 0, SECTOR_SIZE);

  if (lba == 0) {
    /* FAT16 boot sector / BIOS parameter block */
    static const uint8_t jump[3] = { 0xEB, 0x3C, 0x90 };
    memcpy(out + 0, jump, 3);
    memcpy(out + 3, "MSDOS5.0", 8);
    out[11] = (uint8_t) (SECTOR_SIZE & 0xFF);
    out[12] = (uint8_t) (SECTOR_SIZE >> 8);
    out[13] = 1;                                  /* sectors per cluster */
    out[14] = (uint8_t) RESERVED_SECTORS;
    out[15] = 0;
    out[16] = FAT_COUNT;
    out[17] = (uint8_t) (ROOT_ENTRIES & 0xFF);
    out[18] = (uint8_t) (ROOT_ENTRIES >> 8);
    out[19] = (uint8_t) (TOTAL_SECTORS & 0xFF);
    out[20] = (uint8_t) (TOTAL_SECTORS >> 8);
    out[21] = 0xF8;                               /* fixed disk */
    out[22] = (uint8_t) SECTORS_PER_FAT;
    out[23] = 0;
    out[24] = 1;                                  /* sectors per track */
    out[26] = 1;                                  /* heads */
    out[36] = 0x80;                               /* drive number */
    out[38] = 0x29;                               /* extended boot signature */
    out[39] = 0x42; out[40] = 0x41; out[41] = 0x4E; out[42] = 0x44;  /* volume id */
    memcpy(out + 43, VOLUME_LABEL, 11);
    memcpy(out + 54, "FAT16   ", 8);
    out[510] = 0x55;
    out[511] = 0xAA;
    return;
  }

  if (lba < ROOT_START) {
    /* Both FAT copies. Only the first sector of each holds anything: the two
     * reserved entries and the single cluster of INFO_UF2.TXT. */
    uint32_t const sector_in_fat = (lba - FAT_START) % SECTORS_PER_FAT;
    if (sector_in_fat == 0) {
      out[0] = 0xF8; out[1] = 0xFF;   /* entry 0: media descriptor */
      out[2] = 0xFF; out[3] = 0xFF;   /* entry 1: end of chain */
      out[4] = 0xFF; out[5] = 0xFF;   /* entry 2: INFO_UF2.TXT, last cluster */
    }
    return;
  }

  if (lba < DATA_START) {
    /* Root directory: the volume label and one file, both in the first sector. */
    if (lba == ROOT_START) {
      memcpy(out + 0, VOLUME_LABEL, 11);
      out[11] = 0x08;                             /* volume label */

      memcpy(out + 32, "INFO_UF2TXT", 11);
      out[32 + 11] = 0x01;                        /* read only */
      /* Write time and date. A zero date is out of range for FAT and makes
       * some file managers show the entry as invalid; this is 2026-01-01. */
      out[32 + 24] = (uint8_t) (FAT_DATE & 0xFF);
      out[32 + 25] = (uint8_t) (FAT_DATE >> 8);
      out[32 + 26] = (uint8_t) (INFO_CLUSTER & 0xFF);
      out[32 + 27] = (uint8_t) (INFO_CLUSTER >> 8);
      out[32 + 28] = (uint8_t) (INFO_SIZE & 0xFF);
      out[32 + 29] = (uint8_t) ((INFO_SIZE >> 8) & 0xFF);
      out[32 + 30] = (uint8_t) ((INFO_SIZE >> 16) & 0xFF);
      out[32 + 31] = (uint8_t) ((INFO_SIZE >> 24) & 0xFF);
    }
    return;
  }

  if (lba == DATA_START + (INFO_CLUSTER - 2U)) {
    memcpy(out, info_txt, INFO_SIZE);
  }
  /* Every other data sector reads back as zeros. */
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void *buffer, uint32_t bufsize)
{
  (void) lun;

  if (lba >= TOTAL_SECTORS) return -1;
  if (offset + bufsize > SECTOR_SIZE) return -1;

  static uint8_t sector[SECTOR_SIZE];
  read_sector(lba, sector);
  memcpy(buffer, sector + offset, bufsize);

  return (int32_t) bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t *buffer, uint32_t bufsize)
{
  (void) lun;

  if (lba >= TOTAL_SECTORS) return -1;

  /* CFG_TUD_MSC_EP_BUFSIZE is 512, so a whole sector arrives at once and a UF2
   * block is never split across two calls. Anything else is filesystem
   * housekeeping, which is accepted and dropped. */
  if (offset == 0 && bufsize == SECTOR_SIZE) uf2_consume(buffer);

  return (int32_t) bufsize;
}

/* Not in TinyUSB's own SCSI opcode enum (msc.h stops at WRITE_10), but it's a
 * real, commonly-issued command: Windows in particular sends it after writes,
 * sometimes right as a copy is finishing. */
#define SCSI_CMD_SYNCHRONIZE_CACHE_10 0x35

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
  (void) lun; (void) buffer; (void) bufsize;

  switch (scsi_cmd[0]) {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      return 0;   /* nothing to lock; report success so the host stops asking */

    case SCSI_CMD_SYNCHRONIZE_CACHE_10:
      /* "Flush anything you're still holding in a write cache to permanent
       * storage." Every write10_cb() call above already lands in the staging
       * buffer synchronously, so there's nothing to flush — but failing this
       * is a needless landmine: some host stacks treat a rejected cache-sync
       * as an error and retry or stall rather than shrugging it off. */
      return 0;

    default:
      tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
      return -1;
  }
}
