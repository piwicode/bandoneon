/* Host unit tests for the property system. Pure C, no HAL — compile and run
 * natively (see `just test`). Framework-free: a tiny assert macro that counts
 * failures and reports a final summary. */

#include "properties.h"
#include "properties_console.h"

#include <stdio.h>
#include <string.h>

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_failures++;                                                     \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
    }                                                                   \
  } while (0)

/* Resolve a property index by name or abort the test if it is missing. */
static size_t idx(const char *name)
{
  size_t i = (size_t)-1;
  if (!property_by_name(name, &i)) {
    printf("FATAL: unknown property '%s'\n", name);
    g_failures++;
  }
  return i;
}

static void test_defaults(void)
{
  /* No reset first: this verifies the static initialization of the live
   * struct (values are valid before any call). Runs first in main(). */
  CHECK(property_count() == 46);

  /* Direct reads match defaults from property_table.def. */
  CHECK(g_properties->key_press == 1900);
  CHECK(g_properties->key_release == 2100);
  CHECK(g_properties->bellow_center == 11280);
  CHECK(g_properties->bellow_cc_period_ms == 10);

  /* Same value via the index API. */
  uint16_t v = 0;
  CHECK(property_get_u16(idx("key_press"), &v) && v == 1900);
}

static void test_lookup(void)
{
  size_t i;
  CHECK(property_by_name("bellow_center", &i));
  CHECK(property_at(i)->tag == 3);
  CHECK(strcmp(property_at(i)->name, "bellow_center") == 0);

  CHECK(!property_by_name("does_not_exist", &i));
  CHECK(property_at(property_count()) == NULL);
}

static void test_set_clamp(void)
{
  property_reset_all();
  size_t i = idx("key_press"); /* range [0,4095] */

  CHECK(property_set_u16(i, 1234));
  CHECK(g_properties->key_press == 1234);

  CHECK(property_set_u16(i, 60000)); /* above max -> clamps to 4095 */
  CHECK(g_properties->key_press == 4095);

  size_t j = idx("bellow_cc_period_ms"); /* range [1,1000] */
  CHECK(property_set_u16(j, 0));  /* below min -> clamps to 1 */
  CHECK(g_properties->bellow_cc_period_ms == 1);

  CHECK(!property_set_u16(property_count(), 5)); /* bad index */
}

static void test_type_guards(void)
{
  property_reset_all();
  size_t i = idx("key_press"); /* a U16 property */
  bool b;
  CHECK(!property_get_bool(i, &b));      /* wrong type */
  CHECK(!property_set_bool(i, true));    /* wrong type */
}

static void test_reset(void)
{
  property_reset_all();
  size_t i = idx("key_press");
  CHECK(property_set_u16(i, 42));
  CHECK(g_properties->key_press == 42);
  CHECK(property_reset(i));
  CHECK(g_properties->key_press == 1900);

  CHECK(property_set_u16(idx("bellow_center"), 100));
  property_reset_all();
  CHECK(g_properties->bellow_center == 11280);
}

static void test_pack_unpack_roundtrip(void)
{
  property_reset_all();
  property_set_u16(idx("key_press"), 1500);
  property_set_u16(idx("bellow_center"), 3800);
  property_set_u16(idx("bellow_cc_period_ms"), 20);

  uint8_t buf[256];
  size_t n = property_pack(buf, sizeof(buf));
  CHECK(n == property_blob_size());
  CHECK(n > 0);

  property_reset_all(); /* wipe back to defaults */
  CHECK(g_properties->key_press == 1900);

  CHECK(property_unpack(buf, n));
  CHECK(g_properties->key_press == 1500);
  CHECK(g_properties->bellow_center == 3800);
  CHECK(g_properties->bellow_cc_period_ms == 20);
  CHECK(g_properties->key_release == 2100); /* untouched -> default */

  /* Too-small buffer yields 0 bytes and writes nothing. */
  CHECK(property_pack(buf, 4) == 0);
}

static void test_corruption_rejected(void)
{
  property_reset_all();
  property_set_u16(idx("key_press"), 1500);
  uint8_t buf[256];
  size_t n = property_pack(buf, sizeof(buf));

  property_reset_all();
  buf[10] ^= 0xff; /* flip a payload byte; checksum no longer matches */
  CHECK(!property_unpack(buf, n));
  CHECK(g_properties->key_press == 1900); /* state untouched on rejection */

  /* Truncated blob is rejected too. */
  CHECK(!property_unpack(buf, 3));
}

/* Little-endian helpers mirroring the blob format, for hand-crafting blobs. */
static void put16(uint8_t *b, size_t off, uint16_t v) { b[off] = v & 0xff; b[off + 1] = v >> 8; }

static void test_out_of_range_loads_default(void)
{
  /* Hand-build a valid-checksum blob: header + one pair (tag 1 = key_press,
   * value 9999 which is above its max 4095) + checksum. */
  uint8_t buf[64];
  size_t off = 0;
  put16(buf, off, 0x4e44); off += 2; /* magic low  */
  put16(buf, off, 0x4241); off += 2; /* magic high */
  put16(buf, off, 1);      off += 2; /* version    */
  put16(buf, off, 1);      off += 2; /* count = 1  */
  put16(buf, off, 1);      off += 2; /* tag = 1 (key_press) */
  put16(buf, off, 9999);   off += 2; /* value (out of range) */
  uint16_t cs = 0;
  for (size_t o = 0; o < off; o += 2) cs ^= (uint16_t)(buf[o] | (buf[o + 1] << 8));
  put16(buf, off, cs); off += 2;

  property_reset_all();
  CHECK(property_unpack(buf, off));
  CHECK(g_properties->key_press == 1900); /* out-of-range -> default, not 9999/4095 */
}

static void test_forward_compat_unknown_tag(void)
{
  /* Blob with an unknown tag (50) plus a known one (tag 3 = bellow_center). */
  uint8_t buf[64];
  size_t off = 0;
  put16(buf, off, 0x4e44); off += 2;
  put16(buf, off, 0x4241); off += 2;
  put16(buf, off, 1);      off += 2; /* version */
  put16(buf, off, 2);      off += 2; /* count = 2 */
  put16(buf, off, 50);     off += 2; /* unknown tag */
  put16(buf, off, 1234);   off += 2;
  put16(buf, off, 3);      off += 2; /* bellow_center */
  put16(buf, off, 3900);   off += 2;
  uint16_t cs = 0;
  for (size_t o = 0; o < off; o += 2) cs ^= (uint16_t)(buf[o] | (buf[o + 1] << 8));
  put16(buf, off, cs); off += 2;

  property_reset_all();
  CHECK(property_unpack(buf, off));
  CHECK(g_properties->bellow_center == 3900); /* known tag applied */
  CHECK(g_properties->key_press == 1900);   /* absent -> default */
}

static void test_flash_stubs(void)
{
  CHECK(!property_load_from_flash());
  CHECK(!property_save_to_flash());
}

static void test_complete(void)
{
  const char *out[64];
  /* "bellow_" matches the bellow_* properties (calibration, CC, scale, inertia) */
  CHECK(properties_complete("bellow_", out, 64) == 20);
  /* "key_" matches the two key_* properties, not keyboard_tuning */
  CHECK(properties_complete("key_", out, 64) == 2);
  /* empty prefix matches all (buffer is sized above property_count()) */
  CHECK(properties_complete("", out, 64) == property_count());
  /* cap is honored */
  CHECK(properties_complete("", out, 3) == 3);
  /* no match */
  CHECK(properties_complete("nope", out, 64) == 0);
}

int main(void)
{
  test_defaults();
  test_lookup();
  test_set_clamp();
  test_type_guards();
  test_reset();
  test_pack_unpack_roundtrip();
  test_corruption_rejected();
  test_out_of_range_loads_default();
  test_forward_compat_unknown_tag();
  test_flash_stubs();
  test_complete();

  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
