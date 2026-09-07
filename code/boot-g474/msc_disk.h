#ifndef BOOT_MSC_DISK_H
#define BOOT_MSC_DISK_H

#include <stdbool.h>

/* State of the UF2 image being assembled by the MSC write callbacks.
 * See msc_disk.c for the staging scheme. */

/* True once every block of a complete image has been received and the staged
 * copy is ready to be written to flash. */
bool uf2_commit_ready(void);

/* True while blocks are arriving but the image is not yet complete. */
bool uf2_receiving(void);

/* Write the staged image to the application region and clear the ready flag.
 * Returns false if programming failed. Takes on the order of a second. */
bool uf2_commit(void);

#endif /* BOOT_MSC_DISK_H */
