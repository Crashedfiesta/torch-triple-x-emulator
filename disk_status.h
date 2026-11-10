#ifndef TORCH_DISK_STATUS_H
#define TORCH_DISK_STATUS_H
#include <stddef.h>
enum {
    DISK_STATUS_MAIN,
    DISK_STATUS_EXTRA,
    DISK_STATUS_KEY,
    DISK_STATUS_UNIX,
    DISK_STATUS_COUNT
};
enum DiskActivity {
    DISK_ACTIVITY_IDLE,
    DISK_ACTIVITY_READ,
    DISK_ACTIVITY_WRITE
};
typedef struct {
    const char *path;       /* Borrowed pointer; NULL when no image loaded. */
    size_t size_bytes;      /* Decoded capacity for the IMD key disk. */
    int loaded;
    int file_writable;     /* Writes can persist to the image file. */
    int activity;          /* enum DiskActivity; includes a 150 ms hold. */
} DiskImageStatus;
/* Call on the emulator/main thread. Returned paths remain emulator-owned. */
void ncr_get_disk_status(DiskImageStatus out[DISK_STATUS_COUNT]);
#endif
