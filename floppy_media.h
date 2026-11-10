#ifndef TORCH_FLOPPY_MEDIA_H
#define TORCH_FLOPPY_MEDIA_H
/* Main-thread API: requests are validated now, installed only at bus idle. */
int ncr_floppy_insert(const char *path);
int ncr_floppy_eject(void);
int ncr_floppy_pending(void);
const char *ncr_floppy_message(void);
void ncr_floppy_service(void);
void ncr_floppy_shutdown(void);
#endif
