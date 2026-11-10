/* Included once by triplex.c, after the image-loading helpers.
 * Uses the existing SCSI image variables; this is not a separate translation unit. */
typedef struct {
    uint8_t *data;
    size_t size;
    int fd, writable;
    char *path;
} FloppyMedia;
static FloppyMedia floppy_next = {NULL, 0, -1, 0, NULL};
static char *floppy_owned_path;
static int floppy_request; /* 0 none, 1 insert, 2 eject */
static int floppy_attention;
static uint8_t floppy_sense_key, floppy_sense_asc;
static char floppy_message[512] = "Unmount the Unix floppy in Torch before changing images.";

static void floppy_discard(FloppyMedia *m)
{
#ifdef _WIN32
    free(m->data);
    if (m->fd >= 0) _close(m->fd);
#else
    if (m->data) munmap(m->data, m->size);
    if (m->fd >= 0) close(m->fd);
#endif
    free(m->path);
    *m = (FloppyMedia){NULL, 0, -1, 0, NULL};
}

/* Stage the entire new mapping/buffer before touching the current medium. */
static int floppy_prepare(const char *path, FloppyMedia *m)
{
    struct stat st;
    size_t n;
    if (!path || !*path || stat(path, &st) || !S_ISREG(st.st_mode)) {
        snprintf(floppy_message, sizeof floppy_message, "Cannot open a regular image file: %.350s", path ? path : "");
        return -1;
    }
    if (st.st_size <= 0 || st.st_size > 32 * 1024 * 1024 || st.st_size % 512) {
        snprintf(floppy_message, sizeof floppy_message, "Use a raw image: positive size, multiple of 512 bytes, at most 32 MiB.");
        return -1;
    }
    /* Avoid loading the same backing file twice, including Linux hard links. */
    for (int i = 0; i < DISK_STATUS_COUNT; ++i) {
        if (!g_disk_paths[i]) continue;
#ifdef _WIN32
        char a[_MAX_PATH], b[_MAX_PATH];
        if (_fullpath(a, path, sizeof a) && _fullpath(b, g_disk_paths[i], sizeof b) && !_stricmp(a,b)) {
#else
        struct stat other;
        if (!stat(g_disk_paths[i], &other) && st.st_dev == other.st_dev && st.st_ino == other.st_ino) {
#endif
            snprintf(floppy_message, sizeof floppy_message, "That file is already loaded. Choose another image or eject it first.");
            return -1;
        }
    }
    n = strlen(path) + 1;
    m->path = malloc(n);
    if (!m->path) goto fail;
    memcpy(m->path, path, n);
#ifdef _WIN32
    m->data = host_load_image(path, &m->size, &m->fd, &m->writable);
    if (!m->data) goto fail;
#else
    m->fd = open(path, O_RDWR);
    m->writable = m->fd >= 0;
    if (m->fd < 0) m->fd = open(path, O_RDONLY);
    if (m->fd < 0 || fstat(m->fd, &st) || !S_ISREG(st.st_mode) ||
        st.st_size <= 0 || st.st_size > 32 * 1024 * 1024 || st.st_size % 512) goto fail;
    m->size = (size_t)st.st_size;
    m->data = mmap(NULL, m->size, m->writable ? PROT_READ | PROT_WRITE : PROT_READ,
                   m->writable ? MAP_SHARED : MAP_PRIVATE, m->fd, 0);
    if (m->data == MAP_FAILED) { m->data = NULL; goto fail; }
#endif
    if (!m->size || m->size > 32 * 1024 * 1024 || m->size % 512) goto fail;
    if ((m->size >= 4 && !memcmp(m->data, "IMD ", 4)) ||
        (m->data[0] == 0x1f && m->data[1] == 0x8b)) {
        floppy_discard(m);
        snprintf(floppy_message, sizeof floppy_message, "IMD and gzip files are not raw Unix floppy images.");
        return -1;
    }
    return 0;
fail:
    floppy_discard(m);
    snprintf(floppy_message, sizeof floppy_message, "Unable to load image; current floppy kept. Check the path and file permissions.");
    return -1;
}

int ncr_floppy_pending(void) { return floppy_request != 0; }
const char *ncr_floppy_message(void) { return floppy_message; }
int ncr_floppy_insert(const char *path)
{
    if (floppy_request) return -1;
    if (floppy_prepare(path, &floppy_next)) return -1;
    floppy_request = 1;
    snprintf(floppy_message, sizeof floppy_message, "Insert queued: waiting for SCSI bus idle.");
    return 0;
}
int ncr_floppy_eject(void)
{
    if (floppy_request) return -1;
    if (!g_unix_floppy_image) {
        snprintf(floppy_message, sizeof floppy_message, "The Unix floppy drive is already empty.");
        return -1;
    }
    floppy_request = 2;
    snprintf(floppy_message, sizeof floppy_message, "Eject queued: waiting for SCSI bus idle.");
    return 0;
}

static int floppy_release_current(void)
{
    if (!g_unix_floppy_image) return 0;
#ifdef _WIN32
    if (g_unix_floppy_writeable && g_unix_floppy_fd >= 0 && _commit(g_unix_floppy_fd)) return -1;
    free(g_unix_floppy_image);
    if (g_unix_floppy_fd >= 0) _close(g_unix_floppy_fd);
#else
    if (g_unix_floppy_writeable &&
        (msync(g_unix_floppy_image, g_unix_floppy_size, MS_SYNC) ||
         (g_unix_floppy_fd >= 0 && fsync(g_unix_floppy_fd)))) return -1;
    if (munmap(g_unix_floppy_image, g_unix_floppy_size)) return -1;
    if (g_unix_floppy_fd >= 0) close(g_unix_floppy_fd);
#endif
    g_unix_floppy_fd = -1;
    g_unix_floppy_image = NULL;
    g_unix_floppy_size = 0;
    g_unix_floppy_writeable = 0;
    g_disk_paths[DISK_STATUS_UNIX] = NULL;
    free(floppy_owned_path);
    floppy_owned_path = NULL;
    return 0;
}

void ncr_floppy_service(void)
{
    int request;
    /* Test register bits here because this include precedes NCR_ICR defines. */
    if (!floppy_request || g_ncr.phase != PHASE_BUS_FREE || g_ncr.bsy_target ||
        (g_ncr.icr & (0x04 | 0x10 | 0x80))) return; /* SEL, ACK or RESET still asserted */
    request = floppy_request;
    if (floppy_release_current()) {
        floppy_discard(&floppy_next);
        floppy_request = 0;
        snprintf(floppy_message, sizeof floppy_message, "Could not flush/release current floppy; change cancelled and current image retained.");
        return;
    }
    if (request == 1) {
        g_unix_floppy_image = floppy_next.data;
        g_unix_floppy_size = floppy_next.size;
        g_unix_floppy_fd = floppy_next.fd;
        g_unix_floppy_writeable = floppy_next.writable;
        floppy_owned_path = floppy_next.path;
        g_disk_paths[DISK_STATUS_UNIX] = floppy_owned_path;
        floppy_next = (FloppyMedia){NULL, 0, -1, 0, NULL};
    }
    ncr_activity_clear(DISK_STATUS_UNIX);
    g_ncr.data_buf = NULL;
    g_ncr.data_len = 0;
    floppy_attention = 1;
    floppy_sense_key = floppy_sense_asc = 0;
    floppy_request = 0;
    snprintf(floppy_message, sizeof floppy_message, "%s", request == 1 ? "Unix floppy inserted." : "Unix floppy ejected.");
}

void ncr_load_unix_floppy(const char *path)
{
    /* Startup uses the same validated loader, without runtime Unit Attention. */
    if (!ncr_floppy_insert(path)) {
        ncr_floppy_service();
        floppy_attention = 0;
    }
    fprintf(stderr, "[UNIX-FLOPPY] %s\n", floppy_message);
}
void ncr_floppy_shutdown(void)
{
    floppy_discard(&floppy_next);
    floppy_request = 0;
    if (floppy_release_current()) fprintf(stderr, "[UNIX-FLOPPY] Final flush failed\n");
}

/* Handle media-related errors for ID 0/LUN 2 only.
 * Return 1 when dispatch should skip the normal CDB implementation. */
static int floppy_media_cdb(int target, int unit, uint8_t op)
{
    static uint8_t sense[18];
    if (target != 0 || unit != 2 || op == 0x12) return 0;
    if (op == 0x03) {
        memset(sense, 0, sizeof sense);
        sense[0] = 0x70; sense[7] = 10;
        sense[2] = floppy_attention ? 0x06 : floppy_sense_key;
        sense[12] = floppy_attention ? 0x28 : floppy_sense_asc;
        floppy_attention = 0;
        floppy_sense_key = floppy_sense_asc = 0;
        g_ncr.data_buf = sense;
        g_ncr.data_len = g_ncr.cmd_bytes[4] < 18 ? g_ncr.cmd_bytes[4] : 18;
        return 1;
    }
    if (floppy_attention) {
        g_ncr.status_byte = 0x02;
        return 1;
    }
    if (!g_unix_floppy_image) {
        floppy_sense_key = 0x02; floppy_sense_asc = 0x3a; /* medium absent */
        g_ncr.status_byte = 0x02;
        return 1;
    }
    if ((op == 0x0a || op == 0x2a) && !g_unix_floppy_writeable) {
        floppy_sense_key = 0x07; floppy_sense_asc = 0x27; /* data protect */
        g_ncr.status_byte = 0x02;
        return 1;
    }
    return 0;
}
