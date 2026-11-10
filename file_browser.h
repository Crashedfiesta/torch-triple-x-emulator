#ifndef TORCH_FILE_BROWSER_H
#define TORCH_FILE_BROWSER_H
#define FB_PATH 4096
#define FB_LIMIT 2048
typedef struct { char *name; int directory; } BrowserEntry;
typedef struct {
    char directory[FB_PATH], selection[FB_PATH], message[256];
    BrowserEntry entries[FB_LIMIT];
    int count, open;
} FileBrowser;
void file_browser_init(FileBrowser *b);
int file_browser_scan(FileBrowser *b, const char *path);
int file_browser_choose(FileBrowser *b, int index);
void file_browser_up(FileBrowser *b);
void file_browser_free(FileBrowser *b);
#endif
