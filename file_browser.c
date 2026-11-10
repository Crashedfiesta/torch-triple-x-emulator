#include "file_browser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#define fb_getcwd _getcwd
#else
#include <dirent.h>
#include <unistd.h>
#define fb_getcwd getcwd
#endif
static int join_path(char *out, const char *dir, const char *name)
{
    size_t n = strlen(dir);
    int sep = n && dir[n-1] != '/' && dir[n-1] != '\\';
    return snprintf(out, FB_PATH, "%s%s%s", dir, sep ? "/" : "", name) < FB_PATH ? 0 : -1;
}
void file_browser_free(FileBrowser *b)
{
    for (int i=0; i<b->count; ++i) free(b->entries[i].name);
    b->count=0;
}
static void add_entry(FileBrowser *b, const char *name, int directory)
{
    if (!strcmp(name,".") || !strcmp(name,"..")) return;
    if (b->count == FB_LIMIT) { snprintf(b->message,sizeof b->message,"Listing limited to %d entries.", FB_LIMIT); return; }
    char *copy = malloc(strlen(name)+1);
    if (!copy) { snprintf(b->message,sizeof b->message,"Not enough memory for listing."); return; }
    strcpy(copy,name);
    b->entries[b->count++] = (BrowserEntry){copy,directory};
}
static int compare_entry(const void *a,const void *b)
{
    const BrowserEntry *x=a,*y=b;
    return x->directory != y->directory ? y->directory-x->directory : strcmp(x->name,y->name);
}
int file_browser_scan(FileBrowser *b, const char *path)
{
    char dir[FB_PATH];
    if (!path || !*path || strlen(path)>=sizeof dir) return -1;
    strcpy(dir,path); /* path may alias b->directory */
#ifdef _WIN32
    WIN32_FIND_DATAA entry;
    char pattern[FB_PATH];
    DWORD attrs=GetFileAttributesA(dir);
    if (attrs==INVALID_FILE_ATTRIBUTES || !(attrs & FILE_ATTRIBUTE_DIRECTORY) || join_path(pattern,dir,"*")) goto fail;
    HANDLE d=FindFirstFileA(pattern,&entry);
    if (d==INVALID_HANDLE_VALUE) goto fail;
#else
    DIR *d=opendir(dir);
    if (!d) goto fail;
#endif
    file_browser_free(b);
    strcpy(b->directory,dir);b->selection[0]=0;b->message[0]=0;
#ifdef _WIN32
    do { add_entry(b,entry.cFileName,(entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)!=0); }
    while(FindNextFileA(d,&entry));
    FindClose(d);
#else
    struct dirent *entry;
    while ((entry=readdir(d))) {
        char full[FB_PATH];struct stat st;
        if (!join_path(full,dir,entry->d_name) && !stat(full,&st) && (S_ISREG(st.st_mode)||S_ISDIR(st.st_mode)))
            add_entry(b,entry->d_name,S_ISDIR(st.st_mode));
    }
    closedir(d);
#endif
    qsort(b->entries,b->count,sizeof b->entries[0],compare_entry);
    return 0;
fail:
    snprintf(b->message,sizeof b->message,"Cannot open that directory; previous listing retained.");
    return -1;
}
void file_browser_init(FileBrowser *b)
{
    memset(b,0,sizeof *b);
    if (!fb_getcwd(b->directory,sizeof b->directory)) strcpy(b->directory,".");
    file_browser_scan(b,b->directory);
}
int file_browser_choose(FileBrowser *b,int index)
{
    char full[FB_PATH];
    if (index<0 || index>=b->count) return -1;
    if (join_path(full,b->directory,b->entries[index].name)) {
        snprintf(b->message,sizeof b->message,"Path is too long.");return -1;
    }
    if (b->entries[index].directory) return file_browser_scan(b,full);
    strcpy(b->selection,full);return 0;
}
void file_browser_up(FileBrowser *b)
{
    char parent[FB_PATH];size_t n;
    strcpy(parent,b->directory);n=strlen(parent);
    while(n>1 && (parent[n-1]=='/' || parent[n-1]=='\\')) parent[--n]=0;
    char *a=strrchr(parent,'/'),*c=strrchr(parent,'\\');
    char *p=!a?c:(!c?a:(a>c?a:c));
    if (!p) { file_browser_scan(b,".");return; }
    if (p==parent) p[1]=0;
    else if (p==parent+2 && parent[1]==':') p[1]=0;
    else *p=0;
    file_browser_scan(b,parent);
}
