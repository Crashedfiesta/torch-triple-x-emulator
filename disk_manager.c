#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "disk_manager.h"
#include "disk_status.h"
#include "floppy_media.h"
#include "file_browser.h"
#if !SDL_VERSION_ATLEAST(2,0,18)
#error "The Nuklear SDL renderer requires SDL 2.0.18 or later"
#endif
#define NK_INCLUDE_FIXED_TYPES
#define NK_INCLUDE_STANDARD_IO
#define NK_INCLUDE_STANDARD_VARARGS
#define NK_INCLUDE_DEFAULT_ALLOCATOR
#define NK_INCLUDE_VERTEX_BUFFER_OUTPUT
#define NK_INCLUDE_FONT_BAKING
#define NK_INCLUDE_DEFAULT_FONT
#define NK_IMPLEMENTATION
#include "third_party/nuklear/nuklear.h"
#define NK_SDL_RENDERER_SDL_H <SDL2/SDL.h>
#define NK_SDL_RENDERER_IMPLEMENTATION
#include "third_party/nuklear/nuklear_sdl_renderer.h"

static SDL_Window *window;
static SDL_Renderer *renderer;
static struct nk_context *ctx;
static struct nk_font *heading_font;
static int visible;
static FileBrowser browser;
static char browse_directory[FB_PATH], chosen_path[FB_PATH];
static char notice[512] = "Ready";
static int notice_error, was_pending, dismiss_dialog;
enum Dialog { DIALOG_NONE, DIALOG_PICKER, DIALOG_REPLACE, DIALOG_EJECT, DIALOG_DETAILS };
static enum Dialog dialog;
static const char *drive_names[] = {"System disk", "Additional disk", "Key disk", "Unix floppy"};
static const struct nk_color muted = {153, 168, 190, 255};
static const struct nk_color blue = {97, 177, 245, 255};
static const struct nk_color green = {99, 208, 169, 255};
static const struct nk_color amber = {241, 186, 107, 255};

static const char *basename_only(const char *path)
{
    const char *base = path ? path : "";
    for (const char *p=base; *p; ++p) if (*p=='/' || *p=='\\') base=p+1;
    return base;
}
static void capacity(char *out, size_t cap, size_t bytes)
{
    if (bytes >= 1024u*1024u) snprintf(out,cap,"%.1f MiB",(double)bytes/(1024.0*1024.0));
    else snprintf(out,cap,"%.0f KiB",(double)bytes/1024.0);
}
/* Elide filenames to the available pixel width; full paths remain in Details. */
static void filename_label(const char *path, float width)
{
    char text[FB_PATH];
    const char *name=path ? basename_only(path) : "No image inserted";
    size_t n=strlen(name);
    if (n>sizeof(text)-4) n=sizeof(text)-4;
    memcpy(text,name,n);text[n]=0;
    const struct nk_user_font *font=ctx->style.font;
    if (font->width(font->userdata,font->height,text,(int)n)>width) {
        while (n && font->width(font->userdata,font->height,text,(int)n)+
               font->width(font->userdata,font->height,"...",3)>width) {
            --n;
            while (n && ((unsigned char)text[n]&0xc0)==0x80) --n;
        }
        memcpy(text+n,"...",4);
    }
    nk_label_colored(ctx,text,NK_TEXT_LEFT,muted);
}
static int button(const char *label, int primary, int disabled)
{
    int clicked;
    if (primary) {
        nk_style_push_style_item(ctx,&ctx->style.button.normal,nk_style_item_color(nk_rgb(54,111,171)));
        nk_style_push_style_item(ctx,&ctx->style.button.hover,nk_style_item_color(nk_rgb(67,133,199)));
        nk_style_push_style_item(ctx,&ctx->style.button.active,nk_style_item_color(nk_rgb(43,91,145)));
    }
    if (disabled) nk_widget_disable_begin(ctx);
    clicked=nk_button_label(ctx,label);
    if (disabled) nk_widget_disable_end(ctx);
    if (primary) { nk_style_pop_style_item(ctx);nk_style_pop_style_item(ctx);nk_style_pop_style_item(ctx); }
    return clicked && !disabled;
}
static void theme(void)
{
    struct nk_color colors[NK_COLOR_COUNT];
    nk_style_default(ctx);
    for(int i=0;i<NK_COLOR_COUNT;++i) colors[i]=nk_rgb(42,55,74);
    colors[NK_COLOR_TEXT]=nk_rgb(232,238,248);
    colors[NK_COLOR_WINDOW]=nk_rgb(23,31,44);
    colors[NK_COLOR_HEADER]=nk_rgb(31,43,60);
    colors[NK_COLOR_BORDER]=nk_rgb(57,74,98);
    colors[NK_COLOR_BUTTON]=nk_rgb(43,59,81);
    colors[NK_COLOR_BUTTON_HOVER]=nk_rgb(56,76,103);
    colors[NK_COLOR_BUTTON_ACTIVE]=nk_rgb(67,91,124);
    colors[NK_COLOR_SELECT]=nk_rgb(38,54,75);
    colors[NK_COLOR_SELECT_ACTIVE]=nk_rgb(47,91,139);
    colors[NK_COLOR_EDIT]=nk_rgb(18,26,38);
    colors[NK_COLOR_EDIT_CURSOR]=nk_rgb(230,237,249);
    colors[NK_COLOR_SCROLLBAR]=nk_rgb(23,31,44);
    colors[NK_COLOR_SCROLLBAR_CURSOR]=nk_rgb(65,85,111);
    colors[NK_COLOR_SCROLLBAR_CURSOR_HOVER]=nk_rgb(83,107,139);
    colors[NK_COLOR_SCROLLBAR_CURSOR_ACTIVE]=nk_rgb(97,177,245);
    nk_style_from_table(ctx,colors);
    ctx->style.window.padding=nk_vec2(18,16);
    ctx->style.window.spacing=nk_vec2(10,8);
    ctx->style.window.group_padding=nk_vec2(12,9);
    ctx->style.window.rounding=8;
    ctx->style.button.rounding=5;
    ctx->style.button.border=1;
    ctx->style.button.padding=nk_vec2(10,6);
    ctx->style.edit.rounding=4;
}
static void fonts(struct nk_font_atlas *atlas)
{
    struct nk_font *body=NULL;
#ifdef _WIN32
    char path[1024];
    const char *windir=getenv("WINDIR");
    if (windir && snprintf(path,sizeof path,"%s/Fonts/segoeui.ttf",windir)<(int)sizeof path) {
        FILE *f=fopen(path,"rb");
        if(f) { fclose(f); body=nk_font_atlas_add_from_file(atlas,path,17,NULL);
            heading_font=nk_font_atlas_add_from_file(atlas,path,23,NULL); }
    }
#else
    const char *paths[]={"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                         "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"};
    for(size_t i=0;i<sizeof paths/sizeof paths[0];++i) {
        FILE *f=fopen(paths[i],"rb");if(!f) continue;fclose(f);
        body=nk_font_atlas_add_from_file(atlas,paths[i],17,NULL);
        heading_font=nk_font_atlas_add_from_file(atlas,paths[i],23,NULL);
        if(body) break;
    }
#endif
    if(!body) body=nk_font_atlas_add_default(atlas,17,NULL);
    atlas->default_font=body;
    if(!heading_font) heading_font=body;
}
void disk_manager_shutdown(void)
{
    file_browser_free(&browser);
    if(ctx) nk_sdl_shutdown();
    ctx=NULL;heading_font=NULL;
    SDL_DestroyRenderer(renderer);SDL_DestroyWindow(window);
    renderer=NULL;window=NULL;visible=0;dialog=DIALOG_NONE;
}
int disk_manager_init(void)
{
    struct nk_font_atlas *atlas;
    file_browser_init(&browser);
    snprintf(browse_directory,sizeof browse_directory,"%s",browser.directory);
    dialog=DIALOG_NONE;dismiss_dialog=was_pending=notice_error=0;
    snprintf(notice,sizeof notice,"Ready");
    window=SDL_CreateWindow("Torch Disk Manager",SDL_WINDOWPOS_UNDEFINED,
               SDL_WINDOWPOS_UNDEFINED,640,580,SDL_WINDOW_RESIZABLE);
    if(!window) goto fail;
    SDL_SetWindowMinimumSize(window,600,560);
    renderer=SDL_CreateRenderer(window,-1,SDL_RENDERER_ACCELERATED);
    if(!renderer) renderer=SDL_CreateRenderer(window,-1,SDL_RENDERER_SOFTWARE);
    if(!renderer) goto fail;
    ctx=nk_sdl_init(window,renderer);if(!ctx) goto fail;
    nk_sdl_font_stash_begin(&atlas);fonts(atlas);nk_sdl_font_stash_end();
    if(!atlas->default_font || !sdl.ogl.font_tex) goto fail;
    theme();visible=1;return 0;
fail:
    fprintf(stderr,"[DISK MANAGER] Cannot initialise: %s\n",SDL_GetError());
    disk_manager_shutdown();return -1;
}

/* SDL positions refer to the client area; include window decorations when
 * keeping the two outer window rectangles apart. */
typedef struct {
    int top, left, bottom, right;
} WindowBorders;
static WindowBorders manager_borders(SDL_Window *w)
{
    WindowBorders b = {0,0,0,0};
    if (SDL_GetWindowBordersSize(w,&b.top,&b.left,&b.bottom,&b.right) < 0)
        b = (WindowBorders){0,0,0,0};
    return b;
}
static int manager_fits(SDL_Rect r, SDL_Rect area)
{
    return r.x >= area.x && r.y >= area.y &&
        r.x+r.w <= area.x+area.w && r.y+r.h <= area.y+area.h;
}
static int manager_clamp(int value, int low, int high)
{
    if (high < low) return low;
    return value < low ? low : value > high ? high : value;
}
static void manager_move_outer(SDL_Window *w, WindowBorders b, int x, int y)
{
    SDL_SetWindowPosition(w,x+b.left,y+b.top);
}
void disk_manager_place_beside(SDL_Window *emulator)
{
    const int gap = 12;
    SDL_Rect area, torch, manager;
    WindowBorders tb, mb;
    int display, pair_w, pair_h, x, y;
    if (!window || !emulator) return;
    display = SDL_GetWindowDisplayIndex(emulator);
    if (display < 0) return;
    if (SDL_GetDisplayUsableBounds(display,&area) < 0 &&
        SDL_GetDisplayBounds(display,&area) < 0) return;
    tb = manager_borders(emulator); mb = manager_borders(window);
    SDL_GetWindowPosition(emulator,&torch.x,&torch.y);
    SDL_GetWindowSize(emulator,&torch.w,&torch.h);
    SDL_GetWindowSize(window,&manager.w,&manager.h);
    torch.x -= tb.left; torch.y -= tb.top;
    torch.w += tb.left+tb.right; torch.h += tb.top+tb.bottom;
    manager.w += mb.left+mb.right; manager.h += mb.top+mb.bottom;
    manager.y = manager_clamp(torch.y,area.y,area.y+area.h-manager.h);

    /* First leave the Torch where the desktop placed it. */
    manager.x = torch.x+torch.w+gap;
    if (manager_fits(manager,area)) goto place_manager;
    manager.x = torch.x-manager.w-gap;
    if (manager_fits(manager,area)) goto place_manager;

    /* The pair may fit even if the desktop initially centred the Torch. */
    pair_w = torch.w+gap+manager.w;
    pair_h = torch.h > manager.h ? torch.h : manager.h;
    if (pair_w <= area.w && pair_h <= area.h) {
        x = area.x+(area.w-pair_w)/2;
        y = area.y+(area.h-pair_h)/2;
        manager_move_outer(emulator,tb,x,y);
        manager.x = x+torch.w+gap; manager.y = y;
        goto place_manager;
    }

    /* Try above/below without moving the Torch, then centre a vertical pair. */
    manager.x = manager_clamp(torch.x,area.x,area.x+area.w-manager.w);
    manager.y = torch.y+torch.h+gap;
    if (manager_fits(manager,area)) goto place_manager;
    manager.y = torch.y-manager.h-gap;
    if (manager_fits(manager,area)) goto place_manager;
    pair_h = torch.h+gap+manager.h;
    pair_w = torch.w > manager.w ? torch.w : manager.w;
    if (pair_h <= area.h && pair_w <= area.w) {
        x = area.x+(area.w-pair_w)/2;
        y = area.y+(area.h-pair_h)/2;
        manager_move_outer(emulator,tb,x,y);
        manager.x = x; manager.y = y+torch.h+gap;
        goto place_manager;
    }

    /* No complete non-overlapping arrangement fits: keep the manager reachable
     * at the right edge, and leave the Torch's position and both sizes alone. */
    manager.x = manager_clamp(area.x+area.w-manager.w,area.x,area.x+area.w-manager.w);
    manager.y = manager_clamp(torch.y,area.y,area.y+area.h-manager.h);
place_manager:
    manager_move_outer(window,mb,manager.x,manager.y);
}

void disk_manager_toggle(void)
{
    if(!window) return;
    visible=!visible;
    if(visible) { SDL_ShowWindow(window);SDL_RaiseWindow(window); }
    else SDL_HideWindow(window);
}
void disk_manager_input_begin(void) { if(ctx) nk_input_begin(ctx); }
void disk_manager_input_end(void) { if(ctx) nk_input_end(ctx); }
int disk_manager_event(SDL_Event *event)
{
    Uint32 id;
    if(!window) return 0;
    switch(event->type) {
    case SDL_WINDOWEVENT:id=event->window.windowID;break;
    case SDL_KEYDOWN:case SDL_KEYUP:id=event->key.windowID;break;
    case SDL_TEXTINPUT:id=event->text.windowID;break;
    case SDL_TEXTEDITING:id=event->edit.windowID;break;
    case SDL_MOUSEMOTION:id=event->motion.windowID;break;
    case SDL_MOUSEBUTTONDOWN:case SDL_MOUSEBUTTONUP:id=event->button.windowID;break;
    case SDL_MOUSEWHEEL:id=event->wheel.windowID;break;
    default:return 0;
    }
    if(id!=SDL_GetWindowID(window)) return 0;
    if((event->type==SDL_KEYDOWN || event->type==SDL_KEYUP) && event->key.keysym.sym==SDLK_ESCAPE) {
        if(event->type==SDL_KEYDOWN && !event->key.repeat) {
            if(dialog!=DIALOG_NONE) dismiss_dialog=1;
            else {visible=0;SDL_HideWindow(window);}
        }
        return 1;
    }
    if(event->type==SDL_WINDOWEVENT && event->window.event==SDL_WINDOWEVENT_CLOSE) {
        visible=0;SDL_HideWindow(window);
    } else if(event->type==SDL_WINDOWEVENT && event->window.event==SDL_WINDOWEVENT_FOCUS_GAINED) {
        SDL_StartTextInput();
    } else if(event->type==SDL_WINDOWEVENT && event->window.event==SDL_WINDOWEVENT_FOCUS_LOST) {
        SDL_StopTextInput();
        for(int k=0;k<NK_KEY_MAX;++k) nk_input_key(ctx,(enum nk_keys)k,0);
        for(int k=0;k<NK_BUTTON_MAX;++k) nk_input_button(ctx,(enum nk_buttons)k,0,0,0);
    } else if(visible) nk_sdl_handle_event(event);
    return 1;
}
static void outcome(int result)
{
    notice_error=result!=0;
    snprintf(notice,sizeof notice,"%s",ncr_floppy_message());
    if(!result) was_pending=1;
}
static void close_popup(void)
{
    dialog=DIALOG_NONE;dismiss_dialog=0;nk_popup_close(ctx);
}
static void open_picker(void)
{
    file_browser_scan(&browser,browser.directory);
    snprintf(browse_directory,sizeof browse_directory,"%s",browser.directory);
    dialog=DIALOG_PICKER;
}
static void choose_image(int loaded)
{
    if(!browser.selection[0] || ncr_floppy_pending()) return;
    snprintf(chosen_path,sizeof chosen_path,"%s",browser.selection);
    if(loaded) dialog=DIALOG_REPLACE;
    else {
        int result=ncr_floppy_insert(chosen_path);outcome(result);
        if(!result) close_popup();
    }
}
static void draw_drive(int index,const DiskImageStatus *s,float width)
{
    char summary[96],size[40];
    capacity(size,sizeof size,s->size_bytes);
    if(!s->loaded) snprintf(summary,sizeof summary,"Empty");
    else snprintf(summary,sizeof summary,"%s / %s",size,index==DISK_STATUS_KEY ? "Memory only" : s->file_writable ? "Writable" : "Read-only");
    nk_layout_row_dynamic(ctx,74,1);
    if(nk_group_begin(ctx,drive_names[index],NK_WINDOW_BORDER|NK_WINDOW_NO_SCROLLBAR)) {
        const float columns[]={0.44f,0.56f};
        nk_layout_row(ctx,NK_DYNAMIC,22,2,columns);
        struct nk_rect label_bounds;
        nk_widget(&label_bounds,ctx);
        struct nk_color light = s->activity == DISK_ACTIVITY_WRITE ? amber :
            s->activity == DISK_ACTIVITY_READ ? green : nk_rgb(68,82,102);
        float cy = label_bounds.y + (label_bounds.h - 9) * 0.5f;
        nk_fill_circle(nk_window_get_canvas(ctx),nk_rect(label_bounds.x,cy,9,9),light);
        struct nk_rect text_bounds = label_bounds;
        text_bounds.x += 18;
        text_bounds.w -= 18;
        nk_draw_text(nk_window_get_canvas(ctx),text_bounds,drive_names[index],
                     (int)strlen(drive_names[index]),ctx->style.font,
                     ctx->style.window.background,ctx->style.text.color);
        nk_label_colored(ctx,summary,NK_TEXT_RIGHT,s->loaded?green:muted);
        nk_layout_row_dynamic(ctx,22,1);
        filename_label(s->loaded?s->path:NULL,width-85);
        nk_group_end(ctx);
    }
}
static void draw_dialog(const DiskImageStatus status[DISK_STATUS_COUNT],int w,int h)
{
    float pw=(float)w-76,ph=(float)h-76,py=8;
    if(dialog==DIALOG_NONE) return;
    if(dialog==DIALOG_EJECT || dialog==DIALOG_REPLACE) {
        ph=dialog==DIALOG_REPLACE?340:260;
        py=((float)h-ph)*0.5f-16;
    }
    nk_fill_rect(nk_window_get_canvas(ctx),nk_rect(0,0,(float)w,(float)h),0,nk_rgba(8,12,20,170));
    if(!nk_popup_begin(ctx,NK_POPUP_STATIC,"Disk action",NK_WINDOW_NO_SCROLLBAR,nk_rect(12,py,pw,ph))) return;
    if(dismiss_dialog) {
        close_popup();nk_popup_end(ctx);return;
    }
    if(dialog==DIALOG_PICKER) {
        int chosen=-1;
        nk_style_push_font(ctx,&heading_font->handle);
        nk_layout_row_dynamic(ctx,30,1);nk_label(ctx,"Choose a Unix floppy image",NK_TEXT_LEFT);
        nk_style_pop_font(ctx);
        nk_layout_row_dynamic(ctx,20,1);nk_label_colored(ctx,"Raw images / select a file, then Open",NK_TEXT_LEFT,muted);
        nk_layout_row_begin(ctx,NK_DYNAMIC,32,3);
        nk_layout_row_push(ctx,0.72f);
        nk_edit_string_zero_terminated(ctx,NK_EDIT_FIELD,browse_directory,sizeof browse_directory,nk_filter_default);
        nk_layout_row_push(ctx,0.15f);
        if(button("Go",0,0)) {file_browser_scan(&browser,browse_directory);snprintf(browse_directory,sizeof browse_directory,"%s",browser.directory);}
        nk_layout_row_push(ctx,0.13f);
        if(button("Up",0,0)) {file_browser_up(&browser);snprintf(browse_directory,sizeof browse_directory,"%s",browser.directory);}
        nk_layout_row_end(ctx);
        nk_layout_row_dynamic(ctx,ph-278,1);
        if(nk_group_begin(ctx,"Files",NK_WINDOW_BORDER)) {
            for(int i=0;i<browser.count;++i) {
                char label[512];nk_bool selected=0;
                const BrowserEntry *entry=&browser.entries[i];
                snprintf(label,sizeof label,"%s%s",entry->directory?"[Folder] ":"",entry->name);
                selected=!entry->directory && browser.selection[0] && !strcmp(basename_only(browser.selection),entry->name);
                nk_layout_row_dynamic(ctx,28,1);
                if(nk_selectable_label(ctx,label,NK_TEXT_LEFT,&selected)) chosen=i;
            }
            nk_group_end(ctx);
        }
        if(chosen>=0) {file_browser_choose(&browser,chosen);snprintf(browse_directory,sizeof browse_directory,"%s",browser.directory);}
        nk_layout_row_dynamic(ctx,24,1);
        filename_label(browser.selection[0]?browser.selection:"Select an image",pw-70);
        nk_layout_row_dynamic(ctx,36,1);
        nk_label_wrap(ctx,browser.message[0]?browser.message:notice_error?notice:"Folders open with one click. Escape cancels.");
        nk_layout_row_dynamic(ctx,34,2);
        if(button("Cancel",0,0)) close_popup();
        if(button("Open",1,!browser.selection[0]||ncr_floppy_pending())) choose_image(status[DISK_STATUS_UNIX].loaded);
    } else if(dialog==DIALOG_REPLACE || dialog==DIALOG_EJECT) {
        int replacing=dialog==DIALOG_REPLACE;
        nk_style_push_font(ctx,&heading_font->handle);
        nk_layout_row_dynamic(ctx,34,1);
        nk_label(ctx,replacing?"Replace this floppy?":"Eject this floppy?",NK_TEXT_LEFT);
        nk_style_pop_font(ctx);
        nk_layout_row_dynamic(ctx,64,1);
        nk_label_wrap(ctx,"Unmount the floppy in Torch Unix before continuing. The emulator cannot check whether Unix still has it mounted.");
        nk_layout_row_dynamic(ctx,28,1);nk_label_colored(ctx,"Current image",NK_TEXT_LEFT,muted);
        nk_layout_row_dynamic(ctx,30,1);filename_label(status[DISK_STATUS_UNIX].path,pw-70);
        if(replacing) {
            nk_layout_row_dynamic(ctx,28,1);nk_label_colored(ctx,"Replacement",NK_TEXT_LEFT,muted);
            nk_layout_row_dynamic(ctx,30,1);filename_label(chosen_path,pw-70);
        }
        nk_layout_row_dynamic(ctx,36,2);
        if(button("Cancel",0,0)) close_popup();
        if(button("Continue",1,ncr_floppy_pending())) {
            int result=replacing?ncr_floppy_insert(chosen_path):ncr_floppy_eject();outcome(result);
            if(!result) close_popup();
            else if(replacing) dialog=DIALOG_PICKER;
            else close_popup();
        }
    } else if(dialog==DIALOG_DETAILS) {
        nk_style_push_font(ctx,&heading_font->handle);
        nk_layout_row_dynamic(ctx,32,1);nk_label(ctx,"Image details",NK_TEXT_LEFT);nk_style_pop_font(ctx);
        nk_layout_row_dynamic(ctx,ph-130,1);
        nk_style_push_vec2(ctx,&ctx->style.window.spacing,nk_vec2(10,4));
        if(nk_group_begin(ctx,"Full paths",0)) {
            for(int i=0;i<DISK_STATUS_COUNT;++i) {
                char detail[160],fullpath[FB_PATH];
                nk_layout_row_dynamic(ctx,22,1);nk_label_colored(ctx,drive_names[i],NK_TEXT_LEFT,blue);
                snprintf(fullpath,sizeof fullpath,"%s",status[i].path?status[i].path:"No image loaded");
                nk_layout_row_dynamic(ctx,30,1);
                nk_edit_string_zero_terminated(ctx,NK_EDIT_FIELD|NK_EDIT_READ_ONLY,fullpath,sizeof fullpath,nk_filter_default);
                snprintf(detail,sizeof detail,"%zu bytes%s",status[i].size_bytes,i==DISK_STATUS_KEY?" / IMD changes are not saved":"");
                nk_layout_row_dynamic(ctx,20,1);nk_label_colored(ctx,detail,NK_TEXT_LEFT,muted);
            }
            nk_group_end(ctx);
        }
        nk_style_pop_vec2(ctx);
        nk_layout_row_dynamic(ctx,34,1);if(button("Done",1,0)) close_popup();
    }
    nk_popup_end(ctx);
}
void disk_manager_render(void)
{
    DiskImageStatus status[DISK_STATUS_COUNT];int w,h,hide=0;
    if(!ctx||!visible) return;
    ncr_get_disk_status(status);int pending=ncr_floppy_pending();
    if(was_pending && !pending) {snprintf(notice,sizeof notice,"%s",ncr_floppy_message());notice_error=strstr(notice,"cancelled")!=NULL;was_pending=0;}
    SDL_GetWindowSize(window,&w,&h);
    if(nk_begin(ctx,"Disk manager",nk_rect(0,0,(float)w,(float)h),NK_WINDOW_NO_SCROLLBAR)) {
        nk_style_push_font(ctx,&heading_font->handle);
        nk_layout_row_dynamic(ctx,30,1);nk_label(ctx,"Torch Triple X",NK_TEXT_LEFT);nk_style_pop_font(ctx);
        const float legend_columns[] = {0.55f,0.15f,0.15f,0.15f};
        nk_layout_row(ctx,NK_DYNAMIC,20,4,legend_columns);
        nk_label_colored(ctx,"DISK MANAGER",NK_TEXT_LEFT,blue);
        nk_label_colored(ctx,"Read",NK_TEXT_RIGHT,green);
        nk_label_colored(ctx,"Write",NK_TEXT_RIGHT,amber);
        nk_label_colored(ctx,"Idle",NK_TEXT_RIGHT,muted);
        for(int i=0;i<DISK_STATUS_COUNT;++i) {
            if(i==DISK_STATUS_UNIX) {
                /* Separate removable media from the fixed/key disk cards. */
                struct nk_rect gap;
                nk_layout_row_dynamic(ctx,12,1);
                nk_widget(&gap,ctx);
                nk_stroke_line(nk_window_get_canvas(ctx),gap.x,gap.y+6,
                               gap.x+gap.w,gap.y+6,1,nk_rgb(57,74,98));
            }
            draw_drive(i,&status[i],(float)w);
        }
        nk_layout_row_begin(ctx,NK_DYNAMIC,36,3);
        nk_layout_row_push(ctx,0.50f);
        if(button(status[DISK_STATUS_UNIX].loaded?"Change floppy...":"Insert floppy...",1,pending)) open_picker();
        nk_layout_row_push(ctx,0.25f);
        if(button("Eject",0,pending||!status[DISK_STATUS_UNIX].loaded)) dialog=DIALOG_EJECT;
        nk_layout_row_push(ctx,0.25f);
        if(button("Details",0,0)) {
            dialog=DIALOG_DETAILS;
            /* Consume the click so it cannot also activate Done. */
            ctx->input.mouse.buttons[NK_BUTTON_LEFT].clicked=0;
        }
        nk_layout_row_end(ctx);
        nk_layout_row_dynamic(ctx,38,1);
        nk_label_colored_wrap(ctx,pending?"Waiting for SCSI to finish...":notice,notice_error?amber:muted);
        nk_layout_row_begin(ctx,NK_DYNAMIC,28,2);
        nk_layout_row_push(ctx,0.78f);
        nk_label_colored(ctx,"F12 shows or hides this window",NK_TEXT_LEFT,muted);
        nk_layout_row_push(ctx,0.22f);
        hide=button("Close",0,0);
        nk_layout_row_end(ctx);
        draw_dialog(status,w,h);
    }
    nk_end(ctx);
    SDL_SetRenderDrawColor(renderer,23,31,44,255);SDL_RenderClear(renderer);
    nk_sdl_render(NK_ANTI_ALIASING_ON);SDL_RenderPresent(renderer);
    if(hide) {visible=0;SDL_HideWindow(window);}
}
