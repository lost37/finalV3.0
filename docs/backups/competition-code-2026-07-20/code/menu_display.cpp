#include "menu_display.h"

#include "menu_render.h"
#include "zf_common_headfile.h"

namespace
{
    uint8_t g_menu_display_dirty = 1;
    uint8_t g_menu_display_inited = 0;
}

void MenuDisplay_Init(void)
{
    ips200_init("/dev/fb0");
    ips200_clear();
    g_menu_display_dirty = 1;
    g_menu_display_inited = 1;
}

void MenuDisplay_RequestRefresh(void)
{
    g_menu_display_dirty = 1;
}

uint8_t MenuDisplay_DrawIfNeeded(MenuCore *menu)
{
    if(g_menu_display_inited == 0 || g_menu_display_dirty == 0)
    {
        return 0;
    }

    MenuRenderPage page{};
    if(MenuRender_BuildPage(menu, &page) == 0)
    {
        return 0;
    }

    ips200_clear();
    for(uint8_t i = 0; i < page.line_count; i++)
    {
        ips200_show_string(0, (uint16)(i * 16), page.lines[i]);
    }

    g_menu_display_dirty = 0;
    return 1;
}
