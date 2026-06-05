#ifndef CODE_MENU_RENDER_H_
#define CODE_MENU_RENDER_H_

#include <stdint.h>

#include "menu_core.h"

#define MENU_RENDER_MAX_LINES 8
#define MENU_RENDER_LINE_LEN 32

typedef struct
{
    char lines[MENU_RENDER_MAX_LINES][MENU_RENDER_LINE_LEN];
    uint8_t line_count;
} MenuRenderPage;

uint8_t MenuRender_BuildPage(MenuCore *menu, MenuRenderPage *page);

#endif /* CODE_MENU_RENDER_H_ */
