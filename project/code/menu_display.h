#ifndef CODE_MENU_DISPLAY_H_
#define CODE_MENU_DISPLAY_H_

#include <stdint.h>

#include "menu_core.h"

void MenuDisplay_Init(void);
void MenuDisplay_RequestRefresh(void);
uint8_t MenuDisplay_DrawIfNeeded(MenuCore *menu);

#endif /* CODE_MENU_DISPLAY_H_ */
