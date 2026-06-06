#ifndef CODE_MENU_APP_H_
#define CODE_MENU_APP_H_

#include <stdint.h>

#include "menu_core.h"

void MenuApp_Init(void);
void MenuApp_HandleAction(MenuCoreAction action);
void MenuApp_DrawIfNeeded(void);
void MenuApp_DrawActiveDisplay(void);
uint8_t MenuApp_IsTuningMode(void);
void MenuApp_SelectTrackImageView(void);
void MenuApp_SelectFullGrayView(void);
void MenuApp_SelectEdgeGrayView(void);
void MenuApp_SelectEdgeBoundaryView(void);
MenuCore *MenuApp_GetCore(void);

#endif /* CODE_MENU_APP_H_ */
