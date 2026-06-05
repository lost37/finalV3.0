#ifndef CODE_MENU_CORE_H_
#define CODE_MENU_CORE_H_

#include <stddef.h>
#include <stdint.h>

#define MENU_CORE_MAX_NODES 64

typedef enum
{
    MENU_CORE_NODE_FOLDER = 0,
    MENU_CORE_NODE_FLOAT,
    MENU_CORE_NODE_INT,
    MENU_CORE_NODE_INT32,
} MenuCoreNodeKind;

typedef enum
{
    MENU_CORE_ACTION_UP = 0,
    MENU_CORE_ACTION_DOWN,
    MENU_CORE_ACTION_ENTER,
    MENU_CORE_ACTION_BACK,
} MenuCoreAction;

typedef struct MenuCoreNode
{
    const char *name;
    MenuCoreNodeKind kind;
    void *value;
    float step;
    float min_value;
    float max_value;
    uint8_t selected;

    struct MenuCoreNode *parent;
    struct MenuCoreNode *first_child;
    struct MenuCoreNode *next;
    struct MenuCoreNode *prev;
} MenuCoreNode;

typedef struct
{
    MenuCoreNode nodes[MENU_CORE_MAX_NODES];
    size_t node_count;
    MenuCoreNode root;
    MenuCoreNode *current;
} MenuCore;

void MenuCore_Init(MenuCore *menu, const char *root_name);
MenuCoreNode *MenuCore_Root(MenuCore *menu);
MenuCoreNode *MenuCore_Current(MenuCore *menu);
MenuCoreNode *MenuCore_AddFolder(MenuCore *menu, MenuCoreNode *parent, const char *name);
MenuCoreNode *MenuCore_AddFloat(MenuCore *menu, MenuCoreNode *parent, const char *name, float *value, float step, float min_value, float max_value);
MenuCoreNode *MenuCore_AddInt(MenuCore *menu, MenuCoreNode *parent, const char *name, int *value, float step, float min_value, float max_value);
MenuCoreNode *MenuCore_AddInt32(MenuCore *menu, MenuCoreNode *parent, const char *name, int32_t *value, float step, float min_value, float max_value);
void MenuCore_HandleAction(MenuCore *menu, MenuCoreAction action);

#endif /* CODE_MENU_CORE_H_ */
