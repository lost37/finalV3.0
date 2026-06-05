#include "menu_core.h"

static float MenuCore_ClampFloat(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }
    if(value > max_value)
    {
        return max_value;
    }
    return value;
}

static MenuCoreNode *MenuCore_AllocNode(MenuCore *menu)
{
    if(menu == nullptr || menu->node_count >= MENU_CORE_MAX_NODES)
    {
        return nullptr;
    }

    MenuCoreNode *node = &menu->nodes[menu->node_count++];
    *node = MenuCoreNode{};
    return node;
}

static void MenuCore_LinkChild(MenuCoreNode *parent, MenuCoreNode *child)
{
    if(parent == nullptr || child == nullptr)
    {
        return;
    }

    child->parent = parent;
    if(parent->first_child == nullptr)
    {
        parent->first_child = child;
        child->next = child;
        child->prev = child;
        return;
    }

    MenuCoreNode *first = parent->first_child;
    MenuCoreNode *last = first->prev;
    child->next = first;
    child->prev = last;
    last->next = child;
    first->prev = child;
}

static MenuCoreNode *MenuCore_AddNode(MenuCore *menu, MenuCoreNode *parent, const char *name, MenuCoreNodeKind kind, void *value, float step, float min_value, float max_value)
{
    if(parent == nullptr)
    {
        return nullptr;
    }

    MenuCoreNode *node = MenuCore_AllocNode(menu);
    if(node == nullptr)
    {
        return nullptr;
    }

    node->name = name;
    node->kind = kind;
    node->value = value;
    node->step = step;
    node->min_value = min_value;
    node->max_value = max_value;
    node->selected = 0;
    MenuCore_LinkChild(parent, node);

    if(menu->current == nullptr && parent == &menu->root)
    {
        menu->current = node;
    }

    return node;
}

static void MenuCore_AdjustCurrent(MenuCore *menu, int direction)
{
    MenuCoreNode *node = MenuCore_Current(menu);
    if(node == nullptr || node->selected == 0)
    {
        return;
    }

    const float delta = node->step * (float)direction;
    if(node->kind == MENU_CORE_NODE_FLOAT)
    {
        float *value = static_cast<float *>(node->value);
        if(value != nullptr)
        {
            *value = MenuCore_ClampFloat(*value + delta, node->min_value, node->max_value);
        }
    }
    else if(node->kind == MENU_CORE_NODE_INT)
    {
        int *value = static_cast<int *>(node->value);
        if(value != nullptr)
        {
            const float clamped = MenuCore_ClampFloat((float)(*value) + delta, node->min_value, node->max_value);
            *value = (int)clamped;
        }
    }
    else if(node->kind == MENU_CORE_NODE_INT32)
    {
        int32_t *value = static_cast<int32_t *>(node->value);
        if(value != nullptr)
        {
            const float clamped = MenuCore_ClampFloat((float)(*value) + delta, node->min_value, node->max_value);
            *value = (int32_t)clamped;
        }
    }
}

void MenuCore_Init(MenuCore *menu, const char *root_name)
{
    if(menu == nullptr)
    {
        return;
    }

    *menu = MenuCore{};
    menu->root.name = root_name;
    menu->root.kind = MENU_CORE_NODE_FOLDER;
    menu->current = nullptr;
}

MenuCoreNode *MenuCore_Root(MenuCore *menu)
{
    if(menu == nullptr)
    {
        return nullptr;
    }
    return &menu->root;
}

MenuCoreNode *MenuCore_Current(MenuCore *menu)
{
    if(menu == nullptr)
    {
        return nullptr;
    }
    return menu->current;
}

MenuCoreNode *MenuCore_AddFolder(MenuCore *menu, MenuCoreNode *parent, const char *name)
{
    return MenuCore_AddNode(menu, parent, name, MENU_CORE_NODE_FOLDER, nullptr, 0.0f, 0.0f, 0.0f);
}

MenuCoreNode *MenuCore_AddFloat(MenuCore *menu, MenuCoreNode *parent, const char *name, float *value, float step, float min_value, float max_value)
{
    return MenuCore_AddNode(menu, parent, name, MENU_CORE_NODE_FLOAT, value, step, min_value, max_value);
}

MenuCoreNode *MenuCore_AddInt(MenuCore *menu, MenuCoreNode *parent, const char *name, int *value, float step, float min_value, float max_value)
{
    return MenuCore_AddNode(menu, parent, name, MENU_CORE_NODE_INT, value, step, min_value, max_value);
}

MenuCoreNode *MenuCore_AddInt32(MenuCore *menu, MenuCoreNode *parent, const char *name, int32_t *value, float step, float min_value, float max_value)
{
    return MenuCore_AddNode(menu, parent, name, MENU_CORE_NODE_INT32, value, step, min_value, max_value);
}

void MenuCore_HandleAction(MenuCore *menu, MenuCoreAction action)
{
    MenuCoreNode *node = MenuCore_Current(menu);
    if(menu == nullptr || node == nullptr)
    {
        return;
    }

    switch(action)
    {
        case MENU_CORE_ACTION_UP:
            if(node->selected)
            {
                MenuCore_AdjustCurrent(menu, 1);
            }
            else if(node->prev != nullptr)
            {
                menu->current = node->prev;
            }
            break;

        case MENU_CORE_ACTION_DOWN:
            if(node->selected)
            {
                MenuCore_AdjustCurrent(menu, -1);
            }
            else if(node->next != nullptr)
            {
                menu->current = node->next;
            }
            break;

        case MENU_CORE_ACTION_ENTER:
            if(node->kind == MENU_CORE_NODE_FOLDER)
            {
                if(node->first_child != nullptr)
                {
                    menu->current = node->first_child;
                }
            }
            else
            {
                node->selected = (uint8_t)!node->selected;
            }
            break;

        case MENU_CORE_ACTION_BACK:
            if(node->selected)
            {
                node->selected = 0;
            }
            else if(node->parent != nullptr && node->parent->parent != nullptr)
            {
                menu->current = node->parent;
            }
            break;

        default:
            break;
    }
}
