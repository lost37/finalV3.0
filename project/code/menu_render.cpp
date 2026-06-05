#include "menu_render.h"

#include <stdio.h>
#include <string.h>

static void MenuRender_ClearPage(MenuRenderPage *page)
{
    if(page == nullptr)
    {
        return;
    }

    memset(page, 0, sizeof(*page));
}

static MenuCoreNode *MenuRender_DisplayParent(MenuCoreNode *current)
{
    if(current == nullptr)
    {
        return nullptr;
    }

    if(current->kind == MENU_CORE_NODE_FOLDER && current->first_child != nullptr)
    {
        return current;
    }

    return current->parent;
}

static void MenuRender_WriteTitle(MenuRenderPage *page, MenuCoreNode *parent)
{
    if(page == nullptr || parent == nullptr)
    {
        return;
    }

    if(parent->parent == nullptr)
    {
        snprintf(page->lines[page->line_count], MENU_RENDER_LINE_LEN, "%s", parent->name);
    }
    else
    {
        snprintf(page->lines[page->line_count], MENU_RENDER_LINE_LEN, "%s/%s", parent->parent->name, parent->name);
    }
    page->line_count++;
}

static void MenuRender_FormatValue(MenuCoreNode *node, char *buffer, size_t buffer_size)
{
    if(node == nullptr || buffer == nullptr || buffer_size == 0)
    {
        return;
    }

    if(node->kind == MENU_CORE_NODE_FLOAT)
    {
        const float value = node->value != nullptr ? *static_cast<float *>(node->value) : 0.0f;
        if(node->selected)
        {
            snprintf(buffer, buffer_size, "<%.3f>", value);
        }
        else
        {
            snprintf(buffer, buffer_size, "%.3f", value);
        }
    }
    else if(node->kind == MENU_CORE_NODE_INT)
    {
        const int value = node->value != nullptr ? *static_cast<int *>(node->value) : 0;
        if(node->selected)
        {
            snprintf(buffer, buffer_size, "<%d>", value);
        }
        else
        {
            snprintf(buffer, buffer_size, "%d", value);
        }
    }
    else if(node->kind == MENU_CORE_NODE_INT32)
    {
        const int32_t value = node->value != nullptr ? *static_cast<int32_t *>(node->value) : 0;
        if(node->selected)
        {
            snprintf(buffer, buffer_size, "<%ld>", (long)value);
        }
        else
        {
            snprintf(buffer, buffer_size, "%ld", (long)value);
        }
    }
    else
    {
        snprintf(buffer, buffer_size, "[%s]", node->first_child != nullptr ? "dir" : "empty");
    }
}

uint8_t MenuRender_BuildPage(MenuCore *menu, MenuRenderPage *page)
{
    MenuRender_ClearPage(page);
    if(menu == nullptr || page == nullptr || MenuCore_Current(menu) == nullptr)
    {
        return 0;
    }

    MenuCoreNode *current = MenuCore_Current(menu);
    MenuCoreNode *parent = MenuRender_DisplayParent(current);
    if(parent == nullptr)
    {
        return 0;
    }

    MenuRender_WriteTitle(page, parent);

    MenuCoreNode *item = parent->first_child;
    if(item == nullptr)
    {
        return 1;
    }

    do
    {
        char value[14] = {0};
        MenuRender_FormatValue(item, value, sizeof(value));
        snprintf(
            page->lines[page->line_count],
            MENU_RENDER_LINE_LEN,
            "%s %-9s %s",
            item == current ? "->" : "  ",
            item->name != nullptr ? item->name : "",
            value
        );
        page->line_count++;
        item = item->next;
    } while(item != nullptr && item != parent->first_child && page->line_count < MENU_RENDER_MAX_LINES);

    return 1;
}
