#include "../../config/config.h"
#include "../../utils/utils.h"
#include "menu.h"

#include <stdbool.h>

#define HEADER_SIZE 40
#define ROW_SIZE 20

#define NK_TEXT_ROW(ctx, text, flags)               \
    do {                                            \
        nk_layout_row_dynamic((ctx), ROW_SIZE, 1);  \
        nk_label((ctx), (text), (flags));           \
    } while(0)                                      \

#define NK_HEADER_ROW(ctx, text, flags)                 \
    do {                                                \
        nk_layout_row_dynamic((ctx), HEADER_SIZE, 1);   \
        nk_label((ctx), (text), (flags));               \
    } while(0)                                          \

#define NK_CHECKBOX_ROW(ctx, text, active_ptr)          \
    do {                                                \
        nk_layout_row_dynamic((ctx), ROW_SIZE, 1);      \
        nk_checkbox_label((ctx), (text), (active_ptr)); \
    } while(0)                                          \

#define NK_FLOAT_SLIDER_ROW(ctx, text, value_ptr, min, max, step) \
    do {                                                          \
        nk_layout_row_dynamic((ctx), ROW_SIZE, 2);                \
        nk_label((ctx), (text), NK_TEXT_LEFT);                    \
        nk_slider_float((ctx), (min), (value_ptr), (max), (step));\
    } while(0)                                                    \

#define NK_COLOR_PICKER_ROW(ctx, text, color_ptr)                                                       \
    do {                                                                                                \
        nk_layout_row_dynamic((ctx), ROW_SIZE, 2);                                                      \
        nk_label((ctx), (text), NK_TEXT_LEFT);                                                          \
        if (nk_combo_begin_color((ctx), nk_rgb_cf(*(color_ptr)), nk_vec2(nk_widget_width(ctx), 400))) { \
            nk_layout_row_dynamic((ctx), 120, 1);                                                       \
            *(color_ptr) = nk_color_picker((ctx), *(color_ptr), NK_RGBA);                               \
            nk_layout_row_dynamic((ctx), 25, 1);                                                        \
            nk_combo_end(ctx);                                                                          \
        }                                                                                               \
    } while(0)                                                                                          \

#define NK_COMBO_BOX_ROW(ctx, id_token, label, text_options, options_count, ...)                                            \
    do {                                                                                                                    \
        static const char **id_token##_options = text_options;                                                              \
        static int *id_token##_selections[] = { __VA_ARGS__ };                                                              \
        static char id_token##_preview_text[128] = "";                                                                      \
        nk_layout_row_dynamic((ctx), ROW_SIZE, 2);                                                                          \
        nk_label((ctx), (label), NK_TEXT_LEFT);                                                                             \
        multi_select_combo_box((ctx), id_token##_options, id_token##_selections, options_count, id_token##_preview_text);   \
    } while(0)                                                                                                              \

void watermark(struct nk_context *ctx)
{
    if (nk_begin(ctx, "watermark", nk_rect(10, 10, 150, 30), NK_WINDOW_BORDER | NK_WINDOW_NO_INPUT | NK_WINDOW_NO_SCROLLBAR))
    {
        NK_TEXT_ROW(ctx, "TF_C by faluthe", NK_TEXT_CENTERED);
    }
    nk_end(ctx);
}

void multi_select_combo_box(struct nk_context *ctx, const char **options, int **selections, int options_count, char *preview_text)
{
    const int preview_text_size = 128;

    preview_text[0] = '\0';
    for (int k = 0; k < options_count; k++)
    {
        if (*selections[k])
        {
            if (strlen(preview_text) > 0)
            {
                strncat(preview_text, ", ", preview_text_size - strlen(preview_text) - 1);
            }
            strncat(preview_text, options[k], preview_text_size - strlen(preview_text) - 1);
        }
    }
    if (strlen(preview_text) == 0)
    {
        strncpy(preview_text, "None", preview_text_size - 1);
        preview_text[preview_text_size - 1] = '\0';
    }

    if (nk_combo_begin_label(ctx, preview_text, nk_vec2(nk_widget_width(ctx), 300)))
    {
        nk_layout_row_dynamic(ctx, 20, 1);
        for (int i = 0; i < options_count; i++)
        {
            if (nk_checkbox_label(ctx, options[i], selections[i]))
            {
                preview_text[0] = '\0';
                for (int k = 0; k < options_count; k++)
                {
                    if (*selections[k])
                    {
                        if (strlen(preview_text) > 0)
                        {
                            strncat(preview_text, ", ", preview_text_size - strlen(preview_text) - 1);
                        }
                        strncat(preview_text, options[k], preview_text_size - strlen(preview_text) - 1);
                    }
                }
                if (strlen(preview_text) == 0)
                {
                    strncpy(preview_text, "None", preview_text_size - 1);
                    preview_text[preview_text_size - 1] = '\0';
                }
            }
        }
        nk_combo_end(ctx);
    }
}

void draw_ent_targeting_manager_tab(struct nk_context *ctx)
{
    
}

void draw_aim_tab(struct nk_context *ctx)
{
    NK_HEADER_ROW(ctx, "General", NK_TEXT_LEFT);
    {
        char fov_text[32];
        // \xC2\xB0 is the UTF-8 encoding for the degree symbol
        sprintf(fov_text, "Aimbot FOV: %.0f\xC2\xB0", config.aimbot.fov);
        
        NK_CHECKBOX_ROW(ctx, "Aimbot enabled", &config.aimbot.aimbot_enabled);
        NK_CHECKBOX_ROW(ctx, "Use aim key", &config.aimbot.key.use_key);
        NK_FLOAT_SLIDER_ROW(ctx, fov_text, &config.aimbot.fov, 1.0f, 50.0f, 1.0f);
        NK_CHECKBOX_ROW(ctx, "Draw FOV", &config.aimbot.draw_fov);
        if (config.aimbot.draw_fov)
        {
            NK_COLOR_PICKER_ROW(ctx, "FOV color:", &config.aimbot.fov_color);
        }
    }

    NK_HEADER_ROW(ctx, "Projectile", NK_TEXT_LEFT);
    {
        static const char *proj_preview_options[] = { "Trace line", "3D Box", "Only draw on target" };
        static const char *previous_shot_options[] = { "Trace line", "3D Box", "Timer" };

        NK_COMBO_BOX_ROW(ctx, projectile_preview, "Projectile preview:", proj_preview_options, 3, &config.aimbot.projectile_preview.draw_line, &config.aimbot.projectile_preview.draw_box, &config.aimbot.projectile_preview.only_draw_if_target);
        if (config.aimbot.projectile_preview.draw_line)
        {
            NK_COLOR_PICKER_ROW(ctx, "Line color:", &config.aimbot.projectile_preview.line_color);
        }
        if (config.aimbot.projectile_preview.draw_box)
        {
            NK_COLOR_PICKER_ROW(ctx, "Box color:", &config.aimbot.projectile_preview.box_color);
        }
        NK_COMBO_BOX_ROW(ctx, previous_shot_preview, "Previous shot preview:", previous_shot_options, 3, &config.aimbot.projectile_preview.previous_shot_line, &config.aimbot.projectile_preview.previous_shot_box, &config.aimbot.projectile_preview.draw_timer);
        if (config.aimbot.projectile_preview.previous_shot_line)
        {
            NK_COLOR_PICKER_ROW(ctx, "Previous shot line color:", &config.aimbot.projectile_preview.previous_shot_line_color);
        }
        if (config.aimbot.projectile_preview.previous_shot_box)
        {
            NK_COLOR_PICKER_ROW(ctx, "Previous shot box color:", &config.aimbot.projectile_preview.previous_shot_box_color);
        }
        if (config.aimbot.projectile_preview.draw_timer)
        {
            NK_COLOR_PICKER_ROW(ctx, "Timer color:", &config.aimbot.projectile_preview.timer_color);
        }
        NK_CHECKBOX_ROW(ctx, "Draw entity prediction", &config.aimbot.projectile_preview.draw_entity_prediction);
        if (config.aimbot.projectile_preview.draw_entity_prediction)
        {
            NK_COLOR_PICKER_ROW(ctx, "Entity prediction color:", &config.aimbot.projectile_preview.entity_prediction_color);
        }
        if (config.aimbot.projectile_preview.previous_shot_box || config.aimbot.projectile_preview.previous_shot_line)
        {
            char previous_shot_linger_time_text[32];
            sprintf(previous_shot_linger_time_text, "Previous shot linger time: %.2f", config.aimbot.projectile_preview.previous_shot_linger_time);
            NK_FLOAT_SLIDER_ROW(ctx, previous_shot_linger_time_text, &config.aimbot.projectile_preview.previous_shot_linger_time, 0.0f, 2.0f, 0.05f);
        }
    }

    // TBD: Macro this
    nk_layout_row_dynamic(ctx, 20, 2);
    nk_label(ctx, "Aimbot key: ", NK_TEXT_LEFT);
    char key_edit_buffer[64];
    if (config.aimbot.key.binding.editing)
    {
        snprintf(key_edit_buffer, sizeof(key_edit_buffer), "Press a key/mouse button");
    }
    else
    {
        if (config.aimbot.key.binding.type == INPUT_KEY)
        {
            snprintf(key_edit_buffer, sizeof(key_edit_buffer), "Key: %d", config.aimbot.key.binding.code);
        }
        else if (config.aimbot.key.binding.type == INPUT_MOUSE)
        {
            snprintf(key_edit_buffer, sizeof(key_edit_buffer), "Mouse: %d", config.aimbot.key.binding.code);
        }
        else
        {
            snprintf(key_edit_buffer, sizeof(key_edit_buffer), "None");
        }
    }

    if (nk_button_label(ctx, key_edit_buffer))
    {
        config.aimbot.key.binding.editing = 1;
    }
}

void draw_esp_tab(struct nk_context *ctx)
{
    NK_HEADER_ROW(ctx, "Enemy Player ESP", NK_TEXT_LEFT);
    {
        NK_CHECKBOX_ROW(ctx, "Health bar", &config.esp.player_health_bar);
        NK_CHECKBOX_ROW(ctx, "Name", &config.esp.player_name);
        NK_CHECKBOX_ROW(ctx, "Bounding box", &config.esp.player_bounding_box);
        if (config.esp.player_bounding_box)
        {
            NK_CHECKBOX_ROW(ctx, "Team color box", &config.esp.team_color);
            if (!config.esp.team_color)
            {
                NK_COLOR_PICKER_ROW(ctx, "Box color:", &config.esp.esp_color);
            }
        }
    }

    NK_HEADER_ROW(ctx, "Entity ESP", NK_TEXT_LEFT);
    {
        static const char *entity_esp_options[] = { "Name", "Bounding box" };
        NK_COMBO_BOX_ROW(ctx, ammo_hp, "Ammo/HP:", entity_esp_options, 2, &config.esp.ammo_hp_ents_name, &config.esp.ammo_hp_ents_bounding_box);
        NK_COMBO_BOX_ROW(ctx, sentry, "Sentry:", entity_esp_options, 2, &config.esp.sentry_name, &config.esp.sentry_bounding_box);
        NK_COMBO_BOX_ROW(ctx, teleporter, "Teleporter:", entity_esp_options, 2, &config.esp.teleporter_name, &config.esp.teleporter_bounding_box);
        NK_COMBO_BOX_ROW(ctx, dispenser, "Dispenser:", entity_esp_options, 2, &config.esp.dispenser_name, &config.esp.dispenser_bounding_box);
        NK_COMBO_BOX_ROW(ctx, friendly_dispenser, "Friendly dispenser:", entity_esp_options, 2, &config.esp.friendly_dispenser_name, &config.esp.friendly_dispenser_bounding_box);
    }
}

void draw_misc_tab(struct nk_context *ctx)
{
    NK_HEADER_ROW(ctx, "General", NK_TEXT_LEFT);
    {
        NK_CHECKBOX_ROW(ctx, "Bunny hop", &config.misc.bunny_hop);
        // TBD: Add thirdperson key stuff + bool
        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_button_label(ctx, "Save config"))
        {
            if (!save_config())
            {
                log_msg("Failed to save config\n");
            }
            else
            {
                log_msg("Config saved\n");
            }
        }

        nk_layout_row_dynamic(ctx, 20, 1);
        if (nk_button_label(ctx, "Load config"))
        {
            log_msg("Loading config\n");
            if (!init_config())
            {
                log_msg("Failed to save config\n");
            }
            else
            {
                log_msg("Config loaded\n");
            }
        }
    }
}

void draw_tab(struct nk_context *ctx, const char *name, int *tab, int index)
{
    if (*tab == index)
    {
        ctx->style.button.normal.data.color = nk_rgb(35, 35, 35);
    }
    else
    {
        ctx->style.button.normal.data.color = nk_rgb(50, 50, 50);
    }
    if (nk_button_label(ctx, name))
    {
        *tab = index;
    }
    ctx->style.button.normal.data.color = nk_rgb(50, 50, 50);
}

void draw_menu(struct nk_context *ctx)
{
    if (nk_begin(ctx, "TF_C", nk_rect(200, 200, 500, 600), NK_WINDOW_BORDER | NK_WINDOW_MOVABLE | NK_WINDOW_TITLE))
        {
            static int tab = 0;

            nk_layout_row_dynamic(ctx, 30, 3);
            draw_tab(ctx, "Aim", &tab, 0);
            draw_tab(ctx, "ESP", &tab, 1);
            draw_tab(ctx, "Misc", &tab, 2);

            switch (tab)
            {
                case 0:
                    draw_aim_tab(ctx);
                    break;
                case 1:
                    draw_esp_tab(ctx);
                    break;
                case 2:
                    draw_misc_tab(ctx);
                    break;
            }
        }
    nk_end(ctx);
}