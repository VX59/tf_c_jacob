#include "../../config/config.h"
#include "../../source_sdk/debug_overlay/debug_overlay.h"
#include "../../source_sdk/engine_client/engine_client.h"
#include "../../source_sdk/engine_trace/engine_trace.h"
#include "../../source_sdk/estimate_abs_velocity/estimate_abs_velocity.h"
#include "../../source_sdk/entity/entity.h"
#include "../../source_sdk/entity/weapon_entity.h"
#include "../../source_sdk/entity_list/entity_list.h"
#include "../../source_sdk/global_vars/global_vars.h"
#include "../../source_sdk/math/vec3.h"
#include "../../source_sdk/user_cmd.h"
#include "../../utils/math/math_utils.h"
#include "../../utils/utils.h"
#include "../paint_traverse/paint_traverse.h"
#include "aimbot_util/aimbot_util.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

int project_speed_per_second(int weapon_id)
{
    switch (weapon_id)
    {
        case TF_WEAPON_ROCKETLAUNCHER:
            return 1100;
        case TF_WEAPON_ROCKETLAUNCHER_DIRECTHIT:
            return 1980;
        case TF_WEAPON_GRENADELAUNCHER:
            return 1216;
        default:
            return 0;
    }
}

void *get_closet_fov_ent_proj(void *localplayer, struct vec3_t *shoot_pos, struct vec3_t view_angle, int weapon_id, struct vec3_t *result_angle, struct vec3_t *result_pos, float *result_time)
{
    void *target_ent = NULL;
    float smallest_fov_angle = __FLT_MAX__;
    int max_clients = get_max_clients();

    for (int ent_index = 1; ent_index <= get_max_entities(); ent_index++)
    {
        void *entity = get_client_entity(ent_index);

        if (entity == NULL || 
            entity == localplayer || 
            is_ent_dormant(entity) || 
            get_ent_team(entity) == get_ent_team(localplayer))
        {
            continue;
        }
        float predicted_time = 0.0f;
        struct vec3_t aim_pos = { 0.0f, 0.0f, 0.0f };
        if (ent_index <= max_clients && get_ent_lifestate(entity) == 1)
        {

            struct vec3_t ent_pos = get_ent_origin(entity);
            struct vec3_t loc_pos = get_ent_origin(localplayer);
            struct vec3_t ent_velocity;
            bool ent_in_air = (get_ent_flags(entity) & 1) == 0;

            estimate_abs_velocity(entity, &ent_velocity);

            double gp = 400;
            double ga = 0;

            if (ent_in_air)
            {
                ga = 800;
            }

            double x_ento = hypotf(ent_pos.x - loc_pos.x, ent_pos.y - loc_pos.y);
            double x_o = 0, y_o = get_ent_eye_pos(localplayer).z;
            double v_o = project_speed_per_second(weapon_id);
            double v_entx = hypotf(ent_velocity.x, ent_velocity.y);
            double v_enty = ent_velocity.z;
            double y_ento = ent_pos.z - loc_pos.z;
            
            // most likely a and be dont bracket the root
            double a = 0, b = 0;
            double Pa = P(a, gp, ga, x_ento, x_o, v_o, v_entx, v_enty, y_ento, y_o);
            //log_msg("Initial P(a) = %f\n", Pa);

            bool valid_interval_found = false;

            for (double s = a; s < M_PI; s += 0.00001) {
                
                double Ps = P(s, gp, ga, x_ento, x_o, v_o, v_entx, v_enty, y_ento, y_o);
                if (Pa > 0 && Ps < 0) {
                    b = s;
                    valid_interval_found = true;
                    break;
                }
                if (Pa < 0 && Ps > 0) {
                    b = s;
                    valid_interval_found = true;
                    break;
                }
            }
            double Pb = P(b, gp, ga, x_ento, x_o, v_o, v_entx, v_enty, y_ento, y_o);
            for (double s = b; s > -M_PI; s -= 0.00001) {
                
                double Ps = P(s, gp, ga, x_ento, x_o, v_o, v_entx, v_enty, y_ento, y_o);
                if (Pb > 0 && Ps < 0) {
                    a = s;
                    valid_interval_found = true;
                    break;
                }
                if (Pb < 0 && Ps > 0) {
                    a = s;
                    valid_interval_found = true;
                    break;
                }
            }
            if (!valid_interval_found) {
                //log_msg("No valid root interval found.\n");
                predicted_time = 0;
                aim_pos = ent_pos;
            }
            else
            {
                //log_msg("Found interval: a = %f, b = %f\n", a, b);

                // Now run Brent's method with tighter settings
                double tol = 0.00001;
                int max_iter = 1000;
                double root = Brent(P, a, b, tol, max_iter, gp, ga, x_ento, x_o, v_o, v_entx, v_enty, y_ento, y_o);

                if (!isnan(root))
                {
                    log_msg("The root is %f\n", root * 180.0f/M_PI);


                    estimate_abs_velocity(entity, &ent_velocity);
                    double T = (x_ento-x_o)/(v_o*cos(root)-v_entx);

                    if (ent_in_air)
                    {

                        ent_pos.x += ent_velocity.x * T;
                        ent_pos.y += ent_velocity.y * T;
                        ent_pos.z += ent_velocity.z * T - (0.5f * 800 * (T * T));

                    }
                    else
                    {

                        ent_pos.x += ent_velocity.x * T;
                        ent_pos.y += ent_velocity.y * T;
                        ent_pos.z += ent_velocity.z * T;

                    }
                    log_msg("Time of flight is %f", T);
                    predicted_time = T;

                    aim_pos = ent_pos;
                }
                else{
                    predicted_time = 0;
                    aim_pos = ent_pos;
                }
            }

            
            // else
            // {
            //     for (float t = 0.0f; t < config.aimbot.projectile_max_time; t += config.aimbot.projectile_time_step)
            //     {
            //         struct vec3_t ent_pos = get_ent_origin(entity);
            //         struct vec3_t ent_velocity;
            //         bool ent_in_air = (get_ent_flags(entity) & 1) == 0;

            //         estimate_abs_velocity(entity, &ent_velocity);

            //         if (ent_in_air)
            //         {
            //             ent_pos.x += ent_velocity.x * t;
            //             ent_pos.y += ent_velocity.y * t;
            //             ent_pos.z += ent_velocity.z * t - (0.5f * 800 * (t * t));
            //         }
            //         else
            //         {
            //             ent_pos.x += ent_velocity.x * t;
            //             ent_pos.y += ent_velocity.y * t;
            //             ent_pos.z += ent_velocity.z * t;
            //         }

            //         float rocket_speed = project_speed_per_second(weapon_id);

            //         float distance = get_distance(ent_pos, *shoot_pos);
            //         float time = distance / rocket_speed;
            //         float time_rounded = roundf(time / config.aimbot.projectile_time_step) * config.aimbot.projectile_time_step;

            //         if (time_rounded > t + config.aimbot.projectile_tolerance_time || 
            //             time_rounded < t - config.aimbot.projectile_tolerance_time || 
            //             !is_pos_visible(localplayer, shoot_pos, ent_pos))
            //         {
            //             continue;
            //         }
            //         predicted_time = t;

            //         aim_pos = ent_pos;
            //     }
            // }
        }
        else
        {
            if (get_class_id(entity) != SENTRY && get_class_id(entity) != DISPENSER && get_class_id(entity) != TELEPORTER)
            {
                continue;
            }

            if (!is_ent_visible(localplayer, shoot_pos, entity))
            {
                continue;
            }

            aim_pos = get_ent_origin(entity);
            aim_pos.z += 5.0f;
        }

        if (aim_pos.x == 0.0f && aim_pos.y == 0.0f && aim_pos.z == 0.0f)
        {
            continue;
        }

        struct vec3_t new_view_angle;
        float fov_distance;

        new_view_angle = get_view_angle(get_difference(aim_pos, *shoot_pos));
        fov_distance = get_fov(view_angle, new_view_angle);

        if (fov_distance <= config.aimbot.fov && fov_distance < smallest_fov_angle)
        {
            target_ent = entity;
            smallest_fov_angle = fov_distance;
            *result_angle = new_view_angle;
            *result_pos = aim_pos;
            *result_time = predicted_time;
        }
    }

    return target_ent;
}

void get_projectile_fire_setup(void *localplayer, struct vec3_t view_angles, struct vec3_t offset, struct vec3_t *result)
{
    struct vec3_t forward, right, up;
    struct vec3_t shoot_pos = get_shoot_pos(localplayer);
    
    angle_vectors(view_angles, &forward, &right, &up);

    result->x = shoot_pos.x + (forward.x * offset.x) + (right.x * offset.y) + (up.x * offset.z);
    result->y = shoot_pos.y + (forward.y * offset.x) + (right.y * offset.y) + (up.y * offset.z);
    result->z = shoot_pos.z + (forward.z * offset.x) + (right.z * offset.y) + (up.z * offset.z);
}

void projectile_aimbot(void *localplayer, struct user_cmd *user_cmd, int weapon_id)
{
    struct vec3_t original_view_angle = user_cmd->viewangles;
    float original_side_move = user_cmd->sidemove;
    float original_forward_move = user_cmd->forwardmove;

    float result_time = -1.0f;
    struct vec3_t result_pos;
    struct vec3_t target_view_angle;
    struct vec3_t shoot_pos;
    struct vec3_t local_pos = get_ent_eye_pos(localplayer);
    
    get_projectile_fire_setup(localplayer, user_cmd->viewangles, (struct vec3_t){ 23.5f, 12.0f, -3.0f }, &shoot_pos);
    void *target_ent = get_closet_fov_ent_proj(localplayer, &local_pos, user_cmd->viewangles, weapon_id, &target_view_angle, &result_pos, &result_time);

    // TBD: This is a really bad method of drawing this
    struct vec3_t mins = { -3, -3, -3 };
    struct vec3_t maxs = { 3, 3, 3 };
    struct vec3_t orientation = { 0, 0, 0 };
    static const float time_per_box = 0.03f;

    if (target_ent == NULL)
    {
        if (config.aimbot.projectile_preview.only_draw_if_target || (!config.aimbot.projectile_preview.draw_box && !config.aimbot.projectile_preview.draw_line))
        {
            return;
        }

        // TBD: Draw predicted pos no aimbot for all guns
        float pitch_rad = user_cmd->viewangles.x * (M_PI / 180.0f);
        float yaw_rad = user_cmd->viewangles.y * (M_PI / 180.0f);

        // Compute the direction vector
        struct vec3_t direction = {
            cos(pitch_rad) * cos(yaw_rad),
            cos(pitch_rad) * sin(yaw_rad),
            sin(pitch_rad)
        };

        // Calculate the end position
        struct vec3_t end_pos = {
            local_pos.x + (direction.x * 1000),
            local_pos.y + (direction.y * 1000),
            local_pos.z - (direction.z * 1000),
        };


        struct ray_t ray = init_ray(&shoot_pos, &end_pos);
        struct trace_filter filter;
        init_trace_filter(&filter, localplayer);
        struct trace_t trace;
        trace_ray(&ray, 0x4600400b, &filter, &trace);

        if (config.aimbot.projectile_preview.draw_box)
        {
            float box_alpha = config.aimbot.projectile_preview.box_color.a * 255;
            struct vec3_t box_color = {
                config.aimbot.projectile_preview.box_color.r * 255,
                config.aimbot.projectile_preview.box_color.g * 255,
                config.aimbot.projectile_preview.box_color.b * 255
            };
            add_box_overlay(&trace.endpos, &mins, &maxs, &orientation, box_color.x, box_color.y, box_color.z, box_alpha, time_per_box);
        }

        if (config.aimbot.projectile_preview.draw_line)
        {
            struct vec3_t line_color = {
                config.aimbot.projectile_preview.line_color.r * 255,
                config.aimbot.projectile_preview.line_color.g * 255,
                config.aimbot.projectile_preview.line_color.b * 255
            };
            // TBD: This should use alpha
            add_line_overlay(&shoot_pos, &trace.endpos, line_color.x, line_color.y, line_color.z, true, time_per_box);
        }

        return;
    }

    add_bbox_decorator(L"TARGET", (struct vec3_t){207, 115, 54}, target_ent);
    if (config.aimbot.projectile_preview.draw_entity_prediction)
    {
        for (float t = 0.0f; t < config.aimbot.projectile_max_time; t += config.aimbot.projectile_time_step)
        {
            struct vec3_t ent_pos = get_ent_origin(target_ent);
            struct vec3_t ent_velocity;
            bool ent_in_air = (get_ent_flags(target_ent) & 1) == 0;

            estimate_abs_velocity(target_ent, &ent_velocity);

            if (ent_in_air)
            {
                ent_pos.x += ent_velocity.x * t;
                ent_pos.y += ent_velocity.y * t;
                ent_pos.z += ent_velocity.z * t - (0.5f * 800 * (t * t));
            }
            else
            {
                ent_pos.x += ent_velocity.x * t;
                ent_pos.y += ent_velocity.y * t;
                ent_pos.z += ent_velocity.z * t;
            }

            add_projectile_target_line_point(ent_pos);
        }
    }

    if (config.aimbot.projectile_preview.draw_box)
    {
        float box_alpha = config.aimbot.projectile_preview.box_color.a * 255;
        struct vec3_t box_color = {
            config.aimbot.projectile_preview.box_color.r * 255,
            config.aimbot.projectile_preview.box_color.g * 255,
            config.aimbot.projectile_preview.box_color.b * 255
        };
        add_box_overlay(&result_pos, &mins, &maxs, &orientation, box_color.x, box_color.y, box_color.z, box_alpha, time_per_box);
    }

    if (config.aimbot.projectile_preview.draw_line)
    {
        struct vec3_t line_color = {
            config.aimbot.projectile_preview.line_color.r * 255,
            config.aimbot.projectile_preview.line_color.g * 255,
            config.aimbot.projectile_preview.line_color.b * 255
        };
        // TBD: This should use alpha
        add_line_overlay(&shoot_pos, &result_pos, line_color.x, line_color.y, line_color.z, true, time_per_box);
    }

    if (config.aimbot.key.use_key && config.aimbot.key.is_pressed)
    {
        user_cmd->buttons |= 1;
    }

    if (((config.aimbot.key.use_key && config.aimbot.key.is_pressed) ||
        (!config.aimbot.key.use_key && (user_cmd->buttons & 1) != 0)) &&
        can_attack(localplayer))
    {
        user_cmd->viewangles = target_view_angle;

        if (config.aimbot.projectile_preview.draw_timer && result_time != -1.0f)
        {
            struct vec3_t timer_color = {
                config.aimbot.projectile_preview.timer_color.r * 255,
                config.aimbot.projectile_preview.timer_color.g * 255,
                config.aimbot.projectile_preview.timer_color.b * 255
            };
            add_timer((struct vec3_t){ result_pos.x, result_pos.y, result_pos.z + 10}, timer_color, get_global_vars_curtime(), get_global_vars_curtime() + result_time);
        }

        if (config.aimbot.projectile_preview.previous_shot_line && result_time != -1.0f)
        {
            struct vec3_t previous_shot_line_color = {
                config.aimbot.projectile_preview.previous_shot_line_color.r * 255,
                config.aimbot.projectile_preview.previous_shot_line_color.g * 255,
                config.aimbot.projectile_preview.previous_shot_line_color.b * 255
            };
            add_line_overlay(&shoot_pos, &result_pos, previous_shot_line_color.x, previous_shot_line_color.y, previous_shot_line_color.z, true, result_time + config.aimbot.projectile_preview.previous_shot_linger_time);
        }

        if (config.aimbot.projectile_preview.previous_shot_box && result_time != -1.0f)
        {
            float box_alpha = config.aimbot.projectile_preview.previous_shot_box_color.a * 255;
            struct vec3_t box_color = {
                config.aimbot.projectile_preview.previous_shot_box_color.r * 255,
                config.aimbot.projectile_preview.previous_shot_box_color.g * 255,
                config.aimbot.projectile_preview.previous_shot_box_color.b * 255
            };
            add_box_overlay(&result_pos, &mins, &maxs, &orientation, box_color.x, box_color.y, box_color.z, box_alpha, result_time + config.aimbot.projectile_preview.previous_shot_linger_time);
        }
    }

    movement_fix(user_cmd, original_view_angle, original_forward_move, original_side_move);
}