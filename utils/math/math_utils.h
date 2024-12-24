#include "../../source_sdk/math/vec3.h"
#include <float.h>
float positive_quadratic_root(float a, float b, float c);
struct vec3_t get_difference(struct vec3_t pos1, struct vec3_t pos2);
float get_distance(struct vec3_t pos1, struct vec3_t pos2);
float angle_between_vectors(struct vec3_t vec1, struct vec3_t vec2);
void angle_vectors(struct vec3_t angles, struct vec3_t *forward, struct vec3_t *right, struct vec3_t *up);
struct vec3_t get_view_angle(struct vec3_t diff);
struct vec3_t get_projectile_lob_angle(struct vec3_t diff, float velocity);


double P(double theta, double g, double a, double x_ento, double x_o, double v_o, 
        double v_entx, double v_enty, double y_ento, double y_o);


double Brent(double (*P)(double, double, double, double, double, double, double, double, double, double)
,double a, double b, double tol, int max_iter, double gp, double ga, double x_ento, double x_o, double v_o, double v_entx, double v_enty, double y_ento, double y_o);