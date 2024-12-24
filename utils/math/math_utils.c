#include "../../source_sdk/math/vec3.h"

#include <math.h>

struct vec3_t get_difference(struct vec3_t pos1, struct vec3_t pos2)
{
    struct vec3_t difference;
    difference.x = pos1.x - pos2.x;
    difference.y = pos1.y - pos2.y;
    difference.z = pos1.z - pos2.z;
    return difference;
}

float get_distance(struct vec3_t pos1, struct vec3_t pos2)
{
  return sqrt(((pos1.x - pos2.x)*(pos1.x - pos2.x)) + ((pos1.y - pos2.y)*(pos1.y - pos2.y)) + ((pos1.z - pos2.z)*(pos1.z - pos2.z)));
}

float positive_quadratic_root(float a, float b, float c)
{
    float discriminant = (b * b) - (4 * a * c);
    if (discriminant < 0)
    {
        return -1;
    }

    float root1 = (-b + sqrt(discriminant)) / (2 * a);
    float root2 = (-b - sqrt(discriminant)) / (2 * a);

    if (root2 > 0 && root1 > 0)
    {
        if (root1 < root2)
        {
            return root1;
        }
        else
        {
            return root2;
        }
    }

    if (root1 > 0)
    {
        return root1;
    }

    if (root2 > 0)
    {
        return root2;
    }

    return -1;
}

float angle_between_vectors(struct vec3_t vec1, struct vec3_t vec2)
{
    float dot_product = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    float magnitude1 = sqrt(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
    float magnitude2 = sqrt(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);
    return acos(dot_product / (magnitude1 * magnitude2)) * 180 / M_PI;
}

void angle_vectors(struct vec3_t angles, struct vec3_t *forward, struct vec3_t *right, struct vec3_t *up)
{
    // Sin/cosine for rotation, pitch, yaw
    float sp = sin(angles.x * (M_PI / 180));
    float cp = cos(angles.x * (M_PI / 180));
    float sy = sin(angles.y * (M_PI / 180));
    float cy = cos(angles.y * (M_PI / 180));
    float sr = sin(angles.z * (M_PI / 180));
    float cr = cos(angles.z * (M_PI / 180));

    if (forward)
    {
        forward->x = cp * cy;
        forward->y = cp * sy;
        forward->z = -sp;
    }

    if (right)
    {
        right->x = (-1 * sr * sp * cy + -1 * cr * -sy);
        right->y = (-1 * sr * sp * sy + -1 * cr * cy);
        right->z = -1 * sr * cp;
    }

    if (up)
    {
        up->x = (cr * sp * cy + -sr * -sy);
        up->y = (cr * sp * sy + -sr * cy);
        up->z = cr * cp;
    }
}

struct vec3_t get_projectile_lob_angle(struct vec3_t diff, float velocity)
{
    // given a shooting position aiming position and velocity we calculate the parabolic arc that connects the 2 points in space
    float g = 400;
    float A = (g*(diff.x*diff.x))/(2*velocity);
    float B = sqrtf((diff.x * diff.x) + (diff.y * diff.y));
    float C = diff.z;

    // the positive root
    float u = (-B + sqrtf(B*B - (4*A*C)))/(2*A);

    float pitch_angle = atan(u) * 180/M_PI;
    float yaw_angle = atan2(diff.y, diff.x) * 180 / M_PI;

    struct vec3_t view_angle = {
        .x = -pitch_angle,
        .y = yaw_angle,
        .z = 0
    };

    return view_angle;
}

struct vec3_t get_view_angle(struct vec3_t diff)
{

    // Common side between two right triangles
    float c = sqrt((diff.x * diff.x) + (diff.y * diff.y));

    float pitch_angle = atan2(diff.z, c) * 180 / M_PI;
    float yaw_angle = atan2(diff.y, diff.x) * 180 / M_PI;

    struct vec3_t view_angle = {
        .x = -pitch_angle,
        .y = yaw_angle,
        .z = 0
    };

    return view_angle;
}

double P(double theta, double g, double a, double x_ento, double x_o, double v_o, 
        double v_entx, double v_enty, double y_ento, double y_o)
{
    double dvx = v_o * cos(theta) - v_entx;
    double dx = x_ento - x_o;

    double dvy = v_enty - v_o * sin(theta);
    double dy = y_ento - y_o;

    if (fabs(dvx) < __DBL_EPSILON__)
    {
        return INFINITY;
    }

    double A = -0.5f*(g+a) * pow(dx/dvx,2);
    double B = dvy*dx/dvx;
    double C = dy;

    return A + B + C;
}

double Brent(double (*P)(double, double, double, double, double, double, double, double, double, double)
,double a, double b, double tol, int max_iter, double gp, double ga, double x_ento, double x_o, double v_o, double v_entx, double v_enty, double y_ento, double y_o)
{
    double fa = P(a,gp,ga,x_ento,x_o,v_o,v_entx,v_enty,y_ento, y_o);
    double fb = P(b,gp,ga,x_ento,x_o,v_o,v_entx,v_enty,y_ento, y_o);
    if (fa * fb >= 0)
    {
        log_msg("No root bracketing: P(a) = %f, P(b) = %f\n", fa, fb);
        return NAN;
    }

    double c = a, fc = fa;
    double d = 0, e = 0;
    double m, total_act;
    for (int i = 0; i < max_iter; i++)
    {
        if (fabs(fc) < fabs(fb))
        {
            a = b; b = c; c = a;
            fa = fb; fb = fc; fc = fa;
        }

        total_act = 2 * __DBL_EPSILON__ * fabs(b) + tol/2.0f;
        m = 0.5 * (c - b);

        if (fabs(m) <= total_act || fb == 0)
        {
            return b;   // converged
        }

        if (fabs(e) >= total_act && fabs(fa) > fabs(fb))
        {
            double s = fb / fa;
            double p , q;

            if (a == c)
            {
                // secant method
                p = 2 * m * s;
                q = 1 - s;
            }
            else
            {
                // inverse quadratic interpolation
                q = fa / fc;
                double r = fb / fc;

                p = s * (2 * m * q * (q - r) - (b - a) * (r - 1));
                q = (q - 1) * (r - 1) * (s - 1);
            }

            if (p > 0) q = -q;
            p = fabs(p);

            if (2 * p < fmin(3 * m * q - fabs(total_act * q), fabs(e * q)))
            {
                e = d; d = p / q;
            }
            else
            {
                d = m; e = m;
            }
        }
        else
        {
            d = m; e = m;
        }

        fb = P(b, gp, ga, x_ento, x_o, v_o, v_entx, v_enty, y_ento, y_o);
        if ((fb > 0 && fc > 0) || (fb < 0 && fc < 0))
        {
            c = a; fc = fa; e = d = b - a;
        }
    }
    log_msg("failed to converge m %f total act %f\n", fabs(m), total_act);
    return NAN;
}