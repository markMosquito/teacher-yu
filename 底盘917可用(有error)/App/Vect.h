#ifndef __VECT_H_
#define __VECT_H_

#include "stm32f4xx.h"
/*点坐标*/
typedef struct 
{
    int32_t x;
    int32_t y;
    float   ang;
}
point_t;

typedef struct 
{
    float x;
    float y;
    float ang;
}
point_f;

typedef struct 
{
    float x;
    float y;
    float ang;
    float angcosf;
    float angsinf;
}
point_cam_off_f;

/*向量*/
typedef struct
{
    point_t s;
    point_t e;
}
vect_t;

typedef struct
{
    point_f s;
    point_f e;
}
vect_f;

/**
 * @Description : 向量相关的函数声明
 */
void SetPoint(point_t * p_point, int32_t x, int32_t y, float ang);
void set_vect(vect_t *p_vect, point_t *p_start, point_t *p_end);
void get_foot_point(vect_t *p_vect,point_t *p0, point_t * foot_point);
int32_t p2v_vertical_dist(vect_t *p_vect, point_t *p0);
int32_t p2v_parallel_dist(vect_t *p_vect, point_t *p0);
uint32_t get_vect_len(vect_t *p_vect);
uint32_t get_point_dis(point_t * p1, point_t * p2);
uint8_t is_vect_same_direction(vect_t * v1, vect_t * v2);
float get_v2x_ang(vect_t *p_vect);
void set_vectf(vect_f *p_vect, point_f *p_start, point_f *p_end);

#endif
