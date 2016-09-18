#include "Vect.h"
#include "MyMath.h"
#include <string.h>

/**
 * @function    : 
 * @Description : 根据两点设置一个向量
**/
void set_vect(vect_t *p_vect, point_t *p_start, point_t *p_end)
{
	memcpy(&p_vect->s, p_start, sizeof(point_t));
	memcpy(&p_vect->e, p_end,   sizeof(point_t));
}

void set_vectf(vect_f *p_vect, point_f *p_start, point_f *p_end)
{
	memcpy(&p_vect->s, p_start, sizeof(point_f));
	memcpy(&p_vect->e, p_end,   sizeof(point_f));
}

void SetPoint(point_t * p_point, int32_t x, int32_t y, float ang)
{
    p_point->x = x;
    p_point->y = y;
    p_point->ang = ang;
}
/**
 * @function    : 
 * @Description : 求得向量外一点到向量的垂线的垂足
 * @param       : 向量的指针，点坐标的指针，垂足的指针
**/
void get_foot_point(vect_t *p_vect,point_t *p0, point_t * foot_point)
{
	float foot_point_float;
	int64_t x0 = p0->x;
	int64_t y0 = p0->y;					/*点坐标*/
	int64_t x1 = p_vect->s.x;
	int64_t y1 = p_vect->s.y;		  	/*向量起点坐标*/
	int64_t x2 = p_vect->e.x;				  
	int64_t y2 = p_vect->e.y;			/*向量终点坐标*/
    
	if (x1 == x2)						/*若目标直线垂直于X轴*/
	{
		foot_point->x = x1;
		foot_point->y = y0;
	} 
	else if (y1 == y2)					/*若目标直线垂直于Y轴*/
	{
		foot_point->x = x0;;
		foot_point->y = y1;  				
	} 
	else								/*一般情况*/
	{
		foot_point_float=p0->x - (float)((y1 - y2)*(x0*y1 - x1*y0 - x0*y2 + x2*y0 + x1*y2 - x2*y1)) 
			                      /(square(x1) - 2*x1*x2 + square(x2) + square(y1) - 2*y1*y2 + square(y2));

		foot_point->x = foot_point_float;
		foot_point->y = (float)((x1-x2) * (foot_point_float-x0)) / (y2-y1) + y0; 
	}
}

/*
 *	p2v_vertical_dist:	求点到向量的垂直距离
 *	p_vect:				向量
 *	p0:					点坐标
 *	返回值：			p0到向量的垂直距离（以向量作为相对的Y轴，若p0的相对X坐标为负，则返回负值）
 */
int32_t p2v_vertical_dist(vect_t *p_vect, point_t *p0)
{
	point_t f_point;    /*footpoint*/
	vect_t  ver_vect;	/*垂线向量*/

	get_foot_point(p_vect, p0, &f_point);	/*垂线交点及相对位置*/
	set_vect(&ver_vect, &f_point, p0);	    /*点以及垂线交点构成的向量*/
							
	return  get_vect_len(&ver_vect);			/*长度*/
}

/*
 *	p2v_parallel_dist:	获取点到向量起点在平行于向量方向上的距离
 *	p_vect:				向量指针
 *	p0：				点坐标
 *	返回值：			p0与向量起点在平行于向量方向上的距离
 */
point_t p2v_foot_point_bluetooth;
int32_t p2v_parallel_dist(vect_t *p_vect, point_t *p0)
{
	point_t f_point;			/*垂线交点*/
	vect_t  par_vect;	   	/*经过点的平行于输入向量的向量*/

	get_foot_point(p_vect, p0, &f_point);		/*求垂线交点*/
	p2v_foot_point_bluetooth.x =f_point.x;
	p2v_foot_point_bluetooth.y =f_point.y;
	p2v_foot_point_bluetooth.ang =f_point.ang;
	
	set_vect(&par_vect, &p_vect->s, &f_point);	/*平行向量*/
	//判断两个向量是不是同向，如果是反向那么返回的距离值是负的
    if ((p_vect->e.x - p_vect->s.x) * (par_vect.e.x - par_vect.s.x) + 
        (p_vect->e.y - p_vect->s.y) * (par_vect.e.y - par_vect.s.y) < 0)      //修正BUG   
        return -get_vect_len(&par_vect);				/*长度*/	
    else 
        return get_vect_len(&par_vect);
}

uint32_t get_vect_len(vect_t *p_vect)
{
	int64_t x, y, l;
	x = p_vect->e.x - p_vect->s.x;
	y = p_vect->e.y - p_vect->s.y;
	l = my_sqrt(square(x)+square(y));
	return l;
}

uint32_t get_point_dis(point_t * p1, point_t * p2)
{
    vect_t vect;
    memcpy(&vect.s,p1,sizeof(point_t));
    memcpy(&vect.e,p2,sizeof(point_t));
    return get_vect_len(&vect);
}

uint8_t is_vect_same_direction(vect_t * v1, vect_t * v2)
{
    int32_t X1 = v1->e.x - v1->s.x;
    int32_t Y1 = v1->e.y - v1->s.y;
    int32_t X2 = v2->e.x - v2->s.x;
    int32_t Y2 = v2->e.y - v2->s.y;
    if (X1 * X2 + Y1 * Y2 >= 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*
 *	get_v2x_ang:	获取向量与X轴的夹角
 *	p_vect:			向量指针
 *	返回值：		向量与X轴的夹角【0-360°）(前开后闭)
 */
float get_v2x_ang(vect_t *p_vect)
{
	int32_t x = p_vect->e.x - p_vect->s.x;		/*输入向量的X分量*/
	int32_t y = p_vect->e.y - p_vect->s.y;		/*输入向量的Y分量*/
	
	/*判断角度区间，并用向量乘法计算与X轴的角度*/
	if (x == 0 && y == 0)
	{
		return 0;
	}								
	else if (y>=0)
	{
		return  my_acos((float)x / sqrt(square(x)+square(y)));
	}
	else
	{
		return 360 - my_acos((float)x / sqrt(square(x)+square(y)));
	}
}
