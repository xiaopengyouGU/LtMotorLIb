#include "ltmotorlib.h"
#include "math.h"

float _constrains(float val, float up_limit, float down_limit)
{
	if(up_limit <= down_limit) return val;
	
	if(val > up_limit)
	{
		val = up_limit;
	}
	else if(val < down_limit)
	{
		val = down_limit;
	}
	
	return val;
}

float _constrains_dead_region(float val,float up_limit, float down_limit)
{
	if(up_limit <= down_limit) return val;
	
	if(val > up_limit)
	{
		val = up_limit;
	}
	else if(val < down_limit)
	{
		val = 0;
	}
	
	return val;
}

rt_uint8_t _get_rotation_dir(float *input)
{
	float val = *input;
	if(val == 0)
	{
		return 0;
	}
	else if(val > 0) 	/* forward rotate */
	{
		return ROT_FORWARD;
	}
	else if(val < 0)  	/* reversal rotate */
	{
		*input = -val;
		return ROT_REVERSAL;
	}
}


rt_uint8_t _get_quard(rt_int32_t x_pos, rt_int32_t y_pos)
{
	if(x_pos >= 0)
	{
		if(y_pos >= 0)	return 0;	/* 1th quadrant */
		else			return 3;	/* 4th quadrant */
	}
	else
	{
		if(y_pos >= 0)	return 1;	/* 2th quadrant */
		else 			return 2;	/* 3th quadrant */
	}
}

/* check whether start pos and end pos in the same quarent and circle */
rt_uint8_t _check_pos(rt_int32_t x_start, rt_int32_t y_start, rt_int32_t x_end, rt_int32_t y_end)
{
	float tmp = sqrtf(x_start*x_start + y_start*y_start);/* get radius */
	float ref = tmp/100;
	tmp -= sqrtf(x_end*x_end + y_end*y_end);			
	/* if radius bias are lower than %1, we regard that two points are in the same circle */
	if(tmp >= ref || tmp <= -ref) return 0;	
	
	/* x_pos has same sign? */
	if(x_start > 0)
	{
		if(x_end < 0) return 0;
	}
	else if(x_start < 0)
	{
		if(x_end > 0) return 0;
	}
	/* y_pos has same sign? */
	if(y_start > 0)
	{
		if(y_end < 0) return 0;
	}
	else if(y_start < 0)
	{
		if(y_end > 0) return 0;
	}
	
	return 1;
}