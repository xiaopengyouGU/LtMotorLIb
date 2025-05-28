#include "ltmotorlib.h"
#include "math.h"

lt_foc_t lt_foc_create()
{
	lt_foc_t foc;
	foc = rt_malloc(sizeof(struct lt_foc_object));
	if(foc == RT_NULL) return RT_NULL;
	rt_memset(foc,0,sizeof(struct lt_foc_object));
	
	return foc;
}

rt_err_t lt_foc_delete(lt_foc_t foc)
{
	RT_ASSERT(foc != RT_NULL);
	rt_free(foc);
	return RT_EOK;
}

void lt_foc_set_maxval(lt_foc_t foc,float max_val)
{
	RT_ASSERT(foc != RT_NULL);
	foc->max_val = _absf(max_val);
}

void lt_foc_set_type(lt_foc_t foc,rt_uint8_t type)
{
	RT_ASSERT(foc != RT_NULL);
	foc->type = type;
}

void lt_foc_process(lt_foc_t foc,float fd, float fq, float angle_el)
{
	RT_ASSERT(foc != RT_NULL);
	/* check angle_el */
	angle_el = _normalize_angle(angle_el);
	float f_alpha, f_beta;
	float _c, _s;						/* cos and sin */
	float center = foc->max_val/2;		
	float u_min, u_max;					
	
	fq = _constrains(fq,center,-center);	/* avoid output saturation */
	fd = _constrains(fd,center,-center);
	_c = cosf(angle_el);
	_s = sinf(angle_el);
	
	/* Park inverse transform */
	f_alpha = _c*fd - _s*fq;
	f_beta  = _s*fd + _c*fq;
	/* Clark inverse transform */
	foc->fa = f_alpha;
	foc->fb = -0.5f*f_alpha + SQRT_3/2*f_beta;
	foc->fc = -0.5f*f_alpha - SQRT_3/2*f_beta;
	
	switch(foc->type)
	{
		case FOC_TYPE_SVPWM:			/* we use center modulation */
		{
			u_min = _min(foc->fa,_min(foc->fb,foc->fc));
			u_max = _max(foc->fa,_max(foc->fb,foc->fc));
			center -= (u_max + u_min)/2;
			break;
		}
		default:break;
	}
	foc->fa += center;
	foc->fb += center;
	foc->fc += center;
	
}

void lt_foc_map_duty(lt_foc_t foc,float* dutyA, float* dutyB,float* dutyC)
{
	RT_ASSERT(foc != RT_NULL);
	if(foc->max_val == 0) 
	{
		*dutyA = 0;
		*dutyB = 0;
		*dutyC = 0;
	}
	else
	{
		*dutyA = foc->fa/foc->max_val;
		*dutyB = foc->fb/foc->max_val;
		*dutyC = foc->fc/foc->max_val;
	}
}
