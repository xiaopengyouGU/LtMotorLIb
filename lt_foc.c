#include "ltmotorlib.h"
#include "math.h"

lt_foc_t lt_foc_create(float max_val)
{
	lt_foc_t foc;
	foc = rt_malloc(sizeof(struct lt_foc_object));
	if(foc == RT_NULL) return RT_NULL;
	rt_memset(foc,0,sizeof(struct lt_foc_object));
	foc->max_val = _fabs(max_val);
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
	foc->max_val = _fabs(max_val);
}

void lt_foc_process(lt_foc_t foc, float angle_el, float fd, float fq)
{
	RT_ASSERT(foc != RT_NULL);
	/* check angle_el */
	angle_el = _abs(angle_el);
	rt_uint16_t n = angle_el/(2*PI);
	float f_alpha, f_beta;
	float _c, _s;						/* cos and sin */
	
	fq = _constrains(fq,0,foc->max_val);
	fd = _constrains(fd,0,foc->max_val);
	angle_el = angle_el - n*2*PI;		/*let angle_el be 0~2pi */
	_c = cosf(angle_el);
	_s = sinf(angle_el);
	
	/* Park inverse transform */
	f_alpha = _c*fd - _s*fq;
	f_beta  = _s*fd + _c*fq;
	/* Clark inverse transform */
	foc->fa = f_alpha;
	foc->fb = -1.0f/2*f_alpha + SQRT_3/2*f_beta;
	foc->fc = -1.0f/2*f_alpha - SQRT_3/2*f_beta;
}

void lt_foc_map_duty(lt_foc_t foc,float* dutyA, float* dutyB,float* dutyC)
{
	RT_ASSERT(foc != RT_NULL);
	if(foc->max_val != 0) 
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
