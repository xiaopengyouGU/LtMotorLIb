#include "ltmotorlib.h"
#include "stdlib.h"
/* use FINSH to help test functions */
#ifdef RT_USING_FINSH
#ifdef LT_USING_MOTOR_MSH_TEST
#define MANAGER_UNVALID				0x00
#define MANAGER_INTRO				0x01
#define MANAGER_HELP				0x02
#define MANAGET_LIST				0x03

lt_manager_t _manager;

static lt_node_t _node_get(lt_node_t list,char* name)
{
	lt_node_t curr = list->next;
	rt_int32_t res;
	
	while(curr != list)
	{
		res = rt_strcasecmp(curr->name,name);		/* ascend sequence */
		if(res == 0) return curr;
		else if(res < 0)
		{
			curr = curr->next;
		}
		else break;		
	}
	return RT_NULL;
}


static lt_node_t _node_create(lt_motor_t motor,char*name)
{
	lt_node_t _node = rt_malloc(sizeof(struct lt_motor_node_object));
	if(_node == RT_NULL) return RT_NULL;
	rt_memset(_node,0,sizeof(struct lt_motor_node_object));
	_node->motor = motor;
	rt_strcpy(_node->name,name);
	_node->prev = _node;
	_node->next = _node;
	
	return _node;
}

static void _node_add(lt_node_t list,lt_node_t node)
{
	lt_node_t curr = list->next;
	rt_int32_t res;
	
	while(curr != list)	
	{
		res = rt_strcasecmp(curr->name,node->name);
		if(res == 0) return;
		if(res < 0)
		{
			curr = curr->next;
		}
		else				/* find pos */
		{
			break;
		}
	}
	/* in cycle list case, list->prev = end, this is so beautiful */
	curr->prev->next = node;
	node->prev = curr->prev;
	node->next = curr;
	curr->prev = node;
		
}

static void _node_delete(lt_node_t list,lt_node_t node)
{
	/* in cycle list case, list->prev = end, this is so beautiful */
	node->prev->next = node->next;
	node->next->prev = node->prev;
	rt_free(node);								/* release memory */
}


/* implements a cycle list */
int lt_manager_create(void)
{
	_manager = rt_malloc(sizeof(struct lt_motor_manager_object));
	if(_manager != RT_NULL) 
	{
		rt_memset(_manager,0,sizeof(struct lt_motor_manager_object));
		_manager->list = _node_create(RT_NULL,"");
		rt_kprintf("LtMotorLib motor manager is created successfully! \n");
		rt_kprintf("Input 'ltmotorlib' to start the journey!!! \n");
	}
	return RT_EOK;
}

rt_err_t lt_manager_add_motor(lt_motor_t motor)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_motor_get_info(motor,&(_manager->info));						/* get motor info */
	lt_node_t _node = _node_create(motor,_manager->info.name);		/* create a node */
	_node_add(_manager->list,_node);									/* add node to list */
	return RT_EOK;
}

rt_err_t lt_manager_delete_motor(lt_motor_t motor)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_motor_get_info(motor,&(_manager->info));							/* get motor info */
	lt_node_t _node = _node_get(_manager->list,_manager->info.name);		/* get node */
	if(_node != RT_NULL)
	{
		_node_delete(_manager->list,_node);
		return RT_EOK;
	}
	else
	{
		return RT_ERROR;
	}
}

lt_motor_t lt_manager_get_motor(char* name)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_node_t node = _node_get(_manager->list,name);
	if(node == RT_NULL) return RT_NULL;
	else return node->motor;
}

rt_err_t lt_manager_delete(void)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_node_t list = _manager->list;
	lt_node_t tmp = list;
	lt_node_t curr = list->next;
	while(curr != list)
	{
		tmp = curr->next;		/* save next pointer */
		_node_delete(list,curr);
		curr = tmp;
	}
	_node_delete(list,list);	/* delete head pointer */
	rt_free(_manager);			/* release memory */
	return RT_EOK;
}

/* list motors  */
static void _manager_info(int type)
{
	if(type == MANAGER_UNVALID)
	{
		rt_kprintf("unvalid cmd!!! \n");
		rt_kprintf("Input 'ltmotorlib help' to see more details \n");
	}
	else if(type == MANAGER_INTRO)
	{
		rt_kprintf("LtMotorLib --> A motor control library based on RT_Thread RTOS!!!\n ");
		rt_kprintf("Author: LvTou, Date: 2025/4/22, Version: 0.2 \n");
		rt_kprintf("LtMotorLib supports finsh and simple pid interface to help you control motors !\n ");
		rt_kprintf("Input like this 'ltmotorlib motor cmd' to call correspond functions \n");
		rt_kprintf("Input 'ltmotorlib list (motor)' to see motor(s) information \n");
		rt_kprintf("Input 'ltmotorlib test motor' to config test motor \n");
		rt_kprintf("Input 'ltmotorlib help' to see cmd called formats and details \n");
		rt_kprintf("Hope you enjoy it!!! \n\n");
	}
	else if(type == MANAGER_HELP)
	{
		rt_kprintf("Input 'ltmotorlib list' to see motor(s) information \n");
		rt_kprintf("Input 'ltmotorlib test motor' to config test motor \n");
		rt_kprintf("Note:  supported test motors: motor_dc, x_stepper, y_stepper \n");
		rt_kprintf("LtMotorLib provides simple pid interface for user \n");
		rt_kprintf("Notice that bellow 'motor' is your configured motor name!");
		rt_kprintf("ltmotorlib call formats: \n\n");
		rt_kprintf("******************* basic part ************************\n");
		rt_kprintf("ltmotorlib motor output val \n");
		rt_kprintf("ltmotorlib motor output_angle val \n");
		rt_kprintf("ltmotorlib motor output_pid val \n");
		rt_kprintf("ltmotorlib motor output_angle_pid val \n");
		rt_kprintf("ltmotorlib motor get_pos \n");
		rt_kprintf("ltmotorlib motor get_velocity \n");
		rt_kprintf("ltmotorlib motor disbale \n");
		rt_kprintf("******************* basic part ************************\n\n");
		rt_kprintf("******************* stepper specified part ************************\n");
		rt_kprintf("Note: the s_curve and 5_section output symmetric velocity curves and parameter units are as followed \n");
		rt_kprintf("trapzoid  --> step: step, acc: Hz/ms,   dec:  Hz/ms speed: Hz \n");
		rt_kprintf("s_curve   --> step: ms,   acc_t: ms,    freq: Hz, flexible: the bigger, the closer to s curve\n");
		rt_kprintf("5_section --> step: step, acc_t: ms,    speed: Hz \n\n");
		rt_kprintf("ltmotorlib motor trapzoid step acc dec speed \n");
		rt_kprintf("ltmotorlib motor s_curve step acc_t freq_max freq_min flexible \n");
		rt_kprintf("ltmotorlib motor 5_section step acc_t speed");
		rt_kprintf("ltmotorlib motor line_interp y_motor x_start, y_start, x_end, y_end \n");
		rt_kprintf("ltmotorlib motor circular_interp y_motor x_start y_start x_end y_end  radius dir(CW/CCW) \n");
		rt_kprintf("******************* stepper specified part ************************\n\n");
		rt_kprintf("******************* bldc specified part ************************\n");
		rt_kprintf("******************* bldc specified part ************************\n\n");
	}
	else if(type == MANAGET_LIST)
	{
		lt_node_t list = _manager->list;
		lt_node_t curr = list->next;
		struct lt_motor_info * info = &(_manager->info);
		rt_kprintf("%*s %15s %15s %15s rad \n",LT_NAME_MAX+5,"NAME","TYPE","STATUS","POSITION");
		while(curr != list)
		{
			lt_motor_get_info(curr->motor,info);
			rt_kprintf("%*s %15s %15s %15.2f \n",LT_NAME_MAX+5,info->name,_type[info->type],_status[info->status],info->position);
		}
	}
}
/* motor basic part */
static void _motor_basic(int argc,char *argv[],lt_motor_t motor, struct lt_motor_info* info)
{
	rt_uint8_t flag;
	if(!rt_strcmp(argv[2],"output"))
	{	/* motor basic part*/
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"output_angle"))
	{
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output_angle(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"get_pos"))
	{
		test_motor_get_position(motor);
		flag = 1;
	}
	else if(!rt_strcmp(argv[2],"get_vel"))
	{
		test_motor_get_velocity(motor);
		flag = 1;
	}
	else if(!rt_strcmp(argv[2],"output_pid"))
	{
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output_pid(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"output_angle_pid"))
	{
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output_angle_pid(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"disable"))
	{
		lt_motor_disable(motor);
		flag = 1;
	}
	if(!flag) _manager_info(MANAGER_UNVALID);
}
/* stepper motor part */
static void _motor_stepper_part(int argc,char *argv[],lt_motor_t motor, struct lt_motor_info* info)
{
	rt_uint8_t flag;
	if(!rt_strcmp(argv[2],"trapzoid"))
	{
		if(argc == 7)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc = (float)atof(argv[4]);
			float dec = (float)atof(argv[5]);
			float speed = (float)atof(argv[6]);
			test_stepper_trapzoid(motor,step,acc,dec,speed);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"s_curve"))
	{
		if(argc == 8)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc_t = (float)atof(argv[4]);
			float max = (float)atof(argv[5]);
			float min = (float)atof(argv[6]);
			float flex = (float)atof(argv[7]);
			test_stepper_s_curve(motor,step,acc_t,max,min,flex);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"5_section"))
	{
		if(argc == 6)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc_t = (float)atof(argv[4]);
			float speed = (float)atof(argv[5]);
			test_stepper_5_section(motor,step,acc_t,speed);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"line_interp") || !rt_strcmp(argv[2],"circular_interp"))
	{
		/* check y_motor */
		lt_motor_t y_motor = lt_manager_get_motor(argv[3]);
		if(y_motor == RT_NULL)
		{
			rt_kprintf("there is no motor named:% !!! \n",argv[3]);
			return;
		}
		else										/* check y_motor type*/
		{
			lt_motor_get_info(y_motor,info);		/* get motor info */
			if(info->type != MOTOR_TYPE_STEPPER)	
			{
				rt_kprintf("motor %s type is incorrect! expected: %s , actual: %s !!!\n",info->name, _type[MOTOR_TYPE_STEPPER],_type[info->type]);
				return;
			}
		}
		/* check params*/
		if(argc == 8 || argc == 10)	
		{
			int x_start = atoi(argv[4]);
			int y_start = atoi(argv[5]);
			int x_end = atoi(argv[6]);
			int y_end = atoi(argv[7]);
			if(argc == 8)				/* line interp */
			{
				test_stepper_line_interp(motor,y_motor,x_start,y_start,x_end,y_end);
			}
			else						/* circular interp */
			{
				float radius = atoi(argv[8]);
				rt_uint8_t dir;
				if(!rt_strcmp(argv[9],"CW"))
				{
					dir = DIR_CW;
					flag = 1;
				}
				else
				{
					dir = DIR_CCW;
					flag = 1;
				}
				test_stepper_circular_interp(motor,y_motor,x_start,y_start,x_end,y_end,radius,dir);
			}
			flag = 1;
		}
	}
	if(!flag) _manager_info(MANAGER_UNVALID);
}

static void _motor_test_config(char* name)
{
	lt_motor_t motor = lt_manager_get_motor(name);
	if(motor != RT_NULL)
	{
		rt_kprintf("already config motor: %s !!! \n",name);
		return;
	}
	
	if(!rt_strcmp(name,"motor_dc"))
	{
		test_motor_dc_config();
	}
	else if(!rt_strcmp(name,"x_stepper"))
	{
		test_stepper_x_config();
	}
	else if(!rt_strcmp(name,"y_stepper"))
	{
		test_stepper_y_config();
	}
	else
	{
		rt_kprintf("there is no test motor named:% !!! \n",name);
	}
}


static void ltmotorlib(int argc, char*argv[])
{
	RT_ASSERT(_manager != RT_NULL);
	lt_motor_t motor;
	struct lt_motor_info* info = &(_manager->info);
	
	if(argc == 1)
	{
		_manager_info(MANAGER_INTRO);
		return;
	}
	else if(argc == 2)
	{
		if(!rt_strcmp(argv[1],"list"))
		{
			_manager_info(MANAGET_LIST);
		}
		else if(!rt_strcmp(argv[1],"help"))
		{
			_manager_info(MANAGER_HELP);
		}
		else
		{
			_manager_info(MANAGER_UNVALID);
		}
		return;
	}
	
	if(argc == 3 && !rt_strcmp(argv[1],"test"))
	{
		_motor_test_config(argv[2]);
		return;
	}
	/* find motor */
	motor = lt_manager_get_motor(argv[1]);
	if(motor == RT_NULL)
	{
		rt_kprintf("there is no motor named:% !!! \n",argv[1]);
		return;
	}
	lt_motor_get_info(motor,info);				/* get motor info */
	
	/* check command */
	if(!rt_strcmp(argv[2],"output") || !rt_strcmp(argv[2],"output_angle") || !rt_strcmp(argv[2],"get_pos") || !rt_strcmp(argv[2],"get_vel") || !rt_strcmp(argv[2],"output_pid") || !rt_strcmp(argv[2],"output_angle_pid"))
	{	/* motor basic part*/
		_motor_basic(argc,argv,motor,info);
	}
	else if(!rt_strcmp(argv[2],"trapzoid") || !rt_strcmp(argv[2],"s_curve") || !rt_strcmp(argv[2],"line_interp") || !rt_strcmp(argv[2],"circular_interp"))
	{	/* stepper motor part */
		if(info->type != MOTOR_TYPE_STEPPER)	/* check motor type */
		{
			rt_kprintf("motor %s type is incorrect! expected: %s , actual: %s !!!\n",info->name, _type[MOTOR_TYPE_STEPPER],_type[info->type]);
			return;
		}
		_motor_stepper_part(argc,argv,motor,info);
	}
	
}
MSH_CMD_EXPORT(ltmotorlib, A powerful motor control library);
INIT_DEVICE_EXPORT(lt_manager_create);			/* create motor manager automatically */
#endif
#endif
