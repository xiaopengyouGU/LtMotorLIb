#include "key.h"

void key_init(void)
{
	rt_pin_mode(KEY0_PIN,PIN_MODE_INPUT_PULLDOWN);	//������Ĭ�ϵ͵�ƽ
	rt_pin_mode(KEY_UP_PIN,PIN_MODE_INPUT_PULLDOWN);//������Ĭ�ϵ͵�ƽ
}

rt_uint8_t key_scan(rt_uint8_t mode)	//����ɨ��
{
	static rt_uint8_t key_up = 1;	//�����ɿ���־
	rt_uint8_t key_val = 0;
	
	if(mode) key_up = 1;		//֧������
	
	if(key_up && (KEY0 == 1 ||KEY_UP == 1))	//��һ������������
	{
		rt_thread_mdelay(10);
		key_up = 0;
		if(KEY0 == 1) key_val = KEY0_PRES;
		if(KEY_UP == 1) key_val = KEY_UP_PRES;
	}
	else if(KEY0 == 0 && KEY_UP == 0)	//û�а�������
	{
		key_up = 1;
	}
	
	return key_val;
}
