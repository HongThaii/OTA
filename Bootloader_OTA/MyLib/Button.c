#include "button.h"

//var button

__attribute__((weak)) void btn_pressing_callback(Button_t *btn){}
__attribute__((weak)) void btn_press_short_callback(Button_t *btn){}
__weak void btn_press_long_callback(Button_t *btn,uint16_t time_press){}
__weak void btn_realease_callback(Button_t *btn){}
__weak void btn_pressing_timeout_callback(Button_t *btn,uint16_t time_press){}
__weak void btn_double_click_callback(Button_t *btn){}
	
void button_handle(Button_t *btn)
{
	uint8_t sta = HAL_GPIO_ReadPin(btn->GPIOx,btn->GPIO_Pin);
	if(sta != btn->btn_filter)
	{
		btn->btn_filter = sta;
		btn->t_debounce = HAL_GetTick();
	}
	if(HAL_GetTick() - btn->t_debounce >= 15)
	{
		btn->btn_curent = btn->btn_filter;
		if(btn->btn_curent != btn->btn_last)
		{
			if(btn->btn_curent == 0)
			{
				btn->t_start_press = HAL_GetTick();
				btn->t_time_out = btn->t_start_press;
				btn->t_repeate_timeout = BTN_TIME_PRESS_TIMEOUT;
				btn->t_timeout_press = btn->t_repeate_timeout;
				btn_pressing_callback(btn);
			}
			else //nha nut
			{
				uint16_t time_pressed = HAL_GetTick() - btn->t_start_press;
			  if(time_pressed <= 1000)
			  {
					if(HAL_GetTick() - btn->t_double_click <= 500)
					{
						btn_double_click_callback(btn);
					}
					btn->t_double_click = HAL_GetTick();
					btn_press_short_callback(btn);
			  }
				else
				{
				 btn_press_long_callback(btn,time_pressed);
				}
				btn_realease_callback(btn);
			}
			btn->btn_last = btn->btn_curent;
		}
		if((btn->btn_curent ==0) && (HAL_GetTick() - btn->t_time_out >= btn->t_repeate_timeout))
		{
			btn_pressing_timeout_callback(btn,btn->t_timeout_press);
			btn->t_repeate_timeout = 1000;
			btn->t_timeout_press += btn->t_repeate_timeout;
			btn->t_time_out = HAL_GetTick();
		}
	}
}
void button_init(Button_t *btn,GPIO_TypeDef *_GPIOx, uint16_t _GPIO_Pin)
{
	btn->GPIOx = _GPIOx;
	btn->GPIO_Pin = _GPIO_Pin;
	btn->btn_last = 1;
	btn->btn_filter = 1;
}
