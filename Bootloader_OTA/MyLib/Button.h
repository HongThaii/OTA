#include "main.h"
#define BTN_TIME_PRESS_TIMEOUT	3000
typedef struct
{
	uint8_t btn_curent;
	uint8_t btn_last;//=1
	uint8_t btn_filter; //=1
	uint32_t t_debounce;
	uint32_t t_start_press;
	uint32_t t_time_out;
	uint32_t t_double_click;
	uint16_t t_repeate_timeout;
	uint16_t t_timeout_press;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
}Button_t;

void button_handle(Button_t *btn);
void button_init(Button_t *btn, GPIO_TypeDef *_GPIOx, uint16_t _GPIO_Pin);
