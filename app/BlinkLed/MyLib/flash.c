#include "flash.h"

HAL_StatusTypeDef flash_lock()
{
	return HAL_FLASH_Lock();
}
HAL_StatusTypeDef flash_unlock()
{
	return HAL_FLASH_Unlock();
}
HAL_StatusTypeDef flash_erase(uint8_t nb_page,uint32_t addr)
{
	FLASH_EraseInitTypeDef erase_init;
	erase_init.NbPages = nb_page;
	erase_init.PageAddress = addr;//127
	erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
	uint32_t page_err;
	return HAL_FLASHEx_Erase(&erase_init,&page_err);
}
HAL_StatusTypeDef flash_write_arr(uint32_t addr,void *data, uint16_t size_data)
{
	uint16_t n = (size_data+1)/2-1;
	uint16_t i=0;
	HAL_StatusTypeDef sta;
	uint8_t*p= (uint8_t*)data;
	for(i=0;i < n;i++)
	{
		sta = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,addr,
											p[2*i]|((uint16_t)p[2*i+1]<<8));
		addr += 2;
	}
	if(size_data%2==0)//chan
	{
		sta =HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,addr,
											p[2*i]|((uint16_t)p[2*i+1]<<8));
	}
	else
	{
		sta =HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,addr,
											p[2*i]);
	}
	return sta;
}
void flash_read_arr(uint32_t addr,void *data, uint16_t size_data)
{
	uint16_t n = (size_data+1)/2-1;
	uint16_t i=0;
	uint16_t *val;
	uint8_t*p= (uint8_t*)data;
	for(i=0;i < n;i++)
	{
		val = (uint16_t *)addr;
		p[2*i] =  *val;
		p[2*i+1]= (*val) >> 8;
		addr += 2;
	}
	val = (uint16_t *)addr;
	if(size_data%2==0)//chan
	{
		p[2*i] =  *val;
		p[2*i+1]= (*val) >> 8;
		addr += 2;
	}
	else
	{
		p[2*i] =  *val;
	}
}
