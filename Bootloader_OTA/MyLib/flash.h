#ifndef __FLASH_H__
#define __FLASH_H__
#include "main.h"
HAL_StatusTypeDef flash_unlock(void);
HAL_StatusTypeDef flash_lock(void);
HAL_StatusTypeDef flash_erase(uint8_t nb_page, uint32_t addr);
HAL_StatusTypeDef flash_write_arr(uint32_t addr, void *data, uint16_t size_data);
void flash_read_arr(uint32_t addr, void *data, uint16_t size_data);
#endif 