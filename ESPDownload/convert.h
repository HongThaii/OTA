#include "stdint.h"
uint8_t checksum(uint8_t *buff, uint8_t len);
uint8_t char_hex_to_int(char c);
bool convert_to_hex(char *input, uint8_t *output, uint8_t *len);
void console(uint8_t *buff, uint8_t len);
