#include "convert.h"
#include "Arduino.h"

uint8_t checksum(uint8_t *buff, uint8_t len)
{
    uint8_t sum=0;
    for(uint8_t i=0;i<len;i++)
    {
       sum += buff[i];
    }
    return sum;
}

uint8_t char_hex_to_int(char c)
{
  if(c>='0' && c<='9')
  {
    return (c-'0');
  }
  else if(c>='a' && c<='f')
  {
    return (c-'a'+10);
  }
  else if(c>='A' && c<='F')
  {
    return (c-'A'+10);
  }
  else
  {
    return 16;
  }
}

bool convert_to_hex(char *input, uint8_t *output, uint8_t *len)
{
  uint8_t i=0;
  char *p = input;
  uint8_t byte;
  if(*p == ':')
  {
    p++;
    while(*p!='\0')
    {
      output[i++] = (char_hex_to_int(*p++) << 4) | char_hex_to_int(*p++);
    }
    if(checksum(output,i) == 0)
    {
      *len = i;
      return 1;
    }
  }
  return 0;
}

void console(uint8_t *buff, uint8_t len)
{
  for(uint8_t i=0; i<len; i++)
  {
    Serial.printf("%02x ", buff[i]);
  }
 Serial.print(" ");
}
