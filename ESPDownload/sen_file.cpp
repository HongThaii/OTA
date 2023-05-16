#include "send_file.h"
#include "Arduino.h"
#include "convert.h"
#include "string.h"
char *token;
uint8_t uart_buff[128];
bool flag = 0;
uint8_t uart_index = 0;
uint8_t hex_data[21], len;
uint8_t retry = 0, retry_send = 0;

typedef enum
{
  SEND_FILE_STATE_IDLE,
  SEND_FILE_STATE_START,
  SEND_FILE_STATE_WAIT_ACK,
  SEND_FILE_STATE_SEND_DATA,
  SEND_FILE_STATE_SUCCESS
} SendfileState_t;

SendfileState_t send_file_state = SEND_FILE_STATE_IDLE;

//======================================== uart_send_data =============================

void uart_send_data(uint8_t *data, uint8_t len)
{
  Serial2.write(data, len);
}

//======================================== send_command_start ==============================

void send_command_start()
{
  uint8_t buff[4];
  buff[0] = START_SEND_FILE;
  buff[1] = 0;
  buff[2] = checksum(buff, 2);
  uart_send_data(buff, 3);
  Serial.printf("send_command_start: ");
  console(buff, 3);
}

//========================================= send_command_data ==============================

void send_command_data(uint8_t *data, uint8_t len)
{
    uint8_t buff[21];
    buff[0] = SEND_DATA;
    buff[1] = len;
    memcpy(&buff[2], data, len);
    buff[len + 2] = checksum(buff, len + 2);
    uart_send_data(buff, len + 3);
}

//========================================== send_command_done =============================

void send_command_done()
{
    uint8_t buff[4];
    buff[0] = SEND_DONE;
    buff[1] = 0;
    buff[2] = checksum(buff, 2);
    uart_send_data(buff, 3);
}

//======================================== receive_data_process =======================

void receive_data_process(uint8_t *data, uint8_t len)
{
  if(len = 4)
  {
    Serial.printf("receive_data_process: len =%d\n", len);
    console(data, len);
    Serial.println();
    switch(data[0])
    {
      case SEND_ACK:
      if(data[2] == ACK)
      {
        switch (send_file_state)
        {
          case SEND_FILE_STATE_WAIT_ACK:
          case SEND_FILE_STATE_START:
            send_file_state = SEND_FILE_STATE_SEND_DATA;
            break;
          case SEND_FILE_STATE_SEND_DATA:
  
            break;
          case SEND_FILE_STATE_SUCCESS:
            send_command_done();
            send_file_state = SEND_FILE_STATE_IDLE;
            Serial.println("finish");
            break;
        default:
          break;
        }
      }
      else
      {
        retry++;
        Serial.printf("retry: %d\n", retry);
        if (retry > 3)
        {
          retry = 0;
          Serial.println("send fail!");
          send_file_state = SEND_FILE_STATE_IDLE;
        }
        switch (send_file_state)
        {
          case SEND_FILE_STATE_START:
            send_command_start();
            break;
          case SEND_FILE_STATE_WAIT_ACK:
            send_command_data(hex_data, len);
            break;
          case SEND_FILE_STATE_SUCCESS:
            break;
          default:
            break;
        }
      }
      break;
    }
  }
}

//===================================== receive_data_handle =================================

void receive_data_handle()
{
  while (Serial2.available() > 0)
  {
    flag = 1;
    uart_buff[uart_index] = (uint8_t)Serial2.read(); // stm gui sangg;
    uart_index++;
  }
  if (flag)
  {
    receive_data_process(uart_buff, uart_index);
    uart_index = 0;
    flag = 0;
  }
}

//====================================== send_file_start ====================================

void send_file_start(char *file)
{
    /* lay token dau tien */
    if (send_file_state == SEND_FILE_STATE_IDLE)
    {
      Serial.println("send_file");
      token = strtok(file, "\n");
      send_command_start();
      send_file_state = SEND_FILE_STATE_START;
    }
}

//====================================== send_file_handle ===================================

void send_file_handle()
{
  switch (send_file_state)
  {
    case SEND_FILE_STATE_START:

      break;
    case SEND_FILE_STATE_SEND_DATA:
    {
      uint8_t record_type;
      if (convert_to_hex(token, hex_data, &len))
      { 
        console(hex_data, len);
        switch(hex_data[3])
        {
          case 0x00:
          {
            Serial.println();
            Serial.printf("Data send: ");
            console((hex_data+4), (len-5));
            send_command_data((hex_data+4), (len-5));
            Serial.println();
            token = strtok(NULL, "\n");
            if (token == NULL)
            {
              send_file_state = SEND_FILE_STATE_SUCCESS;
              Serial.println("send file done");
            }
            else
            {
              send_file_state = SEND_FILE_STATE_WAIT_ACK;
            }
            break;
          }
          case 0x01:
          case 0x05:
          {
            send_command_done();
            send_file_state = SEND_FILE_STATE_IDLE;
            Serial.println("finish by 0x01");
            break;
          }
          case 0x04:
          {
            token = strtok(NULL, "\n");
            Serial.println();
            if (token == NULL)
            {
              send_file_state = SEND_FILE_STATE_SUCCESS;
              Serial.println("send file done");
            }
            break;
          }
          default:
            break;
        }
      }
      else
      {
        Serial.println("file is corrupted!!!");
        retry_send++;
        Serial.printf("retry_send: %d\n", retry_send);
        if (retry_send > 3)
        {
          retry_send = 0;
          Serial.println("stop send file!");
          send_file_state = SEND_FILE_STATE_IDLE;
        }
      }
      break;
    }
    default:
      break;
  }
}
