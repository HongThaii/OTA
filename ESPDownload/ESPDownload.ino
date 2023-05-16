 #include "convert.h"
#include "download_file.h"
#include "send_file.h"

void setup()
{
    Serial.begin(115200); // esp to pc
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // esp to stm
    pinMode(4, OUTPUT);
    digitalWrite(4, 1);
}

void loop()
{
    download_file();
    send_file_handle();
    receive_data_handle();
}
