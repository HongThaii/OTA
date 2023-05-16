enum
{
    START_SEND_FILE = 0x01,
    SEND_DATA = 0x02,
    SEND_DONE = 0x03,
    SEND_ACK = 0x04,
};
#define ACK	0
#define NACK 1
void send_file_handle();
void send_file_start(char *file);
void receive_data_handle();
