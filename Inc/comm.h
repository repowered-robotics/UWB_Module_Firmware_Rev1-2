#ifndef _COMM_H_
#define _COMM_H_

#include "inttypes.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "rtls.h"
//#include "stm32f1xx_ll_usb.h"
//#include "usb_device.h"
//#include "usbd_cdc_if.h"
//#include "usbd_conf.h"
//#include "usbd_desc.h"

#define ANCHOR_INFO_SIZE 	33
#define INPUT_TIMEOUT 		10
#define INPUT_BUFFER_SIZE 	512
#define HDDR_LEN 			2
#define STOP_BYTE 			0xA5
#define SEND_BUF_SIZE 		1024

enum CMD_TYPES {
	CMD_READ_CONFIG		= 0x11,
	CMD_READ_ANCHORS 	= 0x12,
	CMD_SET_CONFIG 		= 0x22,
	CMD_RANGE 			= 0x33,
	CMD_RESTART 		= 0x44,
	CMD_RESET 			= 0x55,
	CMD_SAVE_CONFIG 	= 0x66
};

enum PROCESS_PACKET_STATUS {
	PACKET_OK,
	PACKET_CMD_ERROR,
	PACKET_LENGTH_ERROR
};

// extern uint8_t usb_rx_buffer[];

int serialize_device_config(void* self_data, void* config);
int serialize_anchor_data(void* anchor_data, AnchorData* anchor_list, int list_size);
int process_packet(uint8_t* pack_in, uint8_t* pack_out, CONFIG_FIELD_TYPE* fields, state_data_t* state);

#endif
