/*
 * comm.c
 *
 *  Created on: Aug 21, 2019
 *      Author: Adam Cordingley
 */
#include <comm.h>

// uint8_t usb_rx_buffer[INPUT_BUFFER_SIZE];

int process_packet(uint8_t* pack_in, uint8_t* pack_out, CONFIG_FIELD_TYPE* fields, state_data_t* state){
	int ind = 0;
	int retval = PACKET_OK;
	switch(pack_in[0]){
	case CMD_READ_CONFIG:{
		pack_out[ind++] = pack_in[0];
		pack_out[ind++] = pack_in[1]*(FIELD_SIZE*sizeof(CONFIG_FIELD_TYPE) + 1); // set body length in bytes
		int i;
		for(i = HDDR_LEN; i < pack_in[1] + HDDR_LEN; i++){
			pack_out[ind++] = pack_in[i]; 					// copy field ID
			get_field(fields, pack_in[i], (void*)(pack_out + ind)); 	// copy field value
			ind += FIELD_SIZE;
		}
		pack_out[ind++] = STOP_BYTE;
		break;}
	case CMD_READ_ANCHORS:{
		pack_out[ind++] = pack_in[0];
		//get_field(fields, FIELD_NUMBER_OF_ANCHORS, (void*)num_anchors);
		int body_size = serialize_anchor_data(pack_out + HDDR_LEN, state->anchors, state->num_anchors);
		pack_out[ind++] = body_size; // length of the body
		ind += body_size;
		pack_out[ind] = STOP_BYTE;
		break;}
	case CMD_SET_CONFIG:{
		int i;
		for(i = HDDR_LEN; i < pack_in[1] + HDDR_LEN; i += FIELD_SIZE + 1){
			set_field(fields, pack_in[i], (void*)(pack_in + i + 1));
		}
		pack_out[0] = pack_in[0];
		pack_out[1] = 0;
		pack_out[2] = STOP_BYTE;
		break;}
	case CMD_RANGE:
		break;
	case CMD_RESTART:
		break;
	case CMD_RESET:
		break;
	case CMD_SAVE_CONFIG:
		//save_fields_to_eeprom(fields);
		pack_out[0] = pack_in[0];
		pack_out[1] = 0;
		pack_out[2] = STOP_BYTE;
		break;
	default:
		pack_out[0] = 0xFF;
		pack_out[1] = 0;
		pack_out[2] = STOP_BYTE;
//		retval = PACKET_CMD_ERROR;
		break;
	}
	return retval;
}


int serialize_anchor_data(void* anchor_data, AnchorData* anchor_list, int list_size){
	int ind = 0;
	int size = 0;
	for(int i = 0; i < list_size; i++){
		if(anchor_list[i].distance < 0.0 || anchor_list[i].timestamp == 0){
			continue;
		}
		memcpy(anchor_data + ind, &(anchor_list[i].id), size = sizeof(anchor_list[i].id));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].timestamp), size = sizeof(anchor_list[i].timestamp));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].x), size = sizeof(anchor_list[i].x));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].y), size = sizeof(anchor_list[i].y));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].z), size = sizeof(anchor_list[i].z));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].distance), size = sizeof(anchor_list[i].distance));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].rx_power), size = sizeof(anchor_list[i].rx_power));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].fp_power), size = sizeof(anchor_list[i].fp_power));
		ind += size;
		memcpy(anchor_data + ind, &(anchor_list[i].fp_snr), size = sizeof(anchor_list[i].fp_snr));
		ind += size;
	}
	return ind;
}
