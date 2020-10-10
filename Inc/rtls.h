#ifndef _RTLS_H_
#define _RTLS_H_

#include "stm32f1xx_hal.h"
#include <inttypes.h>
#include "deca_port.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_mac.h"
#include "eeprom.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define DEVICE_MODE_TAG 	0x00
#define DEVICE_MODE_ANCHOR 	0x01

#define TX_BUFFER_SIZE 		1024

#define DFLT_SELF_ID 			0xDF
#define DFLT_MODE 	 			DEVICE_MODE_TAG
#define DFLT_CHANNEL 			((uint8_t)2)
#define DFLT_SAMPLES_PER_RANGE 	3
#define DFLT_NUMBER_OF_ANCHORS 	5
#define DFLT_RANGING_PERIOD 	50
#define RANGING_TIMEOUT 		25

/* Type of the config buffer */
#define CONFIG_FIELD_TYPE uint8_t
/* number of config fields */
#define NUM_FIELDS 			8
/* size of each field data */
#define FIELD_SIZE 			4
/* bytes to occupy */
#define FIELD_MEM_SIZE 		NUM_FIELDS*FIELD_SIZE

/* how many anchors to store in our list */
#define ANCHOR_LIST_SIZE 		10

/* number of tag transactions to queue up */
#define TAG_QUEUE_SIZE 			4

/* max number to be ranging with (restricted by our comm buffer size) */
#define MAX_NUMBER_OF_ANCHORS 	7

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 	1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 		1000

#define FRAME_LEN_MAX 		127

#define DELAY_BEFORE_TX 	0

#define SAMPLES_PER_POINT 	2
/* Change to match the device you're programming */
#define XTAL_TRIM 			15

/* Change to match which device you're programming */
#define TX_ANT_DLY 			16442
#define RX_ANT_DLY 			16442

/* Broadcast period in milliseconds */
#define ANCHOR_BROADCAST_PERIOD 1000

/* time to wait after TX to turn on RX */
#define TX_TO_RX_DELAY_UUS  50

/* weights for our confidence calculation */
#define K1 					100.0
#define K2 					1.0

#define FRAME_TYPE_DATA 	0x01
#define FRAME_TYPE_BEACON 	0x00

#define DISTANCE_FROM_TOF(tof) 	(float)(299792458.0 * (1.0*tof / (128.0*499200000.0)))

/* config fields */
enum FIELDS {
	FIELD_SELF_ID 	= 0x00,
	FIELD_MODE 		= 0x01,
	FIELD_CHANNEL 	= 0x02,
	FIELD_SAMPLES_PER_RANGE 	= 0x03,
	FIELD_NUMBER_OF_ANCHORS 	= 0x04,
	FIELD_X = 0x05,
	FIELD_Y = 0x06,
	FIELD_Z = 0x07
};

typedef struct {
	uint8_t type;
	uint8_t anchor_id;
	float distance;
	int16 confidence;
} DistanceFrame;

typedef struct AnchorTimeStamps{
	unsigned long long t_rp;
	unsigned long long t_sr;
	unsigned long long t_rf;
} AnchorTimeStamps;

typedef struct BeaconTimeStamps{
	unsigned long long t_sp;
	unsigned long long t_rr;
	unsigned long long t_sf;
	unsigned long long t_ff;
} BeaconTimeStamps;

typedef struct point {
	float x;
	float y;
	float z;
}point_t;

typedef struct AnchorData {
	uint8_t id;
	int timeout_count;
	bool is_alive;
	uint32_t timestamp;
	float x;
	float y;
	float z;
	float distance;
	float rx_power;
	float fp_power;
	float fp_snr;
}AnchorData;

typedef struct TagData {
	_Bool active; 				// indicate whether this tag needs to be ranged with
	uint16_t id; 				// tag id
	unsigned long long t_rp; 	// receive poll timestamp
	unsigned long long t_sr; 	// send response timestamp
	unsigned long long t_rf; 	// receive final timestamp
}TagData;

typedef enum STATE {
	IDLE,
	WAIT_FOR_POLL,
	WAIT_FOR_RESPONSE,
	WAIT_FOR_FINAL,
	WAIT_FOR_DATA
}state_t;

typedef enum TX_STATUS {
	TX_SUCCESS,
	TX_TIMEOUT
} TxStatus;

typedef enum RX_STATUS {
	RX_TIMEOUT,
	RX_DATA_FRAME_READY,
	RX_ERROR
} RxStatus;

typedef struct state_data {
	state_t state;
	state_t last_state;
	uint8_t mode;
	uint16_t self_id; 		// id of self
	uint8_t channel; 		// UWB channel
	uint16_t transact_id; 	// id of the other node in the transaction
	int seq_num; 			// the sequence number
	_Bool ranging; 			// flag to indicate that we're currently ranging
	uint32_t ranging_period; // delay between ranging operations
	// uint32_t timer_start; 	// used to keep track of timeouts
	// uint32_t broadcast_timer;// for anchors to track broadcast intervals
	AnchorData* anchors; 	// array of anchors
	TagData* tags; 			// array of tags, if we're in anchor mode

	int n_range_with; 		// how many anchors to range with
	int num_anchors; 		// how many anchors are in our anchors array
	int anchor_ind; 		// current index in anchors array
	int tag_ind; 			// current index in tag queue, used if we're in anchor mode

	// Anchor timestamps
	unsigned long long t_rp; // receive poll
	unsigned long long t_sr; // send response
	unsigned long long t_rf; // receive final

	// TAG timestamps
	unsigned long long t_sp; // send poll
	unsigned long long t_rr; // receive response
	unsigned long long t_sf; // send final
	unsigned long long t_ff; // i forget... not used currently iirc

	int64_t tof;
	float x;
	float y;
	float z;
	float distance;

	_Bool new_frame;
	uint8_t tx_buffer[FRAME_LEN_MAX];
	TxStatus tx_status;
	uint8_t rx_buffer[FRAME_LEN_MAX];
	RxStatus rx_status;
}state_data_t;

typedef enum RANGING_STATUS {
	RANGING_SUCCESS,
	SEND_POLL_FAILED,
	RECEIVE_POLL_FAILED,
	RECEIVE_RESPONSE_FAILED,
	SEND_FINAL_FAILED,
	RECEIVE_FINAL_FAILED,
	RECEIVE_TIMESTAMPS_FAILED
}RangingStatus;

typedef enum MESSAGE_TYPES {
	POLL,
	RESPONSE_INIT,
	SEND_FINAL,
	RESPONSE_DATA,
	ANCHOR_BROADCAST
}MessageType;

// extern state_data_t state_data;
extern CONFIG_FIELD_TYPE self_config[]; 	// configuration for ourselves
extern AnchorData* anchor_data;


void set_state(state_data_t* sd, state_t state);
void set_state_idle(state_data_t* state);
void set_state_tag_idle(state_data_t* sd);
void set_state_anchor_idle(state_data_t* state);
void set_wait_for_poll(state_data_t* sd);
void set_wait_for_final(state_data_t* sd);
void set_wait_for_repsonse(state_data_t* sd);
void set_wait_for_data(state_data_t* sd);
void set_state_process_rx_frame(state_data_t* state);
void process_anchor_broadcast(state_data_t* state);
void tag_wait_timeout(state_data_t* state);
void anchor_wait_timeout(state_data_t* state);
void next_anchor(state_data_t* state);
int find_tag(uint16_t id, TagData* tags);
int add_tag(TagData tag, TagData* tags);
int  check_rx_state();
void disable_ranging(state_data_t* state);
void enable_ranging(state_data_t* state);
int read_rx_frame(uint8* buffer);
void rtls_fsm(state_t* state);
void send_response_init(state_data_t* state_data);
void send_response_data(state_data_t* state_data);
void send_response_final(state_data_t* state_data);
void send_anchor_broadcast(state_data_t* state);
RangingStatus range_with_anchor(uint8_t anchor_id, AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps, uint8* seq_num);
unsigned long long get_tx_timestamp(void);
unsigned long long get_rx_timestamp(void);
TxStatus transmit_frame(uint8* frame, int f_len, _Bool ranging);
int receive_frame(uint8* buffer, int max_len, int timeout);
double get_rx_power(dwt_rxdiag_t* diagnostics);
double get_fp_power(dwt_rxdiag_t* diagnostics);
double get_fp_snr(dwt_rxdiag_t* diagnostics);
int16 get_confidence(double rx_power, double fp_power, double snr);
int64_t get_tof(state_data_t* state);
void u_delay(int usec);
uint8 rtls_make_mac_header(state_data_t* state, uint8 frame_type);
AnchorData get_anchor_from_frame(uint8_t* buffer);
float get_distance(point_t a, point_t b);
void read_config_from_eeprom(CONFIG_FIELD_TYPE* config);
void save_fields_to_eeprom(CONFIG_FIELD_TYPE* fields);
void init_anchor_array(AnchorData* anchors, int len);
void init_field_memory(CONFIG_FIELD_TYPE* fields);
void init_from_fields(void* fields, dwt_config_t* dw_config);
void init_from_config(CONFIG_FIELD_TYPE* config, dwt_config_t* dw_config, state_data_t* state);
void set_field(CONFIG_FIELD_TYPE* fields, int field_id, void* value);
void get_field(CONFIG_FIELD_TYPE* fields, int field_id, void* value);
int rx_power_comparator(const void* p1, const void* p2);
int rx_qual_comp(const void* p1, const void* p2);
void sort_anchors_by_rx_power(AnchorData* anchors, int size);
void sort_anchors_by_rx_qual(AnchorData* anchors, int size);
void add_new_anchor(state_data_t* state, AnchorData new_anchor);
int remove_dead_anchors(AnchorData* anchors, int len);

#endif
