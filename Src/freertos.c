/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include <inttypes.h>
#include "semphr.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "comm.h"
#include "rtls.h"
#include "usart.h"
#include "printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IF_TYPE_USB 	0x01
#define IF_TYPE_UART 	0x02
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t* debug_buf;
int debug_buf_offset = 0; // indicates length of the string in debug_buf
char debug_text[80];
char scratch_buf[128];
int print_ind = 0;

static dwt_config_t config = {
		2,               /* Channel number. */
		DWT_PRF_64M,     /* Pulse repetition frequency. */
		DWT_PLEN_128,   /* Preamble length. Used in TX only. */
		DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
		9,               /* TX preamble code. Used in TX only. */
		9,               /* RX preamble code. Used in RX only. */
		0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
		DWT_BR_6M8,      /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		(1025 + 64)    	 /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

state_data_t 	state_data;
CONFIG_FIELD_TYPE self_config[FIELD_SIZE*NUM_FIELDS]; 	// configuration for ourselves
extern AnchorData* anchor_data;
extern TagData* tag_data;
extern uint8_t UserRxBufferFS[];
extern uint8_t UserTxBufferFS[];
state_t 		state 		= IDLE;
HAL_StatusTypeDef 	UART_status;
HAL_StatusTypeDef 	uart2_status; 	// status of uart2
int8_t 				usb_status;
uint32_t 			bytes_read; 	//
uint8_t packet[INPUT_BUFFER_SIZE]; 	// will hold received packet from host
uint8_t packet_type = 0; 			// type of received packet
int packet_len 	= 0; 			// length of received packet
int packet_rcvd 	= 0; 			// indicate if packet has been received and how to process it
uint8_t send_buf[SEND_BUF_SIZE]; 	// lorge buffer for response packet
int usb_ind = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 200 * 4
};
/* Definitions for userIOTask */
osThreadId_t userIOTaskHandle;
const osThreadAttr_t userIOTask_attributes = {
  .name = "userIOTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 200 * 4
};
/* Definitions for rangingTimer */
osTimerId_t rangingTimerHandle;
const osTimerAttr_t rangingTimer_attributes = {
  .name = "rangingTimer"
};
/* Definitions for broadcastTimer */
osTimerId_t broadcastTimerHandle;
const osTimerAttr_t broadcastTimer_attributes = {
  .name = "broadcastTimer"
};
/* Definitions for rangingTimeoutTimer */
osTimerId_t rangingTimeoutTimerHandle;
const osTimerAttr_t rangingTimeoutTimer_attributes = {
  .name = "rangingTimeoutTimer"
};
/* Definitions for dw_irq_sem */
osSemaphoreId_t dw_irq_semHandle;
const osSemaphoreAttr_t dw_irq_sem_attributes = {
  .name = "dw_irq_sem"
};
/* Definitions for debug_bin_sem */
osSemaphoreId_t debug_bin_semHandle;
const osSemaphoreAttr_t debug_bin_sem_attributes = {
  .name = "debug_bin_sem"
};
/* Definitions for doRanging */
osSemaphoreId_t doRangingHandle;
const osSemaphoreAttr_t doRanging_attributes = {
  .name = "doRanging"
};
/* Definitions for doBroadcast */
osSemaphoreId_t doBroadcastHandle;
const osSemaphoreAttr_t doBroadcast_attributes = {
  .name = "doBroadcast"
};
/* Definitions for timeoutSem */
osSemaphoreId_t timeoutSemHandle;
const osSemaphoreAttr_t timeoutSem_attributes = {
  .name = "timeoutSem"
};
/* Definitions for usbDataReadySem */
osSemaphoreId_t usbDataReadySemHandle;
const osSemaphoreAttr_t usbDataReadySem_attributes = {
  .name = "usbDataReadySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool debug_put(char* text);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartUserIOTask(void *argument);
void rangingTimerCallback(void *argument);
void broadcastTimerCallback(void *argument);
void timeoutCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dw_irq_sem */
  dw_irq_semHandle = osSemaphoreNew(1, 1, &dw_irq_sem_attributes);

  /* creation of debug_bin_sem */
  debug_bin_semHandle = osSemaphoreNew(1, 1, &debug_bin_sem_attributes);

  /* creation of doRanging */
  doRangingHandle = osSemaphoreNew(1, 1, &doRanging_attributes);

  /* creation of doBroadcast */
  doBroadcastHandle = osSemaphoreNew(1, 1, &doBroadcast_attributes);

  /* creation of timeoutSem */
  timeoutSemHandle = osSemaphoreNew(1, 1, &timeoutSem_attributes);

  /* creation of usbDataReadySem */
  usbDataReadySemHandle = osSemaphoreNew(1, 1, &usbDataReadySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of rangingTimer */
  rangingTimerHandle = osTimerNew(rangingTimerCallback, osTimerPeriodic, NULL, &rangingTimer_attributes);

  /* creation of broadcastTimer */
  broadcastTimerHandle = osTimerNew(broadcastTimerCallback, osTimerPeriodic, NULL, &broadcastTimer_attributes);

  /* creation of rangingTimeoutTimer */
  rangingTimeoutTimerHandle = osTimerNew(timeoutCallback, osTimerOnce, NULL, &rangingTimeoutTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of userIOTask */
  userIOTaskHandle = osThreadNew(StartUserIOTask, NULL, &userIOTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  init_anchor_array(anchor_data, ANCHOR_LIST_SIZE + 1);
  state_data.anchors = anchor_data;
  state_data.tags = tag_data;
  init_field_memory(self_config); // set everything to defaults
  read_config_from_eeprom(self_config); 	// pull any existing values from flash
  init_from_config(self_config, &config, &state_data); // inititalize the DWM1000 from our fields

  osDelay(state_data.self_id * 10); // stagger startup times

  if(state_data.mode == DEVICE_MODE_TAG){
		osTimerStart(rangingTimerHandle, state_data.ranging_period); // start the ranging timer
  }else{
		osTimerStart(broadcastTimerHandle, ANCHOR_BROADCAST_PERIOD + 10*state_data.self_id); // start the broadcast timer
  }
  /* Infinite loop */
  for(;;)
  {
    // Check if we've received a frame
    if(osSemaphoreAcquire(dw_irq_semHandle, 20) == osOK && read_rx_frame(state_data.rx_buffer) > 0){
      // we've received a frame
      // debug_put("Frame received\r\n");
			if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == ANCHOR_BROADCAST){
        // debug_put("Processing Broadcast\r\n");
				process_anchor_broadcast(&state_data); // this also reenables receiver
			}else if(state_data.mode == DEVICE_MODE_TAG){
        // ==== NEW FRAME STATE MACHINE ====
        switch(state_data.state){
          case IDLE:{
            dwt_rxenable(DWT_START_RX_IMMEDIATE); // re-enable receiver and listen for broadcast
            osDelay(2); // while idle, delay between loop iterations
            break;
          }case WAIT_FOR_RESPONSE:{
            if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == RESPONSE_INIT &&
              get_src_addr(state_data.rx_buffer) == state_data.transact_id){
              
              state_data.t_rr = get_rx_timestamp();

              send_response_final(&state_data);

              if(state_data.tx_status == TX_SUCCESS){
                set_wait_for_data(&state_data); // also enables receiver
                // debug_put("Got response\r\n");
              }else{
                debug_put("Transmit failed.\r\n");
                dwt_forcetrxoff();
                // init_from_config(self_config, &config, &state_data); // just reinitialize
                set_state_tag_idle(&state_data);
                osDelay(5);
              }
              
            }
            else{
              dwt_rxenable(DWT_START_RX_IMMEDIATE); // re-enable receiver and keep listening
            }
            break;
          }case WAIT_FOR_DATA:{
            if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == RESPONSE_DATA && get_src_addr(state_data.rx_buffer) == state_data.transact_id){
              
              osTimerStop(rangingTimeoutTimerHandle); // stop the timeout timer

              AnchorData* anchor = &(state_data.anchors[state_data.anchor_ind]);

              // COPY ALL THE DATA OVER
              int ind = MAC_SIZE_EXPECTED + 1; // index for start of timestamps
              memcpy(&state_data.t_rp, state_data.rx_buffer + ind, sizeof(state_data.t_rp));
              ind += sizeof(state_data.t_rp);
              memcpy(&state_data.t_sr, state_data.rx_buffer + ind, sizeof(state_data.t_sr));
              ind += sizeof(state_data.t_sr);
              memcpy(&state_data.t_rf, state_data.rx_buffer + ind, sizeof(state_data.t_rf));
              ind += sizeof(state_data.t_rf);
              memcpy(&anchor->x, state_data.rx_buffer + ind, sizeof(anchor->x));
              ind += sizeof(anchor->x);
              memcpy(&anchor->y, state_data.rx_buffer + ind, sizeof(anchor->y));
              ind += sizeof(anchor->y);
              memcpy(&anchor->z, state_data.rx_buffer + ind, sizeof(anchor->z));
              ind += sizeof(anchor->z);
              anchor->timestamp = HAL_GetTick();

              get_tof(&state_data); // updates the tof field

              state_data.distance = DISTANCE_FROM_TOF(state_data.tof);
              
              anchor->timeout_count = 0; // only a successful ranging transaction will reset this counter
              anchor->distance = state_data.distance;
              sprintf(debug_text, "Distance to anchor %d is %.3f\r\n", anchor->id, anchor->distance);
              debug_put(debug_text);

              set_state_tag_idle(&state_data); 		// back to idle state

              next_anchor(&state_data); 				// increment anchor index

              osDelay(5);
            }
            break;
          }default:{
            set_state_tag_idle(&state_data);
            break;
          }
        }
        // dwt_rxenable(DWT_START_RX_IMMEDIATE); 	// finally, re-enable receiver and keep listening
      }else if(state_data.mode == DEVICE_MODE_ANCHOR){
        // react based on the frame type
        switch(state_data.rx_buffer[MAC_SIZE_EXPECTED]){
          case POLL:{
            
            state_data.t_rp = get_rx_timestamp(); 		// stash rx timestamp
            state_data.transact_id = get_src_addr(state_data.rx_buffer); // indicate who we're talking with
            
            // sprintf(debug_text, "Poll received from %d\r\n", state_data.transact_id);
            // debug_put(debug_text);

            TagData new_tag;
            int ind = find_tag(state_data.transact_id, state_data.tags);
            if(ind < 0){
              // didn't find it, so try to add it
              ind = add_tag(new_tag, state_data.tags);
            }
            
            if(ind >= 0){
              // we were able to add it
              enable_ranging(&state_data); // indicate ranging in progress
              state_data.tags[ind].active = 1;
              state_data.tags[ind].id = state_data.transact_id;
              state_data.tags[ind].t_rp = state_data.t_rp;

              dwt_forcetrxoff(); // seems we need to do this
              send_response_init(&state_data); 	// send a RESPONSE_INIT frame & stash t_sr timestamp
              state_data.tags[ind].t_sr = state_data.t_sr; // copy over for when we need to respond

              if(state_data.tx_status != TX_SUCCESS){
                init_from_config(self_config, &config, &state_data); // just reinitialize
              }else{
                // osTimerStart(rangingTimeoutTimerHandle, RANGING_TIMEOUT); // start the ranging timeout timer
                // osSemaphoreAcquire(timeoutSemHandle, 0);
              }
            }else{
              debug_put("Can't add tag, queue full\r\n");
            }
            
            break;
          }case SEND_FINAL:{
            state_data.transact_id = get_src_addr(state_data.rx_buffer); //
            state_data.t_rf = get_rx_timestamp(); 	// stash the timestamp

            int ind = find_tag(state_data.transact_id, state_data.tags);
            if(ind >= 0 && state_data.tags[ind].active){
              // we have a tag matching this transaction id and it's active
              state_data.t_rp = state_data.tags[ind].t_rp;
              state_data.t_sr = state_data.tags[ind].t_sr;
              state_data.tags[ind].t_rf = state_data.t_rf;

              send_response_data(&state_data); 		// send a RESPONSE_DATA frame. it packages all the timestamps from state_data
              disable_ranging(&state_data);
              state_data.tags[ind].active = 0;    // reset this flag since we're now done ranging with it
            }
            break;
          }default:{
            break;
          }
        }
        dwt_rxenable(DWT_START_RX_IMMEDIATE); // finally, make sure receiver is enabled
      }
    }// end if new frame
    else{
      // no RX Interrupt, but do the following anyway. Seems anchors don't work without this
      dwt_rxenable(DWT_START_RX_IMMEDIATE); // make sure receiver is enabled
    }

    // ---- Check if it's time for ranging ----
    if(state_data.mode == DEVICE_MODE_TAG && !state_data.ranging && state_data.num_anchors > 0 && osSemaphoreAcquire(doRangingHandle, 5) == osOK){
      enable_ranging(&state_data);

      dwt_forcetrxoff(); // in case we were in RX mode looking for a beacon frame

      // select whichever anchor we're ranging with
      state_data.transact_id = state_data.anchors[state_data.anchor_ind].id;

      // sprintf(debug_text, "ranging with %d, index %d\r\n", state_data.transact_id, state_data.anchor_ind);
      // debug_put(debug_text);

      int data_len = rtls_make_mac_header(&state_data, FRAME_TYPE_DATA); //make_mac_header(state_data.tx_buffer, state_data.transact_id, state_data.transact_id, state_data.seq_num);
      state_data.tx_buffer[data_len++] = POLL; // right after the MAC header


      if(transmit_frame(state_data.tx_buffer, data_len + 2, 1) != TX_SUCCESS){
        debug_put("Transmit failed.\r\n");
        dwt_forcetrxoff();
        // init_from_config(self_config, &config, &state_data); // just reinitialize
        set_state_tag_idle(&state_data);
        osDelay(5);
      }else{
        set_wait_for_repsonse(&state_data); // advance state machine and enable receiver
        state_data.t_sp = get_tx_timestamp();
        
        if(osTimerIsRunning(rangingTimeoutTimerHandle)){
          osTimerStop(rangingTimeoutTimerHandle);
        }
        osTimerStart(rangingTimeoutTimerHandle, RANGING_TIMEOUT); // start the ranging timeout timer
        osSemaphoreAcquire(timeoutSemHandle, 0);
      }
    }
    // or time for broadcasting
    else if(state_data.mode == DEVICE_MODE_ANCHOR && osSemaphoreAcquire(doBroadcastHandle, 0) == osOK){
      // debug_put("Broadcasting...\r\n");
			dwt_forcetrxoff();
      send_anchor_broadcast(&state_data);
      // uint16_t tempvbat = dwt_readtempvbat(1);
      dwt_rxenable(DWT_START_RX_IMMEDIATE); // re-enable receiver and keep listening
    }

    // ---- TIMEOUT ----
    if(state_data.ranging && osSemaphoreAcquire(timeoutSemHandle, 0) == osOK){ // 

      sprintf(debug_text, "timed out with %d\r\n", state_data.transact_id);
      debug_put(debug_text);
      if(check_rx_state() != 0){
        debug_put("RX RESET\r\n");
        osDelay(10);
      }
      disable_ranging(&state_data);       // reset flag
      if(state_data.mode == DEVICE_MODE_TAG){
        state_data.anchors[state_data.anchor_ind].timeout_count++;
        state_data.num_anchors -= remove_dead_anchors(state_data.anchors, state_data.num_anchors);
        set_state_tag_idle(&state_data);  // back to idle state
        next_anchor(&state_data); 				// increment anchor index
        osDelay(5);
      }
      // perhaps the other side was busy for some reason.
      // If so, delay for a short while to prevent overlap with other devices
      osDelay(5);
    }
    // we should only insert loop delay when we're not in a ranging transaction
    // osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUserIOTask */
/**
* @brief Function implementing the userIOTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUserIOTask */
void StartUserIOTask(void *argument)
{
  /* USER CODE BEGIN StartUserIOTask */
  /* Infinite loop */
  osDelay(100);
  for(;;)
  {
    if(debug_buf_offset > 0){
      HAL_UART_Transmit(&huart1, debug_buf, debug_buf_offset, 5);
      debug_buf_offset = 0;
    }
    // ---- check Interfaces for data ----
	  uart2_status = HAL_UART_Receive(&huart2, packet, HDDR_LEN, 1);
		if((packet_type = packet[0]) != 0 && uart2_status == HAL_OK){
			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_SET);
			packet_len = HDDR_LEN + packet[1] + 1; 	// get the packet length

			if(packet_len > INPUT_BUFFER_SIZE)
				packet_len = INPUT_BUFFER_SIZE; // clamp packet_len just in case

			// read the packet
			uart2_status = HAL_UART_Receive(&huart2, packet + HDDR_LEN, packet[1] + 1, INPUT_TIMEOUT);

			// if all is well
			if(uart2_status == HAL_OK && packet[packet_len - 1] == STOP_BYTE){
				packet_rcvd = IF_TYPE_UART;
			}

			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_RESET);

		}else if(osSemaphoreAcquire(usbDataReadySemHandle, 1) == osOK && (packet_type = UserRxBufferFS[0]) != 0){
      // debug_put("USB data\r\n");
			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_SET);
			packet_len 	= UserRxBufferFS[1] + HDDR_LEN + 1;

			if(packet_len > INPUT_BUFFER_SIZE)
				packet_len = INPUT_BUFFER_SIZE; // clamp packet_len just in case

			if(UserRxBufferFS[packet_len - 1] == STOP_BYTE){
				packet_rcvd = IF_TYPE_USB;
				memcpy(packet, UserRxBufferFS, packet_len);
			}

			// memset(UserRxBufferFS, 0, packet_len);
      UserRxBufferFS[0] = 0;

			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, GPIO_PIN_RESET);
    }

    // ---- if received, process the data ----
    if(packet_rcvd > 0){
      // debug_put("Processing data\r\n");
      int packet_status = process_packet(packet, UserTxBufferFS, self_config, &state_data);

      if(packet_status == PACKET_OK){
        if(packet[0] == CMD_SET_CONFIG){
          init_from_config(self_config, &config, &state_data); // re-initialize with the new parameters
          save_fields_to_eeprom(self_config);
          
          if(state_data.mode == DEVICE_MODE_TAG){
            osTimerStop(broadcastTimerHandle);
		        osTimerStart(rangingTimerHandle, state_data.ranging_period); // start the ranging timer
          }else{
            osTimerStop(rangingTimerHandle);
            osTimerStart(broadcastTimerHandle, ANCHOR_BROADCAST_PERIOD); // start the broadcast timer
          }

        }
      }else{
        debug_put("Packet error\r\n");
      }
      int send_size = HDDR_LEN + UserTxBufferFS[1] + 1;
      switch(packet_rcvd){
      case IF_TYPE_UART:{
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
        uart2_status = HAL_UART_Transmit(&huart2, UserTxBufferFS, send_size, INPUT_TIMEOUT);
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
        break;}
      case IF_TYPE_USB:{
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_SET);
        if(CDC_Transmit_FS(UserTxBufferFS, send_size) != USBD_OK){
          debug_put("Couldn't transmit over USB\r\n");
        }
        HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, GPIO_PIN_RESET);
        break;}
      default:{
        break;}
      }
      packet_rcvd = 0; // reset this flag
    }
    // debug_put("USER IO LOOP\r\n");
    osDelay(25);
  }
  /* USER CODE END StartUserIOTask */
}

/* rangingTimerCallback function */
void rangingTimerCallback(void *argument)
{
  /* USER CODE BEGIN rangingTimerCallback */
  // osSemaphoreRelease(doRangingHandle);
  long task_woken = 0;
  xSemaphoreGiveFromISR(doRangingHandle, &task_woken);
  if(task_woken){
    xTaskResumeFromISR(defaultTaskHandle);
  }
  /* USER CODE END rangingTimerCallback */
}

/* broadcastTimerCallback function */
void broadcastTimerCallback(void *argument)
{
  /* USER CODE BEGIN broadcastTimerCallback */
  // osSemaphoreRelease(doBroadcastHandle);
  xSemaphoreGiveFromISR(doBroadcastHandle, NULL);
  /* USER CODE END broadcastTimerCallback */
}

/* timeoutCallback function */
void timeoutCallback(void *argument)
{
  /* USER CODE BEGIN timeoutCallback */
  xSemaphoreGiveFromISR(timeoutSemHandle, NULL); // indicate that a timeout occurred
  // xTaskResumeFromISR(defaultTaskHandle);
  // HAL_GPIO_TogglePin(RX_LED_GPIO_Port, RX_LED_Pin);
  /* USER CODE END timeoutCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
    
/**
 * callback to run when the DWM_IRQn pin goes HIGH, indicating frame reception
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == DW_IRQn_Pin){
    // osSemaphoreRelease(dw_irq_semHandle);
    long task_woken = 0;
    xSemaphoreGiveFromISR(dw_irq_semHandle, &task_woken);
    if(task_woken){
      xTaskResumeFromISR(defaultTaskHandle);
    }
  }

}

void debug(char* text){
  HAL_UART_Transmit(&huart1, text, strlen(text), 10);
}

bool debug_put(char* text){
  sprintf(scratch_buf, "[%10d] : %s", HAL_GetTick(), text);
  int len = strlen(scratch_buf);
  if((debug_buf_offset + len) > DEBUG_BUF_SIZE){
    len = DEBUG_BUF_SIZE - debug_buf_offset; // clamp length so we don't overrun debug_buf
  }
  memcpy(debug_buf + debug_buf_offset, scratch_buf, len);
  debug_buf_offset += len;
  
  return len > 0;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
