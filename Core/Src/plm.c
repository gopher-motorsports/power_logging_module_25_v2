/*
 * plm.c
 *
 *  Created on: Feb 6, 2025
 *      Author: jonathan
 */

 #include <stdint.h>
 #include <stdio.h>
 #include "plm.h"
 #include "cmsis_os.h"
 #include "main.h"
 #include "GopherCAN.h"
 #include "gopher_sense.h"
 //#include "usb_device.h"
 #include "fatfs.h"
 #include "plm_sd.h"
 #include "plm_sim.h"
 #include "plm_data.h"
 #include "plm_power.h"
 #include "plm_misc.h"
 #include "plm_error.h"
 #include "GPIO_interface.h"
 
 // we might need to turn this up for launch control
 #define CAN_MESSAGE_FORWARD_INTERVAL_ms 50
 //#define NOT_ENDURANCE
 
 extern CAN_HandleTypeDef hcan1;
 extern CAN_HandleTypeDef hcan2;
 
 //extern USBD_HandleTypeDef hUsbDeviceFS;
 
 extern ADC_HandleTypeDef hadc1;
 extern ADC_HandleTypeDef hadc2;
 extern ADC_HandleTypeDef hadc3;
 
 extern PLM_DBL_BUFFER SD_DB;
 extern PLM_DBL_BUFFER RADIO_DB;
 
 extern I2C_HandleTypeDef hi2c2;
 extern SD_HandleTypeDef hsd;
 
 U32 hcan1_rx_callbacks = 0;
 U32 hcan2_rx_callbacks = 0;
 uint8_t usb_state = 0;
 
 void plm_init(void) {
     plm_err_reset();
     S8 err = 0;
 
 #ifdef PLM_DEV_MODE
     // put CAN peripherals into loopback to receive simulated data
     hcan1.Init.Mode = CAN_MODE_LOOPBACK;
     hcan2.Init.Mode = CAN_MODE_LOOPBACK;
     err |= HAL_CAN_Init(&hcan1);
     err |= HAL_CAN_Init(&hcan2);
     if (err) {
         // PLM shouldn't run without CAN
         plm_err_set(PLM_ERR_INIT);
         HAL_Delay(PLM_DELAY_RESTART);
         NVIC_SystemReset();
     }
 #endif
 
     // GopherCAN
     err |= init_can(&hcan1, GCAN0); //24 GopherCan
     err |= init_can(&hcan2, GCAN1); //24 GopherCan
 
 
     if (err) {
         plm_err_set(PLM_ERR_INIT);
         HAL_Delay(PLM_DELAY_RESTART);
         NVIC_SystemReset();
     }
 
     // GopherSense
     gsense_init(&hcan1, &hadc1, &hadc2, &hadc3, 0, 0);
 
     //GPIO Expander
         GPIO_init();
 
     // enable all power channel switches
     for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
         PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
         if (i <= 6) {
             HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
         }
         else if (i > 6) {
             GPIO_Extension_On(channel->external_GPIO_on);
         }
     }
 
     // we dont want to send parameters
     set_all_param_sending(FALSE);
 
 
 #ifdef PLM_DEV_MODE
     printf("PLM successfully initialized\n");
 #endif
 }
 
 void plm_heartbeat(void) {
     uint32_t tick = HAL_GetTick();
 
     static uint32_t last_blink = 0;
     if (tick - last_blink >= PLM_DELAY_HEARTBEAT_BLINK) {
 
 #ifdef PLM_DEV_MODE
         printf("PLM (%lu): ⚡\n", tick);
 #endif
         GPIO_Extension_toggle(0);
         last_blink = tick;
     }
 
     plm_update_logging_metrics();
     plm_sync_rtc();
     plm_err_blink();
 
     osDelay(PLM_TASK_DELAY_HEARTBEAT);
 }
 
 void plm_service_can(void) {
     // send out messages that need to be forwarded
     static U32 last_message_send = 0;
     if (HAL_GetTick() - last_message_send >= CAN_MESSAGE_FORWARD_INTERVAL_ms)
     {
         last_message_send = HAL_GetTick();
     }
 
     service_can_tx(&hcan1);
     service_can_tx(&hcan2);
     service_can_rx_buffer();
 
     osDelay(PLM_TASK_DELAY_CAN);
 }
 
 void GCAN_RxMsgPendingCallback(CAN_HandleTypeDef* hcan, U32 rx_mailbox) {
 //    if (hcan->Instance == CAN1) hcan1_rx_callbacks++;
 //    else if (hcan->Instance == CAN2) hcan2_rx_callbacks++;
 
     service_can_rx_hardware(hcan, rx_mailbox);
 }
 
 
 
 
 //need to update this for tm_collect_data
 void plm_collect_data(void) {
     static uint32_t sd_last_log[NUM_OF_PARAMETERS] = {0};
     static uint32_t radio_last_tx[NUM_OF_PARAMETERS] = {0};
     uint8_t voltage_ok = plmVbatVoltage_V.data >= MIN_VBAT_VOLTAGE_V && plm5VVoltage_V.data >= MIN_5V_VOLTAGE_V;
     uint8_t usb_connected = HAL_GPIO_ReadPin(HS_VBUS_SNS_GPIO_Port, HS_VBUS_SNS_Pin);
 
 #ifdef PLM_DEV_MODE
     voltage_ok = 1;
 #endif
 
     // must have usb disconnected
     if (usb_connected) {
         osDelay(PLM_TASK_DELAY_DATA);
         return;
     }
 
     // swap buffers after transfers are complete
     // critical section entry/exit is fast and fine for a quick swap
     if (SD_DB.tx_cplt) {
         taskENTER_CRITICAL();
         SD_DB.write_index = !SD_DB.write_index;
         SD_DB.buffers[SD_DB.write_index]->fill = 0;
         SD_DB.tx_cplt = 0;
         taskEXIT_CRITICAL();
     }
 
     for (uint16_t i = 1; i < NUM_OF_PARAMETERS; i++) {
         CAN_INFO_STRUCT* param = (CAN_INFO_STRUCT*)(PARAMETERS[i]);
         uint32_t tick = HAL_GetTick();
 
         if (param->last_rx > sd_last_log[i]) {
             // parameter has been updated
 #ifdef PLM_JANK
             // dont log the PLM current channels too fast
             if (param->GROUP_ID <= 0x5 && param->last_rx - sd_last_log[i] < 9)
             {
                 continue;
             }
 #endif
             PLM_RES res = plm_data_record_param(SD_DB.buffers[SD_DB.write_index], param);
             if (res == PLM_OK) {
                 packetsLogged_ul.data += 1;
             }
             else {
                 packetsDropped_ul.data += 1;
                 plm_err_set(res);
             }
             sd_last_log[i] = tick;
         }
 
         if (param->last_rx > radio_last_tx[i] && (tick - radio_last_tx[i]) > TM_RADIO_TX_DELAY) {
             // parameter has been updated and hasn't been sent in a while
             // create packet and add to radio buffer
             PLM_RES res = plm_data_record_param(RADIO_DB.buffers[RADIO_DB.write_index], param);
             if (res == PLM_OK) {
                 radio_last_tx[i] = tick;
             } else {
                 tm_RadioPacketsDropped_ul.data += 1;
             }
         }
     }
 
     // tm_SDBufferFill_percent.data = (float) SD_DB.buffers[SD_DB.write_index]->fill / SD_DB.buffers[SD_DB.write_index]->size * 100.0f;
     tm_RadioBufferFill_percent.data = (float) RADIO_DB.buffers[RADIO_DB.write_index]->fill / RADIO_DB.buffers[RADIO_DB.write_index]->size * 100.0f;
 
 
     osDelay(PLM_TASK_DELAY_DATA);
 }
 
 void plm_store_data(void) {
     // if SD is ready for FatFs interaction
     static uint8_t fs_ready = 0;
 
     // check if device is connected and ready to interact via USB
     uint8_t usb_connected = HAL_GPIO_ReadPin(HS_VBUS_SNS_GPIO_Port, HS_VBUS_SNS_Pin);
 
     // prevent USB access and FatFs interaction at the same time
     // USB callbacks are in USB_DEVICE/App/usbd_storage_if.c
     // uses the FatFs driver in FATFS/Target/sd_diskio.c
     if (usb_connected && (usb_state == 0)) {
 #ifdef PLM_DEV_MODE
         printf("PLM (%lu): USB connected\n", HAL_GetTick());
 #endif
         usb_state = 1;
         plm_sd_deinit();
         fs_ready = 0;
         uint64_t i = 0;
         //__HAL_SD_DISABLE();
         HAL_SD_DeInit(&hsd);
         //MX_FATFS_DeInit();
         osDelay(50);
         HAL_GPIO_WritePin(MEDIA_nRST_GPIO_Port, MEDIA_nRST_Pin, 1);
     }
 
     if (!usb_connected) {
         if (usb_state == 1) {
             HAL_SD_Init(&hsd);
         }
         usb_state = 0;
         if (!fs_ready) {
             // init FatFs and open the current data file
              PLM_RES res = plm_sd_init();
             if (res != PLM_OK) {
                 plm_sd_deinit();
                 plm_err_set(res);
             }
             else fs_ready = 1;
         }
 
         if (fs_ready) {
             // write data
             if (!SD_DB.tx_cplt) {
                 PLM_BUFFER* buffer = SD_DB.buffers[!SD_DB.write_index];
                 if (buffer->fill > 0) {
                     PLM_RES res = plm_sd_write(buffer->bytes, buffer->fill);
                     if (res != PLM_OK) {
                         // write failed
                         fs_ready = 0;
                         plm_sd_deinit();
                         plm_err_set(res);
                     } else {
                         // successful write
                         SD_DB.tx_cplt = 1;
                         GPIO_Extension_toggle(1);
                     }
                 } else SD_DB.tx_cplt = 1;
             }
         }
     }
 
     osDelay(PLM_TASK_DELAY_SD);
 }
 
 void plm_transmit_data(void) {
     static bool tx_in_progress = false;
 
     if (!RADIO_DB.tx_cplt && !tx_in_progress) {
         // waiting for a transfer to radio
         PLM_BUFFER* buffer = RADIO_DB.buffers[!RADIO_DB.write_index];
         if (buffer->fill > 0) {
             HAL_UART_Transmit_DMA(&huart1, buffer->bytes, buffer->fill);
             tx_in_progress = true;
         } else {
             // nothing to transfer
             RADIO_DB.tx_cplt = 1;
         }
     }
 
     // swap buffers after transfer is complete
     // critical section entry/exit is fast and fine for a quick swap
     if (RADIO_DB.tx_cplt) {
         taskENTER_CRITICAL();
         tm_RadioBytesTransferred_bytes.data += RADIO_DB.buffers[!RADIO_DB.write_index]->fill;
         RADIO_DB.buffers[!RADIO_DB.write_index]->fill = 0;
         RADIO_DB.write_index = !RADIO_DB.write_index;
         RADIO_DB.tx_cplt = 0;
         tx_in_progress = false;
         taskEXIT_CRITICAL();
     }
     osDelay(PLM_TASK_DELAY_XB);
 }
 
 void plm_simulate_data(void) {
 #ifndef PLM_SIMULATE_DATA
     osThreadTerminate(osThreadGetId());
 #endif
 
     PLM_RES res = plm_sim_generate_data();
     if (res != PLM_OK) plm_err_set(PLM_ERR_SIM);
 
     osDelay(PLM_TASK_DELAY_SIM);
 
 }
 
 void plm_monitor_current(void) {
 #ifdef PLM_DEV_MODE
     osThreadTerminate(osThreadGetId());
 #endif
 
     for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
         PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
         plm_power_update_channel(channel);
 
         //fix for spiking ADC values
         if (channel->ampsec_sum > 200) {
             channel->ampsec_sum = 0;
         }
 
 
         if (channel->ampsec_sum > channel->ampsec_max && channel->enabled) {
 #ifdef NOT_ENDURANCE
             // channel has reached Amp*sec threshold, open switch
             // Will not do this during endurance
             if (i >= 7){
                 GPIO_Extension_Off((channel->external_GPIO_off));
             } else {
                 HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_RESET);
             }
             channel->enabled = 0;
 #endif
             channel->trip_time = HAL_GetTick();
             channel->overcurrent_count++;
             GPIO_extension_overcurrent_LED(1);
         } else if (!channel->enabled) {
             // check if it's time to re-enable this channel
             uint32_t ms_since_trip = HAL_GetTick() - channel->trip_time;
             if (ms_since_trip >= channel->reset_delay_ms && (channel->overcurrent_count < channel->max_overcurrent_count)) {
                 channel->ampsec_sum = 0;
                 if(i >= 7){
                     GPIO_Extension_On((channel->external_GPIO_on));
                 } else {
                     HAL_GPIO_WritePin(channel->enable_switch_port, channel->enable_switch_pin, GPIO_PIN_SET);
                 }
                 channel->enabled = 1;
                 GPIO_extension_overcurrent_LED(0);
             } else if (channel->overcurrent_count > channel->max_overcurrent_count) {
 #ifdef PLM_DEV_MODE
                 printf("MAX OVERCURRENT COUNT REACHED ON CHANNEL  " + channel + " THIS CHANNEL IS NOW PERMENANTLY DISABLED!!!")
 #endif
                 GPIO_extension_overcurrent_LED(1);
 
             }
         }
     }
 
     osDelay(PLM_TASK_DELAY_POWER);
 }
 
 void tm_heartbeat() {
     uint32_t tick = HAL_GetTick();
     static uint32_t last_can_calc = 0;
 
     HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
 
     // print time stamp
     RTC_TimeTypeDef time;
     RTC_DateTypeDef date;
     HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
     HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
     printf("20%u-%02u-%02u-%02u-%02u-%02u\n", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
 
     // update logging metrics
     tm_SDBufferFill_percent.info.last_rx = tick;
     tm_RadioBufferFill_percent.info.last_rx = tick;
     tm_SDBytesTransferred_bytes.info.last_rx = tick;
     tm_RadioBytesTransferred_bytes.info.last_rx = tick;
     tm_SDPacketsDropped_ul.info.last_rx = tick;
     tm_RadioPacketsDropped_ul.info.last_rx = tick;
 
     // average coin cell voltage samples
     uint32_t adc_total = 0;
     for (size_t i = 0; i < ADC_VBAT_BUF_SIZE; i++) {
         adc_total += ADC_VBAT_BUF[i];
     }
     float adc_avg = (float) adc_total / ADC_VBAT_BUF_SIZE;
     tm_CoinBattery_V.data = adc_avg / 4096.0f * 3.3f;
     tm_CoinBattery_V.info.last_rx = tick;
 
     // estimate CAN utilization
     // assumes 1Mbps bus
     // assumes each frame is 44 + 8N (data bytes) + 3 (inter-frame) = 111 bits
     // assumes every frame has 8 bytes of data (worst case)
     // ignores bit stuffing
     float sec_since_last_update = (HAL_GetTick() - last_can_calc) / 1000.0f;
     float hcan1_util = hcan1_rx_count * 111.0f / sec_since_last_update / 1e6;
     float hcan2_util = hcan2_rx_count * 111.0f / sec_since_last_update / 1e6;
     tm_CAN1Util_percent.data = hcan1_util * 100.0f;
     tm_CAN2Util_percent.data = hcan2_util * 100.0f;
     tm_CAN1Util_percent.info.last_rx = tick;
     tm_CAN2Util_percent.info.last_rx = tick;
     last_can_calc = HAL_GetTick();
 
     osDelay(TM_DELAY_HEARTBEAT);
 }