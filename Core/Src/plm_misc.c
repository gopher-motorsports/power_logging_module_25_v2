/*
 * plm_misc.c
 *
 *  Created on: May 14, 2023
 *      Author: jonathan
 */

 #include "plm_misc.h"
 #include "plm.h"
 #include "plm_data.h"
 #include "GopherCAN.h"
 #include "plm_power.h"
 #include "main.h"
 #include "GopherCAN_network.h"
 #include "GopherCAN_names.h"
 
 extern RTC_HandleTypeDef hrtc;
 
 extern PLM_DBL_BUFFER SD_DB;
 
 extern U32 hcan1_rx_callbacks;
 extern U32 hcan2_rx_callbacks;
 
 typedef struct AdcValues {
     uint16_t Raw[2]; /* Raw values from ADC */
     double IntSensTmp; /* Temperature */
 }adcval_t;
 adcval_t Adc;
 typedef struct Flags {
     uint8_t ADCCMPLT;
 }flag_t;
 flag_t Flg = {0, };
 
 #define TMPSENSOR_USE_INTREF 1 /* 1 - Use Internal Reference Voltage; 0 - Not use; */
 #define TMPSENSOR_AVGSLOPE 2.5 /* mV/°C */
 #define TMPSENSOR_V25  0.76 /* V (at 25 °C)  */
 #define TMPSENSOR_ADCMAX 4095.0 /* 12-bit ADC maximum value (12^2)-1)  */
 #define TMPSENSOR_ADCREFVOL  3.3 /* Typical reference voltage, V  */
 #define TMPSENSOR_ADCVREFINT  1.21 /* Internal reference voltage, V  */
 
 
 /* PFP BEGIN */
 double TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref);
 /* PFP END */
 
 
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
     if(hadc->Instance == ADC1) /* Check if the interrupt comes from ACD1 */
     {
     /* Set flag to true */
     Flg.ADCCMPLT = 255;
     }
 }
 
 double TMPSENSOR_getTemperature(uint16_t adc_sensor, uint16_t adc_intref){
 #if(TMPSENSOR_USE_INTREF)
     double intref_vol = (TMPSENSOR_ADCMAX*TMPSENSOR_ADCVREFINT)/adc_intref;
 #else
     double intref_vol = TMPSENSOR_ADCREFVOL;
 #endif
     double sensor_vol = adc_sensor * intref_vol/TMPSENSOR_ADCMAX;
     double sensor_tmp = (sensor_vol - TMPSENSOR_V25) *1000.0/TMPSENSOR_AVGSLOPE + 25.0;
     return sensor_tmp;
 }
 
 void plm_update_logging_metrics(void) {
     static uint32_t last_update = 0;
     uint32_t tick = HAL_GetTick();
     if (Flg.ADCCMPLT) /* Conversion completed, do calculations */ {
             /* Temperature Sensor ADC-value, Reference Voltage ADC-value (if use) */
             Adc.IntSensTmp = TMPSENSOR_getTemperature(Adc.Raw[1], Adc.Raw[0]);
     }
     if (tick - last_update >= PLM_DELAY_LOGGING_METRICS) {
 
         update_and_queue_param_float(&PLM_Micro_Temp, Adc.IntSensTmp);
 
 
         packetsDropped_ul.info.last_rx = tick;
         packetsLogged_ul.info.last_rx = tick;
 
         // SD and Xbee buffer fill %
         storageBufferFill_percent.data = (float) SD_DB.buffers[SD_DB.write_index]->fill / SD_DB.buffers[SD_DB.write_index]->size * 100.0f;
         storageBufferFill_percent.info.last_rx = tick;
 
         // CAN bus load
         // does not include messages filtered out by hardware
         // does not include messages sent by the PLM
         // assumes all CAN frames are 125 bits (11-bit ID, 8 data bytes, other frame stuff)
         float hcan1_rx_bps = (hcan1_rx_callbacks * 125) / ((float)(tick - last_update) / 1000.0f);
         float hcan2_rx_bps = (hcan2_rx_callbacks * 125) / ((float)(tick - last_update) / 1000.0f);
 
         // bus load = sampled bps / 1Mbps
         can0Utilization_percent.data = hcan1_rx_bps / 1000000.0f * 100.0f;
         can1Utilization_percent.data = hcan2_rx_bps / 1000000.0f * 100.0f;
 
         can0Utilization_percent.info.last_rx = tick;
         can1Utilization_percent.info.last_rx = tick;
 
         for (size_t i = 0; i < NUM_OF_CHANNELS; i++) {
                 PLM_POWER_CHANNEL* channel = POWER_CHANNELS[i];
                 update_and_queue_param_float(channel->parameter, channel->parameter->data);
                 if(channel->overcurrent_count != 0) {
                     update_and_queue_param_float(&Overcurrent_Event, 1);
                     update_and_queue_param_float(channel->overcurrentparam, 1);
                     update_and_queue_param_float(channel->overcurrentcountparam, channel->overcurrent_count);
                 }
         }
 
         hcan1_rx_callbacks = 0;
         hcan2_rx_callbacks = 0;
 
         last_update = tick;
     }
 }
 
 void plm_sync_rtc(void) {
     static uint32_t last_sync = 0;
     uint32_t tick = HAL_GetTick();
 
     RTC_TimeTypeDef time;
     RTC_DateTypeDef date;
 
     if (tick - last_sync >= PLM_DELAY_SYNC_RTC) {
         date.Year = gpsYearUTC_ul.data - 1970;
         date.Month = gpsMonthUTC_ul.data;
         date.Date = gpsDayUTC_ul.data;
         time.Hours = gpsHoursUTC_ul.data;
         time.Minutes = gpsMinutesUTC_ul.data;
         time.Seconds = gpsSecondsUTC_ul.data + ((HAL_GetTick() / 1000) - gpsSecondsUTC_ul.info.last_rx);
 
         HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
         HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
         last_sync = tick;
     }
 }
 
 