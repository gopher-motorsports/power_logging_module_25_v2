/*
 * plm_sd.c
 *
 *  Created on: Jan 29, 2023
 *      Author: jonathan
 */

#include "plm.h"
#include "plm_sd.h"
#include "main.h"
#include "fatfs.h"
#include "plm_error.h"
#include <stdio.h>

#define ZERO_YEAR 1970

extern RTC_HandleTypeDef hrtc;

PLM_RES plm_sd_init(void) {
#ifdef PLM_DEV_MODE
    printf("PLM (%lu): initializing SD card\n", HAL_GetTick());
#endif

    uint8_t sd_detected = HAL_GPIO_ReadPin(SD_SW_CD_GPIO_Port, SD_SW_CD_Pin) == GPIO_PIN_RESET;
    if (!sd_detected) return PLM_ERR_SD_INIT;

    // checks that the card is inserted & initializes SD interface
//    DSTATUS status = SD_Driver.disk_initialize(0);
//    if (status == STA_NOINIT) return PLM_ERR_SD_INIT;

    // empty SD driver message queue
    // this is necessary to get a clean state after disconnecting USB
    // SD_ResetMsgQueue(); I think this might be important but it is not defined. FIX THIS____________________

    // register file system
    FRESULT res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK) return PLM_ERR_SD_INIT;

    // generate filename
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
    char filename[] = "PLM_YYYY-MM-DD-hh-mm-ss.gdat";
    sprintf(filename, "PLM_%04u-%02u-%02u-%02u-%02u-%02u.gdat", date.Year + ZERO_YEAR, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);

    // open data file
    // file is created if it doesn't exist
    res = f_open(&SDFile, filename, FA_WRITE | FA_CREATE_NEW);
    while (res == FR_EXIST) {
    	// increment timestamp until a valid filename is found
    	time.Seconds += 1;
    	sprintf(filename, "PLM_%04u-%02u-%02u-%02u-%02u-%02u.gdat", date.Year + ZERO_YEAR, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
    	res = f_open(&SDFile, filename, FA_WRITE | FA_CREATE_NEW);
    }
    if (res != FR_OK) return PLM_ERR_SD_INIT;

    // write the filename as metadata
    res = f_printf(&SDFile, "/%s:\n", filename);
    if (res <= 0) return PLM_ERR_SD_INIT;

    return PLM_OK;
}

void plm_sd_deinit(void) {
#ifdef PLM_DEV_MODE
    printf("PLM (%lu): de-initializing SD card\n", HAL_GetTick());
#endif

    // close file and unregister file system
    f_close(&SDFile);
    //f_mount(NULL, SDPath, 0);
}

PLM_RES plm_sd_write(uint8_t* buffer, uint16_t size) {
    unsigned int bytes_written = 0;
    FRESULT res = f_write(&SDFile, buffer, size, &bytes_written);
    if (res != FR_OK || bytes_written != size) return PLM_ERR_SD_WRITE;

    // flush cached data
    res = f_sync(&SDFile);
    if (res != FR_OK) return PLM_ERR_SD_WRITE;

    return PLM_OK;
}
