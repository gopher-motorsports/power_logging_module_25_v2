/*
 * plm_data.h
 *
 *  Created on: Mar 20, 2023
 *      Author: jonathan
 */

#ifndef INC_PLM_DATA_H_
#define INC_PLM_DATA_H_

#include "GopherCAN.h"
#include "plm_error.h"

// packet control bytes
#define START_BYTE 0x7e // start of a packet
#define ESCAPE_BYTE 0x7d // next byte is escaped
#define ESCAPE_XOR 0x20

#define PLM_SD_BUFFER_SIZE 5000
#define PLM_XB_BUFFER_SIZE 0

typedef struct {
    uint8_t* bytes;
    size_t size;
    size_t fill;
} PLM_BUFFER;

typedef struct {
    PLM_BUFFER* buffers[2];
    uint8_t write_index;
    uint8_t tx_cplt;
} PLM_DBL_BUFFER;

PLM_RES plm_data_record_param(PLM_BUFFER* buffer, CAN_INFO_STRUCT* param);

#endif /* INC_PLM_DATA_H_ */
