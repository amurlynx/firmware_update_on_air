#ifndef MEM_CONFIG_H__
#define MEM_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf24le1.h"
#include "hal_nrf.h"
#include "hal_flash.h"
#include "hal_clk.h"																				 
#include "protocol.h"

/******************************************************************************/

// Typedefs:

// Device states
typedef enum {
  LISTENING  = 0x00,
  CONNECTED,
  RECEIVING_FIRMWARE,
  ERROR
} state_t;

// Function pointer to firmware
typedef void (*firmware_start)(void);


// Function prototypes:

// Send function for RF communication
void send(command_t command);

// "interrupt" routine to handle RF "interrupts"
void nrf_irq();

// Sets up RF
void configureRF();
// Resets RF
void resetRF();

// Reply to init
void sendInitAck(state_t *state);
// Start receiving new firmware
void startFirmwareUpdate(state_t *state, uint16_t *bytes_total, 
                         uint16_t *bytes_received, uint8_t *firmware_number);
// Write hex data to flash memory
void writeHexRecord(state_t *state, uint16_t *bytes_received);
// Send hex data to host.
void readHexRecord();

/******************************************************************************/
/* These parameters must be decided based on compilation parameters. */

// The number of pages the bootloader occupies
#define BOOTLOADER_PAGES 			  6 

/******************************************************************************/
/* Memory configurations */
#define PMW                     0x10

/* CODE FLASH */
#define FLASH_PAGE_SIZE 			  512U
#define FLASH_TOTAL_PAGES 			32
#define FLASH_BL_PAGES          BOOTLOADER_PAGES
#define FLASH_FW_PAGES          FLASH_TOTAL_PAGES - FLASH_BL_PAGES
#define FLASH_FW_MAX_SIZE       FLASH_FW_PAGES * FLASH_PAGE_SIZE

/* NON VOLATILE DATA FLASH */
#define FW_NV_DATA_PAGE         35
#define FW_NV_DATA_ADDR         0xFE00
#define FW_INSTALLED            FW_NV_DATA_ADDR
#define FW_RESET_VECTOR         FW_NV_DATA_ADDR+1
#define FW_RESET_OPCODE         FW_NV_DATA_ADDR+1
#define FW_RESET_ADDR_H         FW_NV_DATA_ADDR+2
#define FW_RESET_ADDR_L         FW_NV_DATA_ADDR+3
#define FW_NUMBER               FW_NV_DATA_ADDR+4

/******************************************************************************/
/* Communication configurations. */
#define PAYLOAD_SIZE            32

/******************************************************************************/
/* Message protocol specifics */

/* All */
#define MSG_CMD                 rcvd_buf[0]
#define MSG_PAYLOAD(x)          rcvd_buf[1+(x)]

/* FIRMWARE_UPDATE_START */
#define MSG_ST_BYTES_H          MSG_PAYLOAD(0)
#define MSG_ST_BYTES_L          MSG_PAYLOAD(1)
#define MSG_ST_BYTES            ((uint16_t)MSG_ST_BYTES_H<<8) | MSG_ST_BYTES_L
#define MSG_ST_RESET_OPCODE     MSG_PAYLOAD(2)
#define MSG_ST_RESET_ADDR_H     MSG_PAYLOAD(3)
#define MSG_ST_RESET_ADDR_L     MSG_PAYLOAD(4)
#define MSG_ST_NUMBER           MSG_PAYLOAD(5)
#define UPDATE_START_LENGTH     7

/* FIRMWARE_UPDATE_WRITE */
#define MSG_WR_BYTE_COUNT       MSG_PAYLOAD(0)
#define MSG_WR_ADDR_H           MSG_PAYLOAD(1)
#define MSG_WR_ADDR_L           MSG_PAYLOAD(2)
#define MSG_WR_ADDR             ((uint16_t)MSG_WR_ADDR_H<<8) | MSG_WR_ADDR_L 
#define MSG_WR_REC_TYPE         MSG_PAYLOAD(3)
#define MSG_WR_DATA(x)          MSG_PAYLOAD(4+(x))

/* READ */
#define MSG_RE_BYTE_COUNT       MSG_PAYLOAD(0)
#define MSG_RE_ADDR_H           MSG_PAYLOAD(1) 
#define MSG_RE_ADDR_L           MSG_PAYLOAD(2) 
#define MSG_RE_ADDR             ((uint16_t)MSG_RE_ADDR_H<<8) | MSG_RE_ADDR_L 

/******************************************************************************/
/* HEX format */
#define HEX_BYTES               5

/******************************************************************************/
/* Magic numbers */
#define CHANNEL_TIMEOUT         800
#define BOOTLOADER_TIMEOUT      9 //=0s, 1500=4s
#define CONNECTION_TIMEOUT      60000
 

#endif /* MEM_CONFIG_H__ */
