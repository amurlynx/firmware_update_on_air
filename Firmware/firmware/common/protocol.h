/* Header file that describes the protocol of communication between
 * the host and the device.
 */

#ifndef PROTOCOL_RF_
#define PROTOCOL_RF_

typedef enum {
  /* Default, initialization value. Not to be sent between host and device. */
  CMD_NO_CMD = 0x00,
  /* Host sends exit signal to device, connection to be terminated after device
   * has sent ACK. */
  CMD_EXIT,
  /* Host initiates a connection, device should send ACK, and then both will be
   * in the CONNECTED state. */
  CMD_INIT,
  /* Host initiates a firmware update, device should send ACK if ready. */
  CMD_UPDATE_START,
  /* Host requests a portion of the flash memory. 
   * 2B start address, 1B number of requested bytes.
   * ACK respons should contain the data as payload */
  CMD_READ,
  /* Host sends a HEX line, device should send ACK after line has been 
   * written to FLASH, and is ready for another. */
  CMD_WRITE,
  /* Host sends confirmation that update has been completed and 
   * grants permission to enable the new firmware. */
  CMD_UPDATE_COMPLETE,
  /* Acknowledge, might contain data payload depending on corresponding request */
  CMD_ACK,
  /* Negative acknowledge */
  CMD_NACK,
  /* Check for connection integrity. Should be answered with CMD_PONG. */
  CMD_PING,
  /* Answer for CMD_PING. */
  CMD_PONG
} command_t;

typedef enum
{
  /* Generic nack error message defined by nature of request */
  ERROR_GENERIC = 0,
  /* The LU1p has lost contact with the LE1 */
  ERROR_LOST_CONNECTION,
  /* LE1 reports wrong checksum on received line. Line should be sent again */
  ERROR_CHECKSUM_FAIL,
  /* Hex line passed checksum, but contains illegal address. 
   * Update should be cancelled */ 
  ERROR_ILLEGAL_ADDRESS,
  /* Nack error message when CMD_UPDATE_INIT provides a size which is 
   * to big for the flash memory */
  ERROR_ILLEGAL_SIZE
} error_codes_t;



/* RF channels searched for devices */
#define CHANNELS {2, 6, 81}
#define CHANNELS_SIZE 3
/* 5 byte prearranged pipe address */
#define PIPE_ADDRESS {0xBA,0xDA,0x55,0x13,0x37}


#endif //PROTOCOL_RF_
