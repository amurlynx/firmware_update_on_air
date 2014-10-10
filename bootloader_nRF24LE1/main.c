/******************************************************************************
 * Boot loader for the Firmware Update over-the-air Application Note.
 *
 * Authors: Vegar Kåsli and Ole Magnus Ruud
 *
 * abbreviations/acronyms used in this document:
 * BL = Boot Loader
 * FW = Firmware
 * MSG = Message
 * CMD = Command
 * NV = Non-volatile
 * 
 ******************************************************************************/

//lint -e717
//lint -e534
//lint -e714
//lint -e783

#include "main.h"

/******************************************************************************/

// RF Communication 
volatile uint8_t xdata default_channels[CHANNELS_SIZE] = CHANNELS;
volatile uint8_t xdata reset_channel = 0x02;
volatile uint8_t xdata default_pipe_address[5] = PIPE_ADDRESS;
volatile uint8_t xdata reset_pipe_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
volatile uint8_t xdata rcvd_buf[PAYLOAD_SIZE];
volatile uint8_t xdata send_buf[PAYLOAD_SIZE];
volatile bool xdata packet_received;
volatile bool xdata send_success;

// Hardware id number - hard coded into boot loader. 
static const uint8_t model_number = 0x10;

// Temporary buffer in idata memory.
uint8_t idata temp_data[16];

/******************************************************************************/


/* Ma-ma-ma-main function! */
void main()
{
  state_t state = LISTENING;
  command_t cmd = CMD_NO_CMD;
	firmware_start firmware;

  uint16_t channel_timer = 0, bootloader_timer = 0, connection_timer = 0;
  uint8_t ch_i = 0, firmware_number = 0;
  bool running;
	
  uint16_t bytes_received = 0;
  uint16_t bytes_total = 0;

  uint8_t ea_default, rf_default;

  // Disable RF interrupt
  rf_default = RF;
  RF = 0;
  // Disable global interrupt
  ea_default = EA;
  EA = 0;
  
  // Set up parameters for RF communication.
  configureRF();

  #ifdef DEBUG_LED_
  P0DIR = 0;
  P0 = 0x55;
  #endif 

  running = true;
  // Boot loader loop.
  // Will terminate after a couple of seconds if firmware has been successfully
  // installed.
  while (running) {
		
    // Polls the RF-interrupt bit every iteration. 
    if (RFF) {
      RFF = 0;
      nrf_irq();

      if (packet_received) {
        packet_received = false;
        connection_timer = 0;
        cmd = MSG_CMD;
     
        switch (cmd) {
          // Host initiates contact with the device.
          case CMD_INIT:
            // Send ACK to host, go to CONNECTED state if successful.
            sendInitAck(&state);
            // Reset timers 
            channel_timer = bootloader_timer = 0;
            break;

          // Host starts a firmware update.
          case CMD_UPDATE_START:
            if (state == CONNECTED) {
              // Initiate firmware updates, go to RECEIVING_FIRMWARE state
              // if successful.
              startFirmwareUpdate(&state, &bytes_total, &bytes_received, 
                                     &firmware_number);
            }

            #ifdef DEBUG_LED_
            P0 = state;
            #endif 
            break;

          // Write message containing one hex record.
          case CMD_WRITE:
            if (state == RECEIVING_FIRMWARE) {
              writeHexRecord(&state, &bytes_received); 
            }

            #ifdef DEBUG_LED_
            P0 = 0x40;
            #endif
            break;

          // Firmware update has been completed.
          case CMD_UPDATE_COMPLETE:
            CE_LOW();
            // Check that every byte is received.
            if (bytes_received == bytes_total) {
              // Mark firmware as successfully installed. 
              hal_flash_byte_write(FW_INSTALLED, 0x01);
              hal_flash_byte_write(FW_NUMBER, firmware_number); 
              state = CONNECTED;
              send(CMD_ACK);
            } else {
              send(CMD_NACK);
            }

            if (!send_success) {
              state = ERROR;
            }

            #ifdef DEBUG_LED_
            P0 = 0x10;
            #endif
            break;

          // Host request data from flash at specified address.
          case CMD_READ:
            readHexRecord();

            #ifdef DEBUG_LED_
            P0 = 0x20;
            #endif
            break;

          // Host sends ping to check connections with device.
          case CMD_PING:
            if (state != LISTENING) {
              send(CMD_PONG);
            }

            #ifdef DEBUG_LED_
            P0 = 0x80;
            #endif
            break;

          // Host sends disconnect
          case CMD_EXIT:
            state = LISTENING;
            break;

          // These commands should no be received.
          case CMD_NO_CMD:
          default:
            state = ERROR;
            break;
        }
        // Clear command
        cmd = CMD_NO_CMD;
      }

    // RF interrupt bit not set
    } else if (state == LISTENING) {
      // Will listen to one channel for 'a while' before changing.
      channel_timer++;
      if (channel_timer > CHANNEL_TIMEOUT) {
        channel_timer = 0;
        // Go to next channel
        ch_i = (ch_i+1)%3;
        hal_nrf_set_rf_channel(default_channels[ch_i]);

        #ifdef DEBUG_LED_
        P0 = ch_i;
        #endif

        // After changing channels and being in the LISTENING state
        // for 'a while', boot loader loop will check if there is firmware
        // installed, and if so end the while(running) loop.
        bootloader_timer++;
        if (bootloader_timer > BOOTLOADER_TIMEOUT) {
          bootloader_timer = 0;
          running = (hal_flash_byte_read(FW_INSTALLED) == 0x01) ? false : true;
        }
      }

    // While connected must receive something or connection times out.
    // Connection timer reset when packet received.
    } else if (state == CONNECTED) {
      connection_timer++;
      if (connection_timer > CONNECTION_TIMEOUT) {
        state = LISTENING;
      }
    }
	} 

  resetRF();

  #ifdef DEBUG_LED_
  // Default value for P0DIR
  P0 = 0x00;
  P0DIR = 0xFF;
  #endif

  EA = ea_default;
  RF = rf_default;

  // Reads address of firmware's reset vector.
  temp_data[0] = hal_flash_byte_read(FW_RESET_ADDR_H);
  temp_data[1] = hal_flash_byte_read(FW_RESET_ADDR_L);
	firmware = (firmware_start)(((uint16_t)temp_data[0]<<8) | (temp_data[1]));
	
  // Jump to firmware. Goodbye!
	firmware();
}

/* Send function.
 * Write to send_buf[1] - send_buf[31] before calling this function.
 * command will be placed in send_buf[0].*/
void send(command_t command)
{
  uint8_t i;

  // Set operation mode to transmit.
  CE_LOW();
  hal_nrf_set_operation_mode(HAL_NRF_PTX);
  // Copy command to send buffer.
  send_buf[0] = command; 
  hal_nrf_write_tx_payload(send_buf, PAYLOAD_SIZE);
  // Activate sender
  CE_PULSE();
  send_success = false;

  // Wait for radio to transmit
  while (RFF != 1) ;
  RFF = 0;
  nrf_irq(); 
  // Clear send buffer.
  for (i = 0; i < PAYLOAD_SIZE; i++) {
    send_buf[i] = 0x00;
  }
  // Reset operation mode to receive.
  hal_nrf_set_operation_mode(HAL_NRF_PRX);
  CE_HIGH();
}

/* Radio "interrupt" routine.
 * (but it is only manually called) */
void nrf_irq()
{
  uint8_t irq_flags;

  // Read and clear IRQ flags from radio.
  irq_flags = hal_nrf_get_clear_irq_flags();

  switch (irq_flags) {

    // Transmission success
    case (1 << (uint8_t)HAL_NRF_TX_DS):
      send_success = true;
      // Data has been sent
      break;

    // Transmission failed (maximum re-transmits)
    case (1 << (uint8_t)HAL_NRF_MAX_RT):
      hal_nrf_flush_tx();
      send_success = false;
      break;

    // Data received 
    case (1 << (uint8_t)HAL_NRF_RX_DR):
      // Read payload
      while (!hal_nrf_rx_fifo_empty()) { 
        hal_nrf_read_rx_payload(rcvd_buf);
      }
      packet_received = true;
      break;
  
    default:
      ;
  }
}

// Configures RF parameters before Enhanced Shockburst can be used.
void configureRF()
{
  packet_received = false;
  send_success = false;

  // Enable the radio clock
  RFCKEN = 1;
  // Set payload width to 32 bytes
  hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE0, PAYLOAD_SIZE);
  // Set auto-retries to 5 with 500 us intervals
  hal_nrf_set_auto_retr(5, 500);
  // Set pipe address
  hal_nrf_set_address(HAL_NRF_PIPE0, default_pipe_address);
  hal_nrf_set_address(HAL_NRF_TX, default_pipe_address);
  // Set initial channel
  hal_nrf_set_rf_channel(default_channels[1]);
  // Configure radio as primary receiver (PTX)
  hal_nrf_set_operation_mode(HAL_NRF_PRX);
  // Wait for the xtal to power up
  while (hal_clk_get_16m_source() != HAL_CLK_XOSC16M) ;
  // Power up radio
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  // Enable receiver
  CE_HIGH();

  return;
}

// Resets RF parameters to default values.
// Must be called before jumping to new firmware.
void resetRF()
{
  // Reset values set by the RF setup.
  CE_LOW();
  // PWR_UP = 0
  hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
  // PRIM_RX = 0
  hal_nrf_set_operation_mode(HAL_NRF_PTX);
  // RF_CH = 0x02;
  hal_nrf_set_rf_channel(reset_channel);
  // AW = 11 (Default = 5 bytes)
  // RX_ADDR_P0 = TX_ADDR = 0xE7E7E7E7E7
  hal_nrf_set_address(HAL_NRF_TX, reset_pipe_address);
  hal_nrf_set_address(HAL_NRF_PIPE0, reset_pipe_address);
  // ARD = 0000, ARC = 0011
  hal_nrf_set_auto_retr(3, 250);
  // RX_PW_P0 = 0x00
  hal_nrf_set_rx_payload_width((int)HAL_NRF_PIPE0, 0);
  // Disable radio clock
  RFCKEN = 0;

  return;
}

// Sends model number and firmware version number to host.
void sendInitAck(state_t *state)
{
  // Send model number and firmware number
  send_buf[1] = model_number;
  send_buf[2] = hal_flash_byte_read(FW_NUMBER); 
  send(CMD_ACK);

  if (send_success) {
    *state = CONNECTED;
  } else {
    *state = LISTENING;
  }
  
  return;
}

// Verifies that update-start command is valid.
// Will enter RECEIVING_FIRMWARE state if everything checks out.
void startFirmwareUpdate(state_t *state, uint16_t *bytes_total, 
                         uint16_t *bytes_received, uint8_t *firmware_number)
{
  uint8_t i, checksum = 0, reset_vector[3];
  uint16_t bytes = 0;
              
  // Calculate checksum
  for (i = 0; i < UPDATE_START_LENGTH; i++) {
    checksum += MSG_PAYLOAD(i);
  }
  // Checksum fail
  if (checksum != 0) {
    send_buf[1] = ERROR_CHECKSUM_FAIL;
    send(CMD_NACK);
    return;
  }

  // Get firmware size 
  bytes = MSG_ST_BYTES;
  // Check that firmware is within legal size range.
  if (bytes > FLASH_FW_MAX_SIZE) {
    // Send error report to host.
    send_buf[1] = ERROR_ILLEGAL_SIZE;
    send(CMD_NACK);
    return;
  }
  *bytes_total = bytes;

  // Get firmware's reset vector. 
  temp_data[0] = MSG_ST_RESET_OPCODE;
  temp_data[1] = MSG_ST_RESET_ADDR_H;
  temp_data[2] = MSG_ST_RESET_ADDR_L;
  // Write reset vector to non-volatile flash
  hal_flash_page_erase(FW_NV_DATA_PAGE);
  hal_flash_bytes_write(FW_RESET_VECTOR, temp_data, 3);
  // Get firmware serial number. Will be written to NV when update complete.
  *firmware_number = MSG_ST_NUMBER;
  *bytes_received = 0;

  // Read out old reset vector.
  PCON |= PMW;
  hal_flash_bytes_read(0x0000, reset_vector, 3);
  PCON &= ~PMW;
  // Erase first page, containing reset vector.
  hal_flash_page_erase(0);
  // Write back the old reset vector.
  PCON |= PMW;
  hal_flash_bytes_write(0x0000, reset_vector, 3);
  PCON &= ~PMW;
  // Erase the reset of pages available to firmware.
  for (i = 1; i < FLASH_FW_PAGES; i++) {
    hal_flash_page_erase(i);
  }

  send(CMD_ACK);
  if (send_success) {
    *state = RECEIVING_FIRMWARE;
  } else {
    *state = LISTENING;
  }

  return;
}

// Writes hex-record's data field to flash memory.
// Will update bytes_received and send reply to host.
void writeHexRecord(state_t *state, uint16_t *bytes_received)
{
  uint8_t i, checksum = 0, bytes = MSG_WR_BYTE_COUNT;
  uint16_t addr = MSG_WR_ADDR;

  // Disable RF receiving while writing.
  CE_LOW();

  // Calculate checksum for message. 
  for (i = 0; i < bytes+HEX_BYTES; i++) {
    checksum += MSG_PAYLOAD(i);
  }
  if (checksum != 0) {
    // Checksum fail
    send_buf[1] = ERROR_CHECKSUM_FAIL;
    send(CMD_NACK);
    return;
  }

  // Copy data portion of payload to idata temp memory.
  for (i = 0; i < bytes; i++) {
    temp_data[i] = MSG_WR_DATA(i);
  }

  // This will prevent the reset vector from being overwritten. 
  if (addr == 0x0000) {
    PCON |= PMW;
    // Offset write with the 3 bytes of the reset vector
    hal_flash_bytes_write((addr+3), (temp_data+3), (bytes-3));
    PCON &= ~PMW;
    
  // Make sure that bytes to be written is within legal pages.
  } else if (addr+bytes < FLASH_FW_MAX_SIZE) {
    // Write line to flash. 
    PCON |= PMW;
    hal_flash_bytes_write(addr, temp_data, bytes);
    PCON &= ~PMW;

  // Address is outside pages available to new firmware.
  } else {
    // Invalid address
    send_buf[1] = ERROR_ILLEGAL_ADDRESS;
    send(CMD_NACK);
    return;
  }

  // Add bytes to total received.
  *bytes_received += bytes;
  // Acknowledge message
  send(CMD_ACK);
  if (!send_success) {
    *state = ERROR;
  }

  return;
}

// Sends requested bytes of data to host.
void readHexRecord()
{
  uint8_t i, bytes;
  uint16_t addr;

  CE_LOW();

  // Get memory address and number of bytes to read.
  bytes = MSG_RE_BYTE_COUNT;
  addr = MSG_RE_ADDR;
  // Copy flash memory bytes to temporary idata buffer.
  PCON |= PMW;
  hal_flash_bytes_read(addr, temp_data, bytes);
  PCON &= ~PMW;
  // If request is for reset vector, read from non-volatile mem.
  if (addr == 0x0000) {
    hal_flash_bytes_read(FW_RESET_OPCODE, temp_data, 3);
  }
  // Copy to send buffer
  for (i = 0; i < bytes; i++) {
    send_buf[i+1] = temp_data[i];
  }
  send(CMD_ACK);

  return;
}
