//
// The MIT License (MIT)
//
// Copyright (c) 2019 Nathan Conrad
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
//
#define ACTIVE_LEVEL          0
#define IDN                   "INA260\nINA260:VOLTAGE? INA260:CURRENT? INA260:POWER?"
#define IDN_QUERY             "*idn?"
#define RST_CMD               "*rst"
#define MEASURE_VOLTAGE_QUERY "INA260:VOLTAGE?"
#define MEASURE_CURRENT_QUERY "INA260:CURRENT?"
#define MEASURE_POWER_QUERY   "INA260:POWER?"


#define END_RESPONSE     "\n"            // USB488

#include <strings.h>
#include <stdlib.h>     // atoi //
#include "tusb.h"
#include "bsp/board.h"
#include "main.h"
#include "sam.h" // GPIO //

#include <stdint.h>      // I2C //
#include <stdbool.h>     // I2C //
#include "hal_gpio.h"    // I2C //
#include "i2c_master.h"  // I2C //

void ftoa(float num, char *str);


#if (CFG_TUD_USBTMC_ENABLE_488)
static usbtmc_response_capabilities_488_t const
#else
static usbtmc_response_capabilities_t const
#endif
tud_usbtmc_app_capabilities  =
{
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities =
    {
        .listenOnly = 0,
        .talkOnly = 0,
        .supportsIndicatorPulse = 1
    },
    .bmDevCapabilities = {
        .canEndBulkInOnTermChar = 0
    },

#if (CFG_TUD_USBTMC_ENABLE_488)
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 =
    {
        .supportsTrigger = 1,
        .supportsREN_GTL_LLO = 0,
        .is488_2 = 1
    },
    .bmDevCapabilities488 =
    {
      .SCPI = 1,
      .SR1 = 0,
      .RL1 = 0,
      .DT1 =0,
    }
#endif
};

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV          (0x10u)
#define IEEE4882_STB_SER          (0x20u)
#define IEEE4882_STB_SRQ          (0x40u)

static volatile uint8_t status;

// 0=not query, 1=queried, 2=delay,set(MAV), 3=delay 4=ready?
// (to simulate delay)
static volatile uint16_t queryState = 0;
static volatile uint32_t queryDelayStart;
static volatile uint32_t bulkInStarted;

static volatile bool idnQuery;
static volatile bool rst_cmd;
static volatile bool measure_voltage_query;
static volatile bool measure_current_query;
static volatile bool measure_power_query;
char voltage_str[10];
char current_str[10];
char power_str[10];

static uint32_t resp_delay = 125u; // Adjustable delay, to allow for better testing
static size_t   buffer_len;
static size_t   buffer_tx_ix;      // for transmitting using multiple transfers
static uint8_t  buffer[225];       // A few packets long should be enough.


static usbtmc_msg_dev_dep_msg_in_header_t rspMsg = {
    .bmTransferAttributes =
    {
      .EOM = 1,
      .UsingTermChar = 0
    }
};

void tud_usbtmc_open_cb(uint8_t interface_id)
{
  (void)interface_id;
  tud_usbtmc_start_bus_read();
}

#if (CFG_TUD_USBTMC_ENABLE_488)
usbtmc_response_capabilities_488_t const *
#else
usbtmc_response_capabilities_t const *
#endif
tud_usbtmc_get_capabilities_cb()
{
  return &tud_usbtmc_app_capabilities;
}


bool tud_usbtmc_msg_trigger_cb(usbtmc_msg_generic_t* msg) {
  (void)msg;
  // Let trigger set the SRQ
  status |= IEEE4882_STB_SRQ;
  return true;
}

bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const * msgHeader)
{
  (void)msgHeader;
  buffer_len = 0;
  if(msgHeader->TransferSize > sizeof(buffer))
  {

    return false;
  }
  return true;
}

bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete)
{
  // If transfer isn't finished, we just ignore it (for now)

  if(len + buffer_len < sizeof(buffer))
  {
    memcpy(&(buffer[buffer_len]), data, len);
    buffer_len += len;
  }
  else
  {
    return false; // buffer overflow!
  }

  queryState            = transfer_complete;
  idnQuery              = false;
  rst_cmd               = false;
  measure_voltage_query = false;
  measure_current_query = false;
  measure_power_query   = false;

  if(transfer_complete && (len >=4) && !strncasecmp(IDN_QUERY,data,5))
  {
    idnQuery = true;
  }
  else if (transfer_complete && (len >=4) && !strncasecmp(RST_CMD,data,4))
  {
    rst_cmd = true;
    if ( i2c_start(0x44 << 1) ) // 7bit address
    { // Reset INA260 i2c command
      i2c_write_byte(0x00);
      i2c_write_byte(0x80);
      i2c_write_byte(0x00);
      i2c_stop();
    }
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MEASURE_VOLTAGE_QUERY,data,15))
  {
    measure_voltage_query = true;
    if ( i2c_start(0x44 << 1) ) // 7bit address
    { 
      i2c_write_byte(0x02);
      i2c_stop();
      i2c_start((0x44 << 1) + 1); // add 1 for read start
      uint8_t read_byte = 0x00;
      i2c_read_byte(&read_byte,false);
      uint16_t voltage_word = read_byte << 8;
      i2c_read_byte(&read_byte,true);
      i2c_stop();
      voltage_word = voltage_word + read_byte;
      float voltage_float = voltage_word * 0.00125;
      ftoa(voltage_float,voltage_str);
    }
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MEASURE_CURRENT_QUERY,data,15))
  {
    measure_current_query = true;
    if ( i2c_start(0x44 << 1) ) // 7bit address
    { 
      i2c_write_byte(0x01);
      i2c_stop();
      i2c_start((0x44 << 1) + 1); // add 1 for read start
      uint8_t read_byte = 0x00;
      i2c_read_byte(&read_byte,false);
      uint16_t current_word = read_byte << 8;
      i2c_read_byte(&read_byte,true);
      i2c_stop();
      current_word = current_word + read_byte;
      float current_float;
      if ((current_word >> 15) == 1)
      {
        current_float = -1.0 * (0xFFFF - current_word) * 0.00125;
      }
      else
      {
        current_float = current_word * 0.00125;
      }
      ftoa(current_float,current_str);
    }
  }
  else if (transfer_complete && (len >= 9) && !strncasecmp(MEASURE_POWER_QUERY,data,13))
  {
    measure_power_query = true;
    if ( i2c_start(0x44 << 1) ) // 7bit address
    { 
      i2c_write_byte(0x03);
      i2c_stop();
      i2c_start((0x44 << 1) + 1); // add 1 for read start
      uint8_t read_byte = 0x00;
      i2c_read_byte(&read_byte,false);
      uint16_t power_word = read_byte << 8;
      i2c_read_byte(&read_byte,true);
      i2c_stop();
      power_word = power_word + read_byte;
      float power_float = power_word * 0.01;
      ftoa(power_float,power_str);
    }
  }

  if(transfer_complete && !strncasecmp("delay ",data,5))
  {
    queryState = 0;
    int d = atoi((char*)data + 5);
    if(d > 10000)
      d = 10000;
    if(d<0)
      d=0;
    resp_delay = (uint32_t)d;
  }
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_msgBulkIn_complete_cb()
{
  if((buffer_tx_ix == buffer_len) || idnQuery) // done
  {
    status &= (uint8_t)~(IEEE4882_STB_MAV); // clear MAV
    queryState = 0;
    bulkInStarted = 0;
    buffer_tx_ix = 0;
  }
  tud_usbtmc_start_bus_read();

  return true;
}

static unsigned int msgReqLen;

bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const * request)
{
  rspMsg.header.MsgID = request->header.MsgID,
  rspMsg.header.bTag = request->header.bTag,
  rspMsg.header.bTagInverse = request->header.bTagInverse;
  msgReqLen = request->TransferSize;

#ifdef xDEBUG
  uart_tx_str_sync("MSG_IN_DATA: Requested!\r\n");
#endif
  if(queryState == 0 || (buffer_tx_ix == 0))
  {
    TU_ASSERT(bulkInStarted == 0);
    bulkInStarted = 1;

    // > If a USBTMC interface receives a Bulk-IN request prior to receiving a USBTMC command message
    //   that expects a response, the device must NAK the request (*not stall*)
  }
  else
  {
    size_t txlen = tu_min32(buffer_len-buffer_tx_ix,msgReqLen);
    tud_usbtmc_transmit_dev_msg_data(&buffer[buffer_tx_ix], txlen,
        (buffer_tx_ix+txlen) == buffer_len, false);
    buffer_tx_ix += txlen;
  }
  // Always return true indicating not to stall the EP.
  return true;
}

void usbtmc_app_task_iter(void) {
  switch(queryState) {
  case 0:
    break;
  case 1:
    queryDelayStart = board_millis();
    queryState = 2;
    break;
  case 2:
    if( (board_millis() - queryDelayStart) > resp_delay) {
      queryDelayStart = board_millis();
      queryState=3;
      status |= 0x10u; // MAV
      status |= 0x40u; // SRQ
    }
    break;
  case 3:
    if( (board_millis() - queryDelayStart) > resp_delay) {
      queryState = 4;
    }
    break;
  case 4: // time to transmit;
    if(bulkInStarted && (buffer_tx_ix == 0)) {
      if(idnQuery)
      {
        tud_usbtmc_transmit_dev_msg_data(IDN, tu_min32(sizeof(IDN)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (measure_voltage_query)
      {
        tud_usbtmc_transmit_dev_msg_data(voltage_str, tu_min32(sizeof(voltage_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (measure_current_query)
      {
        tud_usbtmc_transmit_dev_msg_data(current_str, tu_min32(sizeof(current_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (measure_power_query)
      {
        tud_usbtmc_transmit_dev_msg_data(power_str, tu_min32(sizeof(power_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (rst_cmd)
      { 
        tud_usbtmc_transmit_dev_msg_data(END_RESPONSE, tu_min32(sizeof(END_RESPONSE)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else
      {
        buffer_tx_ix = tu_min32(buffer_len,msgReqLen);
        tud_usbtmc_transmit_dev_msg_data(buffer, buffer_tx_ix, buffer_tx_ix == buffer_len, false);
      }

      // MAV is cleared in the transfer complete callback.
    }
    break;
  default:
    TU_ASSERT(false,);
    return;
  }
}

bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  return true;
}

bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp)
{
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  buffer_tx_ix = 0u;
  buffer_len = 0u;
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0u;
  return true;
}
bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult)
{
  bulkInStarted = 0;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;

}
bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

void tud_usbtmc_bulkIn_clearFeature_cb(void)
{
}
void tud_usbtmc_bulkOut_clearFeature_cb(void)
{
  tud_usbtmc_start_bus_read();
}

// Return status byte, but put the transfer result status code in the rspResult argument.
uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult)
{
  uint8_t old_status = status;
  status = (uint8_t)(status & ~(IEEE4882_STB_SRQ)); // clear SRQ

  *tmcResult = USBTMC_STATUS_SUCCESS;
  // Increment status so that we see different results on each read...

  return old_status;
}

bool tud_usbtmc_indicator_pulse_cb(tusb_control_request_t const * msg, uint8_t *tmcResult)
{
  (void)msg;
  led_indicator_pulse();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

//---------------------------- New Code ----------------------------//

/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Definitions -------------------------------------------------------------*/

#define __SAMD21E18A__
HAL_GPIO_PIN(SDA,             A, 16);
HAL_GPIO_PIN(SCL,             A, 17);
#define I2C_SERCOM            SERCOM1
#define I2C_SERCOM_PMUX       PORT_PMUX_PMUXE_C_Val
#define I2C_SERCOM_GCLK_ID    SERCOM1_GCLK_ID_CORE
#define I2C_SERCOM_CLK_GEN    0
#define I2C_SERCOM_APBCMASK   PM_APBCMASK_SERCOM1
#define T_RISE                100e-9 // Depends on the board, actually
#define F_CPU                 48000000UL

enum
{
  I2C_TRANSFER_WRITE = 0,
  I2C_TRANSFER_READ  = 1,
};

enum
{
  I2C_PINS_SDA = (1 << 0),
  I2C_PINS_SCL = (1 << 1),
};


/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
int i2c_init(int freq)
{
  int baud   = ((float)F_CPU / freq - (float)F_CPU * T_RISE - 10.0) / 2.0;
  
  if (baud < 0)
    baud = 0;
  else if (baud > 255)
    baud = 255;

  uint8_t speed_val;
  if (freq <= 500000)
    speed_val = 0;
  else if (freq <= 1500000)
    speed_val = 1;
  else
    speed_val = 2;

  freq = (float)F_CPU / (2.0 * (5.0 + baud) + (float)F_CPU * T_RISE);

  PM->APBCMASK.reg |= I2C_SERCOM_APBCMASK;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(I2C_SERCOM_GCLK_ID) |
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(I2C_SERCOM_CLK_GEN);

  I2C_SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SWRST;
  while (I2C_SERCOM->I2CM.CTRLA.reg & SERCOM_I2CM_CTRLA_SWRST);

  I2C_SERCOM->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

//  I2C_SERCOM->I2CM.BAUD.reg   = SERCOM_I2CM_BAUD_BAUD(baud);
  I2C_SERCOM->I2CM.BAUD.reg   = SERCOM_I2CM_BAUD_BAUD(20);
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_ENABLE |
      SERCOM_I2CM_CTRLA_MODE_I2C_MASTER  |
      SERCOM_I2CS_CTRLA_SPEED(speed_val) | SERCOM_I2CM_CTRLA_SDAHOLD(3);
  while (I2C_SERCOM->I2CM.SYNCBUSY.reg);

  I2C_SERCOM->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);

  HAL_GPIO_SDA_in();
  HAL_GPIO_SDA_clr();
  HAL_GPIO_SDA_pmuxen(I2C_SERCOM_PMUX);

  HAL_GPIO_SCL_in();
  HAL_GPIO_SCL_clr();
  HAL_GPIO_SCL_pmuxen(I2C_SERCOM_PMUX);

  return freq;
}

//-----------------------------------------------------------------------------
bool i2c_start(int addr)
{
  I2C_SERCOM->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_ERROR;

  I2C_SERCOM->I2CM.ADDR.reg = addr;

  int timeout_count = 0; // if no nack, timeout

  while ( (0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) &&
           0 == (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB)    ) ||
          (8000 == timeout_count) )
    timeout_count++;

  if (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK ||
      I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_ERROR)
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_stop(void)
{
  if ((I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_MB) ||
      (I2C_SERCOM->I2CM.INTFLAG.reg & SERCOM_I2CM_INTFLAG_SB))
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
  }

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_read_byte(uint8_t *read_byte, bool last)
{
  while (1)
  {
    int flags = I2C_SERCOM->I2CM.INTFLAG.reg;

    if (flags & SERCOM_I2CM_INTFLAG_SB)
      break;

    if (flags & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_ERROR))
      return false;
  }

  if (last)
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(3);
  else
    I2C_SERCOM->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

  *read_byte = I2C_SERCOM->I2CM.DATA.reg;

  return true;
}

//-----------------------------------------------------------------------------
bool i2c_write_byte(uint8_t byte)
{
  I2C_SERCOM->I2CM.DATA.reg = byte;

  while (1)
  {
    int flags = I2C_SERCOM->I2CM.INTFLAG.reg;

    if (flags & SERCOM_I2CM_INTFLAG_MB)
      break;

    if (flags & (SERCOM_I2CM_INTFLAG_SB | SERCOM_I2CM_INTFLAG_ERROR))
      return false;
  }

  if (I2C_SERCOM->I2CM.STATUS.reg & SERCOM_I2CM_STATUS_RXNACK)
  {
    I2C_SERCOM->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(3);
    return false;
  }

  return true;
}

void ftoa(float num, char *str)
{
  int intpart = num;
  int intdecimal;
  char whole[10];
  uint32_t i;
  float decimal_part;
  char decimal[20];

  // clear string result
  memset(str, 0, 10);

  if (num < 0.0) // handle negative numbers
  {
    strcat(str,"-");
  }
  itoa(num, whole, 10);
  strcat(str,whole);

  strcat(str, ".");

  decimal_part = num - intpart;
  intdecimal = decimal_part * 1000;

  if(intdecimal < 0)
  {
    intdecimal = -intdecimal;
  }
  itoa(intdecimal, decimal, 10);
  for(i =0;i < (3 - strlen(decimal));i++)
  {
    strcat(str, "0");
  }
  strcat(str, decimal);
}