/* -----------------------------------------------------------------------------
 * Copyright (c) 2015 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        6. September 2015
 * $Revision:    V1.00
 *
 * Project:      USART Driver Definitions for Milandr MDR1986VE1T
 * -------------------------------------------------------------------------- */

#ifndef __MDR1986VE1T_USART_H
#define __MDR1986VE1T_USART_H

#include <MDR1986VE1T.h>
#include "MDR1986VE1T_PORT.h"

#include "Driver_USART.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#include <math.h>

#define NomClkIrDA 1843200 // 1.8432 MHz

// USART flags
#define USART_FLAG_INITIALIZED       (1 << 0)
#define USART_FLAG_POWERED           (1 << 1)
#define USART_FLAG_CONFIGURED        (1 << 2)
#define USART_FLAG_TX_ENABLED        (1 << 3)
#define USART_FLAG_RX_ENABLED        (1 << 4)
#define USART_FLAG_SEND_ACTIVE       (1 << 5)

// USART Transfer Information (Run-Time)
typedef struct _USART_TRANSFER_INFO
{
  uint32_t                rx_num;        // Total number of data to be received
  uint32_t                tx_num;        // Total number of data to be send
  uint8_t                *rx_buf;        // Pointer to in data buffer
  uint8_t                *tx_buf;        // Pointer to out data buffer
  uint32_t                rx_cnt;        // Number of data received
  uint32_t                tx_cnt;        // Number of data sent
} USART_TRANSFER_INFO;

// USART Information (Run-Time)
typedef struct _USART_INFO
{
  ARM_USART_SignalEvent_t cb_event;      // Event callback
  ARM_USART_STATUS        status;        // Status flags
  USART_TRANSFER_INFO     xfer;          // Transfer information
  uint8_t                 mode;          // USART mode
  uint8_t                 flags;         // USART driver flags
  uint32_t                baudrate;      // Baudrate
} USART_INFO;

// USART Pin Configuration
typedef const struct _USART_PINS
{
	uint8_t							tx_func;
  MDR_PORT_TypeDef*		tx_port;
  uint8_t							tx_bit;
	uint8_t							rx_func;
  MDR_PORT_TypeDef*		rx_port;
  uint8_t							rx_bit;
	uint8_t							cts_func;
  MDR_PORT_TypeDef*		cts_port;
  uint8_t							cts_bit;
	uint8_t							rts_func;
  MDR_PORT_TypeDef*		rts_port;
  uint8_t							rts_bit;
	uint8_t							dcd_func;
  MDR_PORT_TypeDef*		dcd_port;
  uint8_t							dcd_bit;
	uint8_t							dsr_func;
  MDR_PORT_TypeDef*		dsr_port;
  uint8_t							dsr_bit;
	uint8_t							dtr_func;
  MDR_PORT_TypeDef*		dtr_port;
  uint8_t							dtr_bit;
	uint8_t							ri_func;
  MDR_PORT_TypeDef*		ri_port;
  uint8_t							ri_bit;
} USART_PINS;

/* USART Clocks Configuration */
typedef const struct _USART_CLOCK
{
  uint32_t              peri_cfg_val;   // USART peripheral clock configuration register value
  volatile uint32_t    *peri_cfg;       // USART peripheral clock configuration register
	uint32_t							on_msk;
	uint32_t							div;
} USART_CLOCKS;

// USART Resources definitions
typedef struct
{
  ARM_USART_CAPABILITIES  capabilities;  // Capabilities
  MDR_UART_TypeDef				*reg;          // USART peripheral register interface
  USART_PINS              pin;          // USART pins configuration
  USART_CLOCKS            clk;           // USART clocks configuration
  IRQn_Type               irq_num;       // USART IRQ Number
  USART_INFO             *info;          // Run-Time Information
} const USART_RESOURCES;

#endif /* __MDR1986VE1T_USART_H */
