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
 * $Date:        11. October 2015
 * $Revision:    V1.00
 *
 * Driver:       Driver_USART0, Driver_USART1
 * Configured:   via RTE_Device.h configuration file
 * Project:      USART Driver for Milandr MDR1986VE9X
 * -----------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                Value     UART Interface
 *   ---------------------                -----     --------------
 *   Connect to hardware via Driver_USART# = 0      MDR_UART1
 *   Connect to hardware via Driver_UsART# = 1      MDR_UART2
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    - Initial release
 */

#include "MDR1986VE9X_USART.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 00)  /* driver version */

// Driver Version
static const ARM_DRIVER_VERSION USART_DriverVersion = {ARM_USART_API_VERSION, ARM_USART_DRV_VERSION};

// UART0
#if (RTE_UART0)
static USART_INFO USART0_Info = {0};

static const USART_RESOURCES USART0_Resources =
{
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode 
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    0,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#if (RTE_UART0_RTS_PIN_EN == 1)
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#if (RTE_UART0_CTS_PIN_EN == 1)
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    0,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#if (RTE_UART0_RTS_PIN_EN == 1)
    1,  // RTS Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART0_CTS_PIN_EN == 1)
    1,  // CTS Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART0_DTR_PIN_EN == 1)
    1,  // DTR Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART0_DSR_PIN_EN == 1)
    1,  // DSR Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART0_DCD_PIN_EN == 1)
    1,  // DCD Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART0_RI_PIN_EN == 1)
    1,  // RI Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART0_CTS_PIN_EN == 1)
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,
#endif
#if (RTE_UART0_DSR_PIN_EN == 1)
    1,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
#else
    0,
#endif
#if (RTE_UART0_DCD_PIN_EN == 1)
    1,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
#else
    0,
#endif
#if (RTE_UART0_RI_PIN_EN == 1)
    1,  // Signal RI change event: \ref ARM_USART_EVENT_RI
#else
    0,
#endif
  },
    MDR_UART1,
  {     // USART Pin Configuration
    RTE_UART0_TX_FUNC,
		RTE_UART0_TX_PORT,
		RTE_UART0_TX_BIT,
		RTE_UART0_RX_FUNC,
		RTE_UART0_RX_PORT,
		RTE_UART0_RX_BIT,
#if (RTE_UART0_CTS_PIN_EN == 1)
    RTE_UART0_CTS_FUNC,
		RTE_UART0_CTS_PORT,
		RTE_UART0_CTS_BIT,
#else
		PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART0_RTS_PIN_EN == 1)
    RTE_UART0_RTS_FUNC,
		RTE_UART0_RTS_PORT,
		RTE_UART0_RTS_BIT,
#else
		PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART0_DCD_PIN_EN == 1)
    RTE_UART0_DCD_FUNC,
		RTE_UART0_DCD_PORT,
		RTE_UART0_DCD_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART0_DSR_PIN_EN == 1)
		RTE_UART0_DSR_FUNC,
		RTE_UART0_DSR_PORT,
		RTE_UART0_DSR_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART0_DTR_PIN_EN == 1)
    RTE_UART0_DTR_FUNC,
		RTE_UART0_DTR_PORT,
		RTE_UART0_DTR_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART0_RI_PIN_EN == 1)
    RTE_UART0_RI_FUNC,
		RTE_UART0_RI_PORT,
		RTE_UART0_RI_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
  },
  {     // USART Clocks Configuration
		RST_CLK_UART_CLOCK_UART1_CLK_EN | ((RTE_UART0_BRG << RST_CLK_UART_CLOCK_UART1_BRG_Pos) & RST_CLK_UART_CLOCK_UART1_BRG_Msk),
    &(MDR_RST_CLK->UART_CLOCK), RST_CLK_PER_CLOCK_PCLK_EN_UART1, (1 << RTE_UART0_BRG)
  },
	UART1_IRQn,
	&USART0_Info,
};
#endif

// UART1
#if (RTE_UART1)
static USART_INFO USART1_Info = {0};

static const USART_RESOURCES USART1_Resources =
{
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode 
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    0,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#if (RTE_UART1_RTS_PIN_EN == 1)
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#if (RTE_UART1_CTS_PIN_EN == 1)
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    0,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#if (RTE_UART1_RTS_PIN_EN == 1)
    1,  // RTS Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART1_CTS_PIN_EN == 1)
    1,  // CTS Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART1_DTR_PIN_EN == 1)
    1,  // DTR Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART1_DSR_PIN_EN == 1)
    1,  // DSR Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART1_DCD_PIN_EN == 1)
    1,  // DCD Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART1_RI_PIN_EN == 1)
    1,  // RI Line: 0=not available, 1=available
#else
    0,
#endif
#if (RTE_UART1_CTS_PIN_EN == 1)
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,
#endif
#if (RTE_UART1_DSR_PIN_EN == 1)
    1,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
#else
    0,
#endif
#if (RTE_UART1_DCD_PIN_EN == 1)
    1,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
#else
    0,
#endif
#if (RTE_UART1_RI_PIN_EN == 1)
    1,  // Signal RI change event: \ref ARM_USART_EVENT_RI
#else
    0,
#endif
  },
    MDR_UART2,
  {     // USART Pin Configuration
    RTE_UART1_TX_FUNC,
		RTE_UART1_TX_PORT,
		RTE_UART1_TX_BIT,
		RTE_UART1_RX_FUNC,
		RTE_UART1_RX_PORT,
		RTE_UART1_RX_BIT,
#if (RTE_UART1_CTS_PIN_EN == 1)
    RTE_UART1_CTS_FUNC,
		RTE_UART1_CTS_PORT,
		RTE_UART1_CTS_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART1_RTS_PIN_EN == 1)
    RTE_UART1_RTS_FUNC,
		RTE_UART1_RTS_PORT,
		RTE_UART1_RTS_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART1_DCD_PIN_EN == 1)
    RTE_UART1_DCD_FUNC,
		RTE_UART1_DCD_PORT,
		RTE_UART1_DCD_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART1_DSR_PIN_EN == 1)
		RTE_UART1_DSR_FUNC,
		RTE_UART1_DSR_PORT,
		RTE_UART1_DSR_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART1_DTR_PIN_EN == 1)
    RTE_UART1_DTR_FUNC,
		RTE_UART1_DTR_PORT,
		RTE_UART1_DTR_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
#if (RTE_UART1_RI_PIN_EN == 1)
    RTE_UART1_RI_FUNC,
		RTE_UART1_RI_PORT,
		RTE_UART1_RI_BIT,
#else
    PORT_FUNC_MODE_PORT,
		NULL,
		0,
#endif
  },
  {     // USART Clocks Configuration
		RST_CLK_UART_CLOCK_UART2_CLK_EN | ((RTE_UART1_BRG << RST_CLK_UART_CLOCK_UART2_BRG_Pos) & RST_CLK_UART_CLOCK_UART2_BRG_Msk),
    &(MDR_RST_CLK->UART_CLOCK), RST_CLK_PER_CLOCK_PCLK_EN_UART2, (1 << RTE_UART1_BRG)
  },
	UART2_IRQn,
	&USART1_Info,
};
#endif

//
//   Functions
//

void USART_PinConfigure (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t Func, bool PinState)
{
	Port->ANALOG |= (1<<Pin);
	Port->FUNC &= ~(PORT_FUNC_MODE_Msk<<Pin*2);
	Port->FUNC |= (Func<<Pin*2);
	
	if (PinState)
	{
		Port->RXTX &= ~(1<<Pin);
		Port->OE |= (1<<Pin);
		Port->PWR |= (PORT_PWR_Msk<<Pin*2);
		Port->PD &= ~(1<<Pin);
	}
	else
	{
		Port->OE &= ~(1<<Pin);
		Port->GFEN &= ~(1<<Pin);
		Port->PD &= ~(1<<(Pin+PORT_PD_SHM_Pos));
	}
	
	Port->PULL &= ~((1<<Pin) | (1<<(Pin+PORT_PULL_UP_Pos)));
}

void USART_PinUnconfigure (MDR_PORT_TypeDef* Port, uint8_t Pin)
{
	Port->RXTX &= ~(1<<Pin);
	Port->ANALOG &= ~(1<<Pin);
	Port->FUNC &= ~(PORT_FUNC_MODE_Msk<<Pin*2);
	Port->OE &= ~(1<<Pin);
	Port->PWR &= ~(PORT_PWR_Msk<<Pin*2);
	Port->GFEN &= ~(1<<Pin);
	Port->PULL &= ~((1<<Pin) | (1<<(Pin+PORT_PULL_UP_Pos)));
	Port->PD &= ~((1<<Pin) | (1<<(Pin+PORT_PD_SHM_Pos)));
}

void USART_SetBaudrate (uint32_t USART_Baudrate, USART_RESOURCES *usart)
{
	float Div;
	uint16_t DivInt;
	uint8_t DivFl;
	
	Div = RTE_HCLK / usart->clk.div / (16*USART_Baudrate);
	DivInt = floorf (Div);
	DivFl = lroundf ((Div-DivInt)*64+0.5);
	
	usart->reg->IBRD = DivInt;
	usart->reg->FBRD = DivFl;
	usart->reg->LCR_H = usart->reg->LCR_H;
	
	usart->info->baudrate = USART_Baudrate;
}

static ARM_DRIVER_VERSION USART_GetVersion (void)
{
	return USART_DriverVersion;
}

static ARM_USART_CAPABILITIES USART_GetCapabilities (USART_RESOURCES *usart)
{
	return usart->capabilities;
}

static int32_t USART_Initialize (ARM_USART_SignalEvent_t cb_event,
	USART_RESOURCES *usart)
{
	if (usart->info->flags & USART_FLAG_POWERED)
    // Device is powered - could not be re-initialized
    return ARM_DRIVER_ERROR;

  if (usart->info->flags & USART_FLAG_INITIALIZED)
    // Driver is already initialized
    return ARM_DRIVER_OK;
	
	// Initialize USART Run-time Resources
  usart->info->cb_event = cb_event;

  usart->info->status.tx_busy          = 0;
  usart->info->status.rx_busy          = 0;
  usart->info->status.tx_underflow     = 0;
  usart->info->status.rx_overflow      = 0;
  usart->info->status.rx_break         = 0;
  usart->info->status.rx_framing_error = 0;
  usart->info->status.rx_parity_error  = 0;

  usart->info->mode = 0;
	
	// Configure TX pin
	ClkEnPort (usart->pin.tx_port);
	USART_PinConfigure (usart->pin.tx_port, usart->pin.tx_bit, usart->pin.tx_func, PORT_PIN_OUT);
	
	// Configure RX pin
	ClkEnPort (usart->pin.rx_port);
	USART_PinConfigure (usart->pin.rx_port, usart->pin.rx_bit, usart->pin.rx_func, PORT_PIN_IN);
	
	// Configure CTS pin
  if (usart->capabilities.cts)
	{
		ClkEnPort (usart->pin.cts_port);
		USART_PinConfigure (usart->pin.cts_port, usart->pin.cts_bit, usart->pin.cts_func, PORT_PIN_IN);
	}
	
	// Configure RTS pin
  if (usart->capabilities.rts)
	{
		ClkEnPort (usart->pin.rts_port);
		USART_PinConfigure (usart->pin.rts_port, usart->pin.rts_bit, usart->pin.rts_func, PORT_PIN_OUT);
	}
	
	// Configure DCD pin
  if (usart->capabilities.dcd)
	{
		ClkEnPort (usart->pin.dcd_port);
		USART_PinConfigure (usart->pin.dcd_port, usart->pin.dcd_bit, usart->pin.dcd_func, PORT_PIN_IN);
	}
	
	// Configure DSR pin
  if (usart->capabilities.dsr)
	{
		ClkEnPort (usart->pin.dsr_port);
		USART_PinConfigure (usart->pin.dsr_port, usart->pin.dsr_bit, usart->pin.dsr_func, PORT_PIN_IN);
	}
	
	// Configure DTR pin
  if (usart->capabilities.dtr)
	{
		ClkEnPort (usart->pin.dtr_port);
		USART_PinConfigure (usart->pin.dtr_port, usart->pin.dtr_bit, usart->pin.dtr_func, PORT_PIN_OUT);
	}
	
	// Configure RI pin
  if (usart->capabilities.ri)
	{
		ClkEnPort (usart->pin.ri_port);
		USART_PinConfigure (usart->pin.ri_port, usart->pin.ri_bit, usart->pin.ri_func, PORT_PIN_IN);
	}
	
	usart->info->flags = USART_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

static int32_t USART_Uninitialize (USART_RESOURCES *usart)
{
	if (usart->info->flags & USART_FLAG_POWERED)
    // Driver is powered - could not be uninitialized
    return ARM_DRIVER_ERROR;

  if (usart->info->flags == 0)
    // Driver not initialized
    return ARM_DRIVER_OK;
	
	// Reset TX pin configuration
	USART_PinUnconfigure (usart->pin.tx_port, usart->pin.tx_bit);
	
	// Reset RX pin configuration
	USART_PinUnconfigure (usart->pin.rx_port, usart->pin.rx_bit);
	
	// Reset CTS pin configuration
	if (usart->capabilities.cts)
		USART_PinUnconfigure (usart->pin.cts_port, usart->pin.cts_bit);
	
	// Reset RTS pin configuration
	if (usart->capabilities.rts)
		USART_PinUnconfigure (usart->pin.rts_port, usart->pin.rts_bit);
	
	// Reset DCD pin configuration
	if (usart->capabilities.dcd)
		USART_PinUnconfigure (usart->pin.dcd_port, usart->pin.dcd_bit);
	
	// Reset DSR pin configuration
	if (usart->capabilities.dsr)
		USART_PinUnconfigure (usart->pin.dsr_port, usart->pin.dsr_bit);
	
	// Reset DTR pin configuration
	if (usart->capabilities.dtr)
		USART_PinUnconfigure (usart->pin.dtr_port, usart->pin.dtr_bit);
	
	// Reset RI pin configuration
	if (usart->capabilities.ri)
		USART_PinUnconfigure (usart->pin.ri_port, usart->pin.ri_bit);
	
	// Reset USART status flags
  usart->info->flags = 0;

  return ARM_DRIVER_OK;
}

static int32_t USART_PowerControl (ARM_POWER_STATE state,
	USART_RESOURCES *usart)
{
	if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0)
    // Return error, if USART is not initialized
    return ARM_DRIVER_ERROR;

  if (usart->info->status.rx_busy == 1)
    // Receive busy
    return ARM_DRIVER_ERROR_BUSY;

  if (usart->info->flags & USART_FLAG_SEND_ACTIVE)
    // Transmit busy
    return ARM_DRIVER_ERROR_BUSY;
	
  if (usart->info->flags & USART_FLAG_POWERED)
    if (usart->reg->FR & UART_FR_BUSY)
      // Transmit busy
			return ARM_DRIVER_ERROR_BUSY;
		
	switch (state)
	{
    case ARM_POWER_OFF:
      if ((usart->info->flags & USART_FLAG_POWERED) == 0)
        return ARM_DRIVER_OK;

      // Disable UART IRQ
      NVIC_DisableIRQ (usart->irq_num);
			
			usart->reg->CR &= ~UART_CR_UARTEN;   // Disable UART

      // Deactivate UART peripheral clock
      *(usart->clk.peri_cfg) &= ~(usart->clk.peri_cfg_val);
			MDR_RST_CLK->PER_CLOCK &= ~(usart->clk.on_msk);

      usart->info->flags = USART_FLAG_INITIALIZED;
			
      break;
    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if (usart->info->flags & USART_FLAG_POWERED)
        return ARM_DRIVER_OK;

      // Activate UART peripheral clock
			MDR_RST_CLK->PER_CLOCK |= (usart->clk.on_msk);
      *(usart->clk.peri_cfg) |= (usart->clk.peri_cfg_val);

      // Reset USART peripheral
			usart->reg->RSR_ECR = 0x00000000;
			usart->reg->ILPR = 0x00000000;
			usart->reg->IBRD = 0x00000000;
			usart->reg->FBRD = 0x00000000;
			usart->reg->LCR_H = 0x00000000;
			usart->reg->CR = 0x00000300;
			usart->reg->IFLS = 0x00000012;

      // Disable transmitter
      usart->reg->CR &= ~UART_CR_TXE;

      // Disable receiver
      usart->reg->CR &= ~UART_CR_RXE;

      // Disable interrupts
      usart->reg->IMSC = 0x00000000;
			usart->reg->ICR = 0x000007FF;

			// Enable modem lines status interrupts
			if (usart->capabilities.cts)
				usart->reg->IMSC |= UART_IMSC_CTSMIM;
			
			if (usart->capabilities.dcd)
				usart->reg->IMSC |= UART_IMSC_DCDMIM;
			
			if (usart->capabilities.dsr)
				usart->reg->IMSC |= UART_IMSC_DSRMIM;
			
			if (usart->capabilities.ri)
				usart->reg->IMSC |= UART_IMSC_RIMIM;
			
			usart->reg->CR |= UART_CR_UARTEN;   // Enable UART

      usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

      // Clear and Enable USART IRQ
      NVIC_ClearPendingIRQ (usart->irq_num);
      NVIC_EnableIRQ (usart->irq_num);

      break;
    default: return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
	
  return ARM_DRIVER_OK;
}

static int32_t USART_Send (const void *data, uint32_t num,
	USART_RESOURCES *usart)
{
	if ((data == NULL) || (num == 0))
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0)
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;

  if (usart->info->flags & USART_FLAG_SEND_ACTIVE)
    // Send is not completed yet
    return ARM_DRIVER_ERROR_BUSY;

  // Set Send active flag
  usart->info->flags |= USART_FLAG_SEND_ACTIVE;
	
	 // Save transmit buffer info
  usart->info->xfer.tx_buf = (uint8_t *)data;
  usart->info->xfer.tx_num = num;
  usart->info->xfer.tx_cnt = 0;
	
	// Start transmit data
  if (usart->reg->FR & UART_FR_TXFE)
		usart->reg->DR = usart->info->xfer.tx_buf[usart->info->xfer.tx_cnt];
	
	// Enable transmit interrupt
	usart->reg->IMSC |= UART_IMSC_TXIM;

  return ARM_DRIVER_OK;
}

static int32_t USART_Receive (void *data, uint32_t num,
	USART_RESOURCES *usart)
{
	if ((data == NULL) || (num == 0))
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0)
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;

  // Check if receiver is busy
  if (usart->info->status.rx_busy == 1)
    return ARM_DRIVER_ERROR_BUSY;

  // Set RX busy flag
  usart->info->status.rx_busy = 1;
	
	// Save number of data to be received
  usart->info->xfer.rx_num = num;

  // Clear RX statuses
  usart->info->status.rx_break          = 0;
  usart->info->status.rx_framing_error  = 0;
  usart->info->status.rx_overflow       = 0;
  usart->info->status.rx_parity_error   = 0;

  // Save receive buffer info
  usart->info->xfer.rx_buf = (uint8_t *)data;
  usart->info->xfer.rx_cnt =            0;
	
	// RX Line interrupt enable (overrun, framing, parity error, break)
	usart->reg->IMSC |= (UART_IMSC_RXIM | UART_IMSC_FEIM |
		UART_IMSC_PEIM | UART_IMSC_BEIM | UART_IMSC_OEIM);
	
	return ARM_DRIVER_OK;
}

static int32_t USART_Transfer (const void *data_out, void *data_in, uint32_t num,
	USART_RESOURCES *usart)
{
	if ((data_out == NULL) || (data_in == NULL) || (num == 0))
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0)
    // USART is not configured
    return ARM_DRIVER_ERROR;
	
	return ARM_DRIVER_OK;
}

static uint32_t USART_GetTxCount (USART_RESOURCES *usart)
{
	return usart->info->xfer.tx_cnt;
}

static uint32_t USART_GetRxCount (USART_RESOURCES *usart)
{
	return usart->info->xfer.rx_cnt;
}

static int32_t USART_Control (uint32_t control, uint32_t arg,
	USART_RESOURCES *usart)
{
	uint32_t mode;

  if ((usart->info->flags & USART_FLAG_POWERED) == 0)
    // USART not powered
    return ARM_DRIVER_ERROR;

  switch (control & ARM_USART_CONTROL_Msk)
	{
    // Control TX
    case ARM_USART_CONTROL_TX:
      if (arg)
			{
        usart->info->flags |= USART_FLAG_TX_ENABLED;
        usart->reg->CR |= UART_CR_TXE;
				usart->reg->IMSC |= UART_IMSC_TXIM;
      }
			else
			{
        usart->info->flags &= ~USART_FLAG_TX_ENABLED;
        usart->reg->CR &= ~UART_CR_TXE;
				usart->reg->IMSC &= ~UART_IMSC_TXIM;
				usart->reg->ICR |= UART_ICR_TXIC;
      }
			
      return ARM_DRIVER_OK;
    // Control RX
    case ARM_USART_CONTROL_RX:
      if (arg)
			{
        usart->info->flags |= USART_FLAG_RX_ENABLED;
        usart->reg->CR |= UART_CR_RXE;
				// RX Line interrupt enable (overrun, framing, parity error, break)
				usart->reg->IMSC |= (UART_IMSC_RXIM | UART_IMSC_FEIM |
					UART_IMSC_PEIM | UART_IMSC_BEIM | UART_IMSC_OEIM);
      }
			else
			{
        usart->info->flags &= ~USART_FLAG_RX_ENABLED;
				usart->reg->CR &= ~UART_CR_RXE;
				usart->reg->IMSC &= ~(UART_IMSC_RXIM | UART_IMSC_FEIM |
					UART_IMSC_PEIM | UART_IMSC_BEIM | UART_IMSC_OEIM);
				usart->reg->ICR |= (UART_ICR_RXIC | UART_ICR_FEIC |
					UART_ICR_PEIC | UART_ICR_BEIC | UART_ICR_OEIC);
      }
			
      return ARM_DRIVER_OK;
		// Control break
    case ARM_USART_CONTROL_BREAK:
      if (arg)
			{
        if (usart->info->flags & USART_FLAG_SEND_ACTIVE)
					return ARM_DRIVER_ERROR_BUSY;

        usart->reg->LCR_H |= UART_LCR_H_BRK;

        // Set Send active flag
        usart->info->flags |= USART_FLAG_SEND_ACTIVE;
      }
      else
			{
        usart->reg->LCR_H &= ~UART_LCR_H_BRK;

        // Clear Send active flag
        usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;
			}
				
      return ARM_DRIVER_OK;
    // Abort Send
    case ARM_USART_ABORT_SEND:
      // Disable transmit interrupt
      usart->reg->IMSC &= ~UART_IMSC_TXIM;
			usart->reg->ICR |= UART_ICR_TXIC;

      // Clear Send active flag
      usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;
		
      return ARM_DRIVER_OK;
    // Abort receive
    case ARM_USART_ABORT_RECEIVE:
      // Disable receive interrupt
      usart->reg->IMSC &= ~(UART_IMSC_RXIM | UART_IMSC_FEIM |
					UART_IMSC_PEIM | UART_IMSC_BEIM | UART_IMSC_OEIM);
			usart->reg->ICR |= (UART_ICR_RXIC | UART_ICR_FEIC |
					UART_ICR_PEIC | UART_ICR_BEIC | UART_ICR_OEIC);

      // Clear RX busy status
      usart->info->status.rx_busy = 0;
		
      return ARM_DRIVER_OK;
    // Abort transfer
    case ARM_USART_ABORT_TRANSFER:
      // Disable transmit and receive interrupts
			usart->reg->IMSC &= ~(UART_IMSC_RXIM | UART_IMSC_TXIM |
				UART_IMSC_FEIM | UART_IMSC_PEIM | UART_IMSC_BEIM | UART_IMSC_OEIM);
			usart->reg->ICR |= (UART_ICR_RXIC | UART_ICR_TXIC |
				UART_ICR_FEIC | UART_ICR_PEIC | UART_ICR_BEIC | UART_ICR_OEIC);

      // Clear busy statuses
      usart->info->status.rx_busy = 0;
      usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;
		
      return ARM_DRIVER_OK;
  }

  switch (control & ARM_USART_CONTROL_Msk)
	{
    case ARM_USART_MODE_ASYNCHRONOUS:
      mode = ARM_USART_MODE_ASYNCHRONOUS;
		
      break;
    case ARM_USART_MODE_IRDA:
      if (usart->capabilities.irda)
        // Enable IrDA mode
				usart->reg->CR |= UART_CR_SIREN;
			else
				return ARM_USART_ERROR_MODE;
			
      mode = ARM_USART_MODE_IRDA;
			
      break;
    // IrDA pulse
    case ARM_USART_SET_IRDA_PULSE:
      if (usart->capabilities.irda)
				usart->reg->ILPR = RTE_HCLK / usart->clk.div / NomClkIrDA;
			else 
				return ARM_DRIVER_ERROR;
			
      return ARM_DRIVER_OK;
    // Unsupported command
    default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  // Check if Receiver/Transmitter is busy
  if (usart->info->status.rx_busy ||
		(usart->info->flags & USART_FLAG_SEND_ACTIVE))
			return ARM_DRIVER_ERROR_BUSY;

  // USART Data bits
  switch (control & ARM_USART_DATA_BITS_Msk)
	{
    case ARM_USART_DATA_BITS_5:
			usart->reg->LCR_H &= ~UART_LCR_H_WLEN_Msk;
			usart->reg->LCR_H |= (UART_LCR_H_WLEN_5_BITS << UART_LCR_H_WLEN_Pos);
		
			break;
    case ARM_USART_DATA_BITS_6:
			usart->reg->LCR_H &= ~UART_LCR_H_WLEN_Msk;
			usart->reg->LCR_H |= (UART_LCR_H_WLEN_6_BITS << UART_LCR_H_WLEN_Pos);
		
			break;
    case ARM_USART_DATA_BITS_7:
			usart->reg->LCR_H &= ~UART_LCR_H_WLEN_Msk;
			usart->reg->LCR_H |= (UART_LCR_H_WLEN_7_BITS << UART_LCR_H_WLEN_Pos);
		
			break;
    case ARM_USART_DATA_BITS_8:
			usart->reg->LCR_H &= ~UART_LCR_H_WLEN_Msk;
			usart->reg->LCR_H |= (UART_LCR_H_WLEN_8_BITS << UART_LCR_H_WLEN_Pos);
			
			break;
    default:
			return ARM_USART_ERROR_DATA_BITS;
  }

  // USART Parity
  switch (control & ARM_USART_PARITY_Msk)
	{
    case ARM_USART_PARITY_NONE:
			usart->reg->LCR_H &= ~UART_LCR_H_PEN;
			
			break;
    case ARM_USART_PARITY_EVEN:
			usart->reg->LCR_H |= UART_LCR_H_PEN;
			usart->reg->LCR_H &= ~UART_LCR_H_EPS;
			
			break;
    case ARM_USART_PARITY_ODD:
			usart->reg->LCR_H |= (UART_LCR_H_PEN | UART_LCR_H_EPS);
  }

  // USART Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk)
	{
    case ARM_USART_STOP_BITS_1:
			usart->reg->LCR_H &= ~UART_LCR_H_STP2;
			
			break;
    case ARM_USART_STOP_BITS_2:
			usart->reg->LCR_H |= UART_LCR_H_STP2;
			
			break;
    default:
			return ARM_USART_ERROR_STOP_BITS;
  }

	usart->reg->CR &= ~(UART_CR_RTSEN | UART_CR_CTSEN);
		
  switch (control & ARM_USART_FLOW_CONTROL_Msk)
	{
		case ARM_USART_FLOW_CONTROL_NONE:
      break;
    case ARM_USART_FLOW_CONTROL_RTS:
      if (usart->capabilities.flow_control_rts)
        usart->reg->CR |= UART_CR_RTSEN;
      else
				return ARM_USART_ERROR_FLOW_CONTROL;
				
      break;
    case ARM_USART_FLOW_CONTROL_CTS:
      if (usart->capabilities.flow_control_cts)
        usart->reg->CR |= UART_CR_CTSEN;
      else
				return ARM_USART_ERROR_FLOW_CONTROL;
				
      break;
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
      if (usart->capabilities.flow_control_rts &&
				usart->capabilities.flow_control_cts)
					usart->reg->CR |= (UART_CR_RTSEN | UART_CR_CTSEN);
			else
				return ARM_USART_ERROR_FLOW_CONTROL;
				
      break;
    default:
      return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // USART Baudrate
  USART_SetBaudrate (arg, usart);

  // Configuration is OK - Mode is valid
  usart->info->mode = mode;

  // Set configured flag
  usart->info->flags |= USART_FLAG_CONFIGURED;

  return ARM_DRIVER_OK;
}

static ARM_USART_STATUS USART_GetStatus (USART_RESOURCES *usart)
{
	usart->info->status.tx_busy = (usart->reg->FR & UART_FR_TXFE ? (0) : (1));
	
  return usart->info->status;
}

static int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL control,
	USART_RESOURCES *usart)
{
	if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0)
    // USART is not configured
    return ARM_DRIVER_ERROR;
	
	if (control == ARM_USART_RTS_CLEAR)
	{
    if (usart->capabilities.rts)
			usart->reg->CR &= ~UART_CR_RTS;
    else
			return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
	
  if (control == ARM_USART_RTS_SET)
	{
    if (usart->capabilities.rts)
			usart->reg->CR |= UART_CR_RTS;
    else
			return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
	
  if (control == ARM_USART_DTR_CLEAR)
	{
    if (usart->capabilities.dtr)
			usart->reg->CR &= ~UART_CR_DTR;
    else
			return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
	
  if (control == ARM_USART_DTR_SET)
	{
    if (usart->capabilities.dtr)
			usart->reg->CR |= UART_CR_DTR;
    else
			return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
	
  return ARM_DRIVER_OK;
}

static ARM_USART_MODEM_STATUS USART_GetModemStatus (USART_RESOURCES *usart)
{
	ARM_USART_MODEM_STATUS modem_status;
  uint32_t fr;

  if (usart->info->flags & USART_FLAG_CONFIGURED)
	{
    fr = usart->reg->FR;

    if (usart->capabilities.cts)
			modem_status.cts = (fr & UART_FR_CTS ? (1) : (0));
    else
			modem_status.cts = 0;
		
    if (usart->capabilities.dsr)
			modem_status.dsr = (fr & UART_FR_DSR ? (1) : (0));
    else
			modem_status.dsr = 0;
		
    if (usart->capabilities.ri)
			modem_status.ri  = (fr & UART_FR_RI  ? (1) : (0));
    else
			modem_status.ri  = 0;
		
    if (usart->capabilities.dcd)
			modem_status.dcd = (fr & UART_FR_DCD ? (1) : (0));
    else
			modem_status.dcd = 0;
  }
	else
	{
		modem_status.cts = 0;
		modem_status.dsr = 0;
		modem_status.ri  = 0;
		modem_status.dcd = 0;
  }

  return modem_status;
}

static void USART_IRQHandler (USART_RESOURCES *usart)
{
  uint32_t mis, event, buf;

  event = 0;
  mis   = usart->reg->MIS;

	// Transmit register empty
	if (mis & UART_MIS_TXMIS)
	{
		usart->info->xfer.tx_cnt++;
		
		// Check if all data is transmitted
		if (usart->info->xfer.tx_num == usart->info->xfer.tx_cnt)
		{
			// Disable TX interrupt
			usart->reg->IMSC &= ~UART_IMSC_TXIM;
			usart->reg->ICR |= UART_ICR_TXIC;
			
			// Clear TX busy flag
			usart->info->flags &= ~USART_FLAG_SEND_ACTIVE;
			
			event |= ARM_USART_EVENT_SEND_COMPLETE;
    }
		else
			// Write data to transmit register
			usart->reg->DR = usart->info->xfer.tx_buf[usart->info->xfer.tx_cnt];
	}

  // Receive data interrupt
	if (mis & UART_MIS_RXMIS)
	{
		// Read data from receive register into receive buffer
		buf = usart->reg->DR;
		usart->info->xfer.rx_buf[usart->info->xfer.rx_cnt] = buf & UART_DR_DATA_Msk;
			
		// Check RX line interrupt for errors
		// OverRun error
		if (buf & UART_DR_OE)
		{
			usart->info->status.rx_overflow = 1;
			event |= ARM_USART_EVENT_RX_OVERFLOW;
		}
			
		// Parity error
		if (buf & UART_DR_PE)
		{
			usart->info->status.rx_parity_error = 1;
			event |= ARM_USART_EVENT_RX_PARITY_ERROR;
		}

		// Break detected
		if (buf & UART_DR_BE)
		{
			usart->info->status.rx_break = 1;
			event |= ARM_USART_EVENT_RX_BREAK;
		}

		// Framing error
		if (buf & UART_DR_FE)
		{
			usart->info->status.rx_framing_error = 1;
			event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
		}
			
		usart->info->xfer.rx_cnt++;

		// Check if requested amount of data is received
		if (usart->info->xfer.rx_cnt == usart->info->xfer.rx_num)
		{
			// Disable receive and errors interrupt
			usart->reg->IMSC &=  ~(UART_IMSC_RXIM | UART_IMSC_FEIM |
					UART_IMSC_PEIM | UART_IMSC_BEIM | UART_IMSC_OEIM);
			usart->reg->ICR |= (UART_ICR_RXIC | UART_ICR_FEIC |
					UART_ICR_PEIC | UART_ICR_BEIC | UART_ICR_OEIC);
				
			// Clear RX busy flag and set receive transfer complete event
			usart->info->status.rx_busy = 0;
				
			event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
		}
	}

	// CTS state changed
	if ((usart->capabilities.cts) && (mis & UART_MIS_CTSMMIS))
	{
		event |= ARM_USART_EVENT_CTS;
		usart->reg->ICR |= UART_ICR_CTSMIC;
	}
			
	// DSR state changed
	if ((usart->capabilities.dsr) && (mis & UART_MIS_DSRMMIS))
	{
		event |= ARM_USART_EVENT_DSR;
		usart->reg->ICR |= UART_ICR_DSRMIC;
	}
			
	// Ring indicator
	if ((usart->capabilities.ri) && (mis & UART_MIS_RIMMIS))
	{
		event |= ARM_USART_EVENT_RI;
		usart->reg->ICR |= UART_ICR_RIMIC;
	}
			
	// DCD state changed
	if ((usart->capabilities.dcd) && (mis & UART_MIS_DCDMMIS))
	{
		event |= ARM_USART_EVENT_DCD;
		usart->reg->ICR |= UART_ICR_DCDMIC;
	}
	
  if (usart->info->cb_event && event)
    usart->info->cb_event (event);
}

// End USART Interface

#if (RTE_UART0)
// USART0 Driver Wrapper functions
static ARM_USART_CAPABILITIES USART0_GetCapabilities (void)
{
  return USART_GetCapabilities (&USART0_Resources);
}

static int32_t USART0_Initialize (ARM_USART_SignalEvent_t cb_event)
{
  return USART_Initialize (cb_event, &USART0_Resources);
}

static int32_t USART0_Uninitialize (void)
{
  return USART_Uninitialize (&USART0_Resources);
}

static int32_t USART0_PowerControl (ARM_POWER_STATE state)
{
  return USART_PowerControl (state, &USART0_Resources);
}

static int32_t USART0_Send (const void *data, uint32_t num)
{
  return USART_Send (data, num, &USART0_Resources);
}

static int32_t USART0_Receive (void *data, uint32_t num)
{
  return USART_Receive (data, num, &USART0_Resources);
}

static int32_t USART0_Transfer (const void      *data_out,
                                      void      *data_in,
                                      uint32_t   num)
{
  return USART_Transfer (data_out, data_in, num, &USART0_Resources);
}

static uint32_t USART0_GetTxCount (void)
{
  return USART_GetTxCount (&USART0_Resources);
}

static uint32_t USART0_GetRxCount (void)
{
  return USART_GetRxCount (&USART0_Resources); 
}

static int32_t USART0_Control (uint32_t control, uint32_t arg)
{
  return USART_Control (control, arg, &USART0_Resources);
}

static ARM_USART_STATUS USART0_GetStatus (void)
{
  return USART_GetStatus (&USART0_Resources);
}

static int32_t USART0_SetModemControl (ARM_USART_MODEM_CONTROL control)
{
  return USART_SetModemControl (control, &USART0_Resources);
}

static ARM_USART_MODEM_STATUS USART0_GetModemStatus (void)
{
  return USART_GetModemStatus (&USART0_Resources);
}

void UART1_IRQHandler (void)
{
  USART_IRQHandler (&USART0_Resources);
}

// USART0 Driver Control Block
ARM_DRIVER_USART Driver_USART0 =
{
	USART_GetVersion,
  USART0_GetCapabilities,
  USART0_Initialize,
  USART0_Uninitialize,
  USART0_PowerControl,
  USART0_Send, 
  USART0_Receive,
  USART0_Transfer,
  USART0_GetTxCount,
  USART0_GetRxCount,
  USART0_Control,
  USART0_GetStatus,
  USART0_SetModemControl,
	USART0_GetModemStatus
};
#endif

#if (RTE_UART1)
// USART1 Driver Wrapper functions
static ARM_USART_CAPABILITIES USART1_GetCapabilities (void)
{
  return USART_GetCapabilities (&USART1_Resources);
}

static int32_t USART1_Initialize (ARM_USART_SignalEvent_t cb_event)
{
  return USART_Initialize (cb_event, &USART1_Resources);
}

static int32_t USART1_Uninitialize (void)
{
  return USART_Uninitialize (&USART1_Resources);
}

static int32_t USART1_PowerControl (ARM_POWER_STATE state)
{
  return USART_PowerControl (state, &USART1_Resources);
}

static int32_t USART1_Send (const void *data, uint32_t num)
{
  return USART_Send (data, num, &USART1_Resources);
}

static int32_t USART1_Receive (void *data, uint32_t num)
{
  return USART_Receive (data, num, &USART1_Resources);
}

static int32_t USART1_Transfer (const void      *data_out,
                                      void      *data_in,
                                      uint32_t   num)
{
  return USART_Transfer (data_out, data_in, num, &USART1_Resources);
}

static uint32_t USART1_GetTxCount (void)
{
  return USART_GetTxCount (&USART1_Resources);
}

static uint32_t USART1_GetRxCount (void)
{
  return USART_GetRxCount (&USART1_Resources); 
}

static int32_t USART1_Control (uint32_t control, uint32_t arg)
{
  return USART_Control (control, arg, &USART1_Resources);
}

static ARM_USART_STATUS USART1_GetStatus (void)
{
  return USART_GetStatus (&USART1_Resources);
}

static int32_t USART1_SetModemControl (ARM_USART_MODEM_CONTROL control)
{
  return USART_SetModemControl (control, &USART1_Resources);
}

static ARM_USART_MODEM_STATUS USART1_GetModemStatus (void)
{
  return USART_GetModemStatus (&USART1_Resources);
}

void UART2_IRQHandler (void)
{
  USART_IRQHandler (&USART1_Resources);
}

// USART1 Driver Control Block
ARM_DRIVER_USART Driver_USART1 =
{
	USART_GetVersion,
  USART1_GetCapabilities,
  USART1_Initialize,
  USART1_Uninitialize,
  USART1_PowerControl,
  USART1_Send, 
  USART1_Receive,
  USART1_Transfer,
  USART1_GetTxCount,
  USART1_GetRxCount,
  USART1_Control,
  USART1_GetStatus,
  USART1_SetModemControl,
	USART1_GetModemStatus
};
#endif
