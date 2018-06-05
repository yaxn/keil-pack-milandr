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
 * $Date:        18. October 2015
 * $Revision:    V1.00
 *
 * Driver:       Driver_SPI0, Driver_SPI1
 * Configured:   via RTE_Device.h configuration file
 * Project:      SPI (SSP used for SPI) Driver for Milandr MDR1986VE9X
 * -----------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting               Value     SPI Interface
 *   ---------------------               -----     -------------
 *   Connect to hardware via Driver_SPI# = 0       MDR_SSP1
 *   Connect to hardware via Driver_SPI# = 1       MDR_SSP2
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    - Initial CMSIS Driver API V1.00 release
 */

#include "MDR1986VE9X_SPI.h"

#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 00) /* driver version */

#if ((defined(RTE_Drivers_SPI0) || defined(RTE_Drivers_SPI1)) && (!RTE_SSP0) && (!RTE_SSP1))
	#error "SSP not configured in RTE_Device.h!"
#endif

#if (RTE_SSP0)
static SSP_INFO          SSP0_Info = { 0 };
static SSP_TRANSFER_INFO SSP0_Xfer;

static SSP_RESOURCES     SSP0_Resources =
{
    MDR_SSP1,
  { RTE_SSP0_SSEL_PIN_EN,
    RTE_SSP0_SSEL_PORT,
    RTE_SSP0_SSEL_BIT,
    RTE_SSP0_SSEL_FUNC,
    RTE_SSP0_SCK_PORT,
    RTE_SSP0_SCK_BIT,
    RTE_SSP0_SCK_FUNC,
    RTE_SSP0_MISO_PORT,
    RTE_SSP0_MISO_BIT,
    RTE_SSP0_MISO_FUNC,
    RTE_SSP0_MOSI_PORT,
    RTE_SSP0_MOSI_BIT,
    RTE_SSP0_MOSI_FUNC },
  { RST_CLK_SSP_CLOCK_SSP1_CLK_EN | ((RTE_SSP0_BRG << RST_CLK_SSP_CLOCK_SSP1_BRG_Pos) & RST_CLK_SSP_CLOCK_SSP1_BRG_Msk),
    &(MDR_RST_CLK->SSP_CLOCK), RST_CLK_PER_CLOCK_PCLK_EN_SPI1, (1 << RTE_SSP0_BRG)},
    SSP1_IRQn,
		&SSP0_Info,
		&SSP0_Xfer
};
#endif

#if (RTE_SSP1)
static SSP_INFO          SSP1_Info = { 0 };
static SSP_TRANSFER_INFO SSP1_Xfer;
static SSP_RESOURCES     SSP1_Resources =
{
		MDR_SSP2,
  { RTE_SSP1_SSEL_PIN_EN,
    RTE_SSP1_SSEL_PORT,
    RTE_SSP1_SSEL_BIT,
    RTE_SSP1_SSEL_FUNC,
    RTE_SSP1_SCK_PORT,
    RTE_SSP1_SCK_BIT,
    RTE_SSP1_SCK_FUNC,
    RTE_SSP1_MISO_PORT,
    RTE_SSP1_MISO_BIT,
    RTE_SSP1_MISO_FUNC,
    RTE_SSP1_MOSI_PORT,
    RTE_SSP1_MOSI_BIT,
    RTE_SSP1_MOSI_FUNC },
  { RST_CLK_SSP_CLOCK_SSP2_CLK_EN | ((RTE_SSP1_BRG << RST_CLK_SSP_CLOCK_SSP2_BRG_Pos) & RST_CLK_SSP_CLOCK_SSP2_BRG_Msk),
    &(MDR_RST_CLK->SSP_CLOCK), RST_CLK_PER_CLOCK_PCLK_EN_SPI2, (1 << RTE_SSP1_BRG)},
		SSP2_IRQn,
		&SSP1_Info,
		&SSP1_Xfer
};
#endif

void SSP_PinConfigure (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t Func, bool PinState)
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

void SSP_PinUnconfigure (MDR_PORT_TypeDef* Port, uint8_t Pin)
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

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
	ARM_SPI_API_VERSION,
	ARM_SPI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities =
{
	0, /* Simplex Mode (Master and Slave) */
	1, /* TI Synchronous Serial Interface */
	1, /* Microwire Interface */
	0  /* Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT */
};

//
//  Functions
//

static ARM_DRIVER_VERSION SSP_GetVersion (void)
{
	return DriverVersion;
}

static ARM_SPI_CAPABILITIES SSP_GetCapabilities (void)
{
	return DriverCapabilities;
}

static int32_t SSPx_Initialize (ARM_SPI_SignalEvent_t cb_event, SSP_RESOURCES *ssp)
{
	if (ssp->info->state & SSP_INITIALIZED)
		return ARM_DRIVER_OK;
	
  if (ssp->info->state & SSP_POWERED)
		return ARM_DRIVER_ERROR;

  // Initialize SPI Run-Time Resources
  ssp->info->cb_event          = cb_event;
  ssp->info->status.busy       = 0;
  ssp->info->status.data_lost  = 0;
  ssp->info->status.mode_fault = 0;

  // Clear transfer information
  memset (ssp->xfer, 0, sizeof(SSP_TRANSFER_INFO));

  // Configure pins
	ClkEnPort (ssp->pin.ssel_port);
	ClkEnPort (ssp->pin.sck_port);
	ClkEnPort (ssp->pin.mosi_port);
	ClkEnPort (ssp->pin.miso_port);
	
	SSP_PinUnconfigure (ssp->pin.ssel_port, ssp->pin.ssel_bit);
	SSP_PinConfigure (ssp->pin.sck_port, ssp->pin.sck_bit, ssp->pin.sck_func, PORT_PIN_OUT);
	SSP_PinConfigure (ssp->pin.mosi_port, ssp->pin.mosi_bit, ssp->pin.mosi_func, PORT_PIN_OUT);
	SSP_PinConfigure (ssp->pin.miso_port, ssp->pin.miso_bit, ssp->pin.miso_func, PORT_PIN_IN);

  ssp->info->state = SSP_INITIALIZED;   // SSP is initialized

  return ARM_DRIVER_OK;
}

static int32_t SSPx_Uninitialize (SSP_RESOURCES *ssp)
{
	if (!(ssp->info->state & SSP_INITIALIZED))
		return ARM_DRIVER_OK;
	
  if (ssp->info->state & SSP_POWERED)
		return ARM_DRIVER_ERROR;

  // Unconfigure pins
	if (ssp->pin.ssel_en == 1)
		SSP_PinUnconfigure (ssp->pin.ssel_port, ssp->pin.ssel_bit);
	
	SSP_PinUnconfigure (ssp->pin.sck_port, ssp->pin.sck_bit);
	SSP_PinUnconfigure (ssp->pin.mosi_port, ssp->pin.mosi_bit);
	SSP_PinUnconfigure (ssp->pin.miso_port, ssp->pin.miso_bit);

  ssp->info->state = 0;                 // SSP is uninitialized

  return ARM_DRIVER_OK;
}

static int32_t SSPx_PowerControl (ARM_POWER_STATE state, SSP_RESOURCES *ssp)
{
	if (!(ssp->info->state & SSP_INITIALIZED))
		return ARM_DRIVER_ERROR;
	
  if (ssp->info->status.busy)
		return ARM_DRIVER_ERROR_BUSY;
	
  switch (state)
  {
		case ARM_POWER_OFF:
			if (ssp->info->state & SSP_POWERED)
			{
        NVIC_DisableIRQ (ssp->irq_num);   // Disable SSP IRQ in NVIC
 
        ssp->reg->CR1 &= ~SSP_CR1_SSE;   // Disable SSP

        // Deactivate SSP peripheral clock
        *(ssp->clk.peri_cfg) &= ~(ssp->clk.peri_cfg_val);
				MDR_RST_CLK->PER_CLOCK &= ~(ssp->clk.on_msk);
				
				ssp->info->state &= ~SSP_POWERED; // SSP is not powered
      }
			
			break;
		case ARM_POWER_FULL:
			if (!(ssp->info->state & SSP_POWERED))
			{	
        // Activate SSP peripheral clock
				MDR_RST_CLK->PER_CLOCK |= (ssp->clk.on_msk);
        *(ssp->clk.peri_cfg) |= (ssp->clk.peri_cfg_val);

        // Reset SSP peripheral
				ssp->reg->CR0 = 0;
				ssp->reg->CR1 = 0;
				ssp->reg->DR = 0;
				ssp->reg->CPSR = 0;
				ssp->reg->DMACR = 0;

        ssp->reg->IMSC = 0;																// Disable SSP interrupts
        ssp->reg->ICR  = SSP_ICR_RORIC | SSP_ICR_RTIC;		// Clear SSP interrupts

        NVIC_ClearPendingIRQ (ssp->irq_num);
        NVIC_EnableIRQ (ssp->irq_num);    // Enable SSP IRQ in NVIC
				
				ssp->info->state |= SSP_POWERED; // SSP is powered
      }
			
			break;
		default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	
	return ARM_DRIVER_OK;
}

static int32_t SSPx_Send (const void *data, uint32_t num, SSP_RESOURCES *ssp)
{
	if ((data == NULL) || (num == 0))
		return ARM_DRIVER_ERROR_PARAMETER;
	
  if (!(ssp->info->state & SSP_CONFIGURED))
		return ARM_DRIVER_ERROR;
	
  if (ssp->info->status.busy)
		return ARM_DRIVER_ERROR_BUSY;
	
  ssp->info->status.busy       = 1;
  ssp->info->status.data_lost  = 0;
  ssp->info->status.mode_fault = 0;

  ssp->xfer->rx_buf = NULL;
  ssp->xfer->tx_buf = (uint8_t *)data;

  ssp->xfer->num    = num;
  ssp->xfer->rx_cnt = 0;
  ssp->xfer->tx_cnt = 0;
	
	ssp->reg->IMSC = SSP_IMSC_TXIM | SSP_IMSC_RXIM | SSP_IMSC_RTIM | SSP_IMSC_RORIM;
	
	return ARM_DRIVER_OK;
}

static int32_t SSPx_Receive (void *data, uint32_t num, SSP_RESOURCES *ssp)
{
  if ((data == NULL) || (num == 0))
		return ARM_DRIVER_ERROR_PARAMETER;
	
  if (!(ssp->info->state & SSP_CONFIGURED))
		return ARM_DRIVER_ERROR;
	
  if (ssp->info->status.busy)
		return ARM_DRIVER_ERROR_BUSY;
	
  ssp->info->status.busy       = 1;
  ssp->info->status.data_lost  = 0;
  ssp->info->status.mode_fault = 0;

  ssp->xfer->rx_buf = (uint8_t *)data;
  ssp->xfer->tx_buf = NULL;

  ssp->xfer->num    = num;
  ssp->xfer->rx_cnt = 0;
  ssp->xfer->tx_cnt = 0;
	
	ssp->reg->IMSC = SSP_IMSC_TXIM | SSP_IMSC_RXIM | SSP_IMSC_RTIM | SSP_IMSC_RORIM;
	
	return ARM_DRIVER_OK;
}

static int32_t SSPx_Transfer (const void *data_out, void *data_in, uint32_t num, SSP_RESOURCES *ssp)
{
	if ((data_out == NULL) || (data_in == NULL) || (num == 0))
		return ARM_DRIVER_ERROR_PARAMETER;
	
  if (!(ssp->info->state & SSP_CONFIGURED))
		return ARM_DRIVER_ERROR;
	
  if (ssp->info->status.busy)
		return ARM_DRIVER_ERROR_BUSY;
	
  ssp->info->status.busy       = 1;
  ssp->info->status.data_lost  = 0;
  ssp->info->status.mode_fault = 0;

  ssp->xfer->rx_buf = (uint8_t *)data_in;
  ssp->xfer->tx_buf = (uint8_t *)data_out;

  ssp->xfer->num    = num;
  ssp->xfer->rx_cnt = 0;
  ssp->xfer->tx_cnt = 0;
	
	ssp->reg->IMSC = SSP_IMSC_TXIM | SSP_IMSC_RXIM | SSP_IMSC_RTIM | SSP_IMSC_RORIM;
	
	return ARM_DRIVER_OK;
}

static uint32_t SSPx_GetDataCount (SSP_RESOURCES *ssp)
{
	if (!(ssp->info->state & SSP_CONFIGURED))
		return 0;
	
	if (ssp->xfer->rx_buf == NULL)	// If send operation
    return ssp->xfer->tx_cnt;
	else														// If receive or transfer operation
    return ssp->xfer->rx_cnt;
}

static int32_t SSPx_Control (uint32_t control, uint32_t arg, SSP_RESOURCES *ssp)
{
	uint32_t cpsr, scr, clk, data_bits;
  uint32_t best_cpsr = 2, best_scr = 0;
	
	if (!(ssp->info->state & SSP_POWERED))
		return ARM_DRIVER_ERROR;
	
	if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER)
	{
		ssp->reg->CR1 &= ~SSP_CR1_SSE;         // Disable SSP
    memset (ssp->xfer, 0, sizeof(SSP_TRANSFER_INFO));
    ssp->reg->IMSC = 0;                    // Disable interrupts
    ssp->info->status.busy = 0;
    ssp->reg->CR1 |= SSP_CR1_SSE;          // Enable  SSP
		
    return ARM_DRIVER_OK;
  }
	
	if (ssp->info->status.busy)
		return ARM_DRIVER_ERROR_BUSY;
	
	switch (control & ARM_SPI_CONTROL_Msk)
	{
		default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_SPI_MODE_INACTIVE:             // SPI Inactive
			ssp->reg->CR1    &= ~SSP_CR1_SSE;     // Disable SSP
      ssp->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      ssp->info->mode  |=  ARM_SPI_MODE_INACTIVE;
      ssp->info->state &= ~SSP_CONFIGURED;
		
			return ARM_DRIVER_OK;
    case ARM_SPI_MODE_MASTER:               // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
			ssp->reg->CR1    &= ~SSP_CR1_SSE;     // Disable SSP
      ssp->reg->CR1    &= ~SSP_CR1_MS;      // Set master mode
      ssp->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      ssp->info->mode  |=  ARM_SPI_MODE_MASTER;
      ssp->info->state |=  SSP_CONFIGURED;
      ssp->reg->CR1    |=  SSP_CR1_SSE;     // Enable  SSP
		
      goto set_speed;
    case ARM_SPI_MODE_SLAVE:                // SPI Slave  (Output on MISO, Input on MOSI)
			ssp->reg->CR1    &= ~SSP_CR1_SSE;     // Disable SSP
      ssp->reg->CR1    |=  SSP_CR1_MS;      // Set slave mode
      ssp->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      ssp->info->mode  |=  ARM_SPI_MODE_SLAVE;
      ssp->info->state |=  SSP_CONFIGURED;
      ssp->reg->CR1    |=  SSP_CR1_SSE;     // Enable  SSP
		
			break;
    case ARM_SPI_MODE_MASTER_SIMPLEX:       // SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
    case ARM_SPI_MODE_SLAVE_SIMPLEX:        // SPI Slave  (Output/Input on MISO)
        return ARM_SPI_ERROR_MODE;

    case ARM_SPI_SET_BUS_SPEED:             // Set Bus Speed in bps; arg = value
set_speed:
      clk = RTE_HCLK / ssp->clk.div;
		
      for (cpsr = 2; cpsr < 255; cpsr += 2)	// Loop through clock prescaler
        for (scr = 0; scr < 256; scr++)			// Loop through bit prescaler
          if (clk == (arg * cpsr * (scr+1)))
					{
            best_cpsr = cpsr;
            best_scr  = scr;
						
            goto found_best;
          }
					else
						if ((((clk % (best_cpsr * (best_scr+1))) * 1024) / (best_cpsr * (best_scr+1))) >
							(((clk % (cpsr * (scr+1))) * 1024) / (cpsr * (scr+1))))
						{
							best_cpsr = cpsr;
							best_scr  = scr;
						}
found_best:
      ssp->reg->CPSR =  best_cpsr & SSP_CPSR_CPSDVSR_Msk;
      ssp->reg->CR0 &= ~SSP_CR0_SCR_Msk;
      ssp->reg->CR0 |= ((scr << SSP_CR0_SCR_Pos) & SSP_CR0_SCR_Msk);
						
      if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_SET_BUS_SPEED)
				return ARM_DRIVER_OK;
			
			break;
    case ARM_SPI_GET_BUS_SPEED:             // Get Bus Speed in bps
			return (RTE_HCLK / ssp->clk.div /
				((ssp->reg->CPSR & SSP_CPSR_CPSDVSR_Msk) * (((ssp->reg->CR0 & SSP_CR0_SCR_Msk) >> SSP_CR0_SCR_Pos) + 1)));
			
    case ARM_SPI_SET_DEFAULT_TX_VALUE:      // Set default Transmit value; arg = value
			ssp->xfer->def_val = (uint16_t)(arg & 0xFFFF);
		
      return ARM_DRIVER_OK;
    case ARM_SPI_CONTROL_SS:                // Control Slave Select; arg = 0:inactive, 1:active
			if (((ssp->info->mode & ARM_SPI_CONTROL_Msk) != ARM_SPI_MODE_MASTER) ||
					((ssp->info->mode & ARM_SPI_SS_MASTER_MODE_Msk) != ARM_SPI_SS_MASTER_SW))
				return ARM_DRIVER_ERROR;
						
      if (ssp->pin.ssel_en == 0)
        return ARM_DRIVER_ERROR;
				
      if (arg == ARM_SPI_SS_INACTIVE)
        PortPinWrite (ssp->pin.ssel_port, ssp->pin.ssel_bit, PORT_PIN_ON);
      else
        PortPinWrite (ssp->pin.ssel_port, ssp->pin.ssel_bit, PORT_PIN_OFF);
			
			return ARM_DRIVER_OK;
	}
	
	if ((ssp->info->mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER)
    switch (control & ARM_SPI_SS_MASTER_MODE_Msk)
		{
      case ARM_SPI_SS_MASTER_UNUSED:        // SPI Slave Select when Master: Not used (default)
        if (ssp->pin.ssel_en == 1)
					SSP_PinUnconfigure (ssp->pin.ssel_port, ssp->pin.ssel_bit);
				
        ssp->info->mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        ssp->info->mode |= ARM_SPI_SS_MASTER_UNUSED;
				
        break;
      case ARM_SPI_SS_MASTER_HW_INPUT:      // SPI Slave Select when Master: Hardware monitored Input
        ssp->info->mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			
        return ARM_SPI_ERROR_SS_MODE;
      case ARM_SPI_SS_MASTER_SW:            // SPI Slave Select when Master: Software controlled
        ssp->info->mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			
        if (ssp->pin.ssel_en == 1)
				{
					PortPinConfigure (ssp->pin.ssel_port, ssp->pin.ssel_bit, PORT_PIN_OUT);
					PortPinWrite (ssp->pin.ssel_port, ssp->pin.ssel_bit, PORT_PIN_ON);
					
          ssp->info->mode |= ARM_SPI_SS_MASTER_SW;
        }
				else
					return ARM_SPI_ERROR_SS_MODE;
				
        break;
      case ARM_SPI_SS_MASTER_HW_OUTPUT:     // SPI Slave Select when Master: Hardware controlled Output
        ssp->info->mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
			
				if (ssp->pin.ssel_en == 1)
				{
          SSP_PinConfigure (ssp->pin.ssel_port, ssp->pin.ssel_bit, ssp->pin.ssel_func, PORT_PIN_OUT);
          ssp->info->mode |= ARM_SPI_SS_MASTER_HW_OUTPUT;
        }
				else
					return ARM_SPI_ERROR_SS_MODE;
    }
		
	if ((ssp->info->mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_SLAVE)
    switch (control & ARM_SPI_SS_SLAVE_MODE_Msk)
		{
      case ARM_SPI_SS_SLAVE_HW:             // SPI Slave Select when Slave: Hardware monitored (default)
        ssp->info->mode &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
			
        if (ssp->pin.ssel_en == 1)
				{
          SSP_PinConfigure (ssp->pin.ssel_port, ssp->pin.ssel_bit, ssp->pin.ssel_func, PORT_PIN_IN);
          ssp->info->mode |= ARM_SPI_SS_SLAVE_HW;
        }
				else
          return ARM_SPI_ERROR_SS_MODE;
					
        break;
      case ARM_SPI_SS_SLAVE_SW:             // SPI Slave Select when Slave: Software controlled
        ssp->info->mode &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
			
        return ARM_SPI_ERROR_SS_MODE;
    }
		
	// Configure Frame Format
  switch (control & ARM_SPI_FRAME_FORMAT_Msk)
	{
    case ARM_SPI_CPOL0_CPHA0:
      ssp->reg->CR0 &= ~SSP_CR0_FRF_Msk;
      ssp->reg->CR0 &= ~(SSP_CR0_SPO | SSP_CR0_SPH);
		
      break;
    case ARM_SPI_CPOL0_CPHA1:
      ssp->reg->CR0 &= ~SSP_CR0_FRF_Msk;
      ssp->reg->CR0 &= ~SSP_CR0_SPO;
      ssp->reg->CR0 |=	SSP_CR0_SPH;
		
      break;
    case ARM_SPI_CPOL1_CPHA0:
      ssp->reg->CR0 &= ~SSP_CR0_FRF_Msk;
      ssp->reg->CR0 |=	SSP_CR0_SPO;
      ssp->reg->CR0 &= ~SSP_CR0_SPH;
		
      break;
    case ARM_SPI_CPOL1_CPHA1:
      ssp->reg->CR0 &= ~SSP_CR0_FRF_Msk;
      ssp->reg->CR0 |= (SSP_CR0_SPO | SSP_CR0_SPH);
		
      break;
    case ARM_SPI_TI_SSI:
      ssp->reg->CR0 &= ~SSP_CR0_FRF_Msk;
			ssp->reg->CR0 |= (SSP_CR0_FRF_SSI_TI << SSP_CR0_FRF_Pos);
		
      break;
    case ARM_SPI_MICROWIRE:
      ssp->reg->CR0 &= ~SSP_CR0_FRF_Msk;
			ssp->reg->CR0 |= (SSP_CR0_FRF_MW_NS << SSP_CR0_FRF_Pos);
		
      break;
    default:
      return ARM_SPI_ERROR_FRAME_FORMAT;
  }

  // Configure Number of Data Bits
  data_bits = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);
	
  if ((data_bits >= 4) && (data_bits <= 16))
	{
		ssp->reg->CR0 &= ~SSP_CR0_DSS_Msk;
		ssp->reg->CR0 |= ((data_bits-1) << SSP_CR0_DSS_Pos);
	}
	else
		return ARM_SPI_ERROR_DATA_BITS;

  // Configure Bit Order
  if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB)
		return ARM_SPI_ERROR_BIT_ORDER;

  return ARM_DRIVER_OK;
}

static ARM_SPI_STATUS SSPx_GetStatus (SSP_RESOURCES *ssp)
{
	return (ssp->info->status);
}

static void SSPx_IRQHandler (SSP_RESOURCES *ssp)
{
  uint16_t data;
  uint32_t mis;

  mis = ssp->reg->MIS;
  ssp->reg->ICR = mis & (SSP_ICR_RORIC | SSP_ICR_RTIC);
																											// Handle transfer
  if ((ssp->reg->SR & SSP_SR_TNF) && (ssp->xfer->num > ssp->xfer->tx_cnt))
	{
    if (ssp->xfer->tx_buf)
		{																									// If data available
      data = *(ssp->xfer->tx_buf++);
			
      if ((ssp->reg->CR0 & SSP_CR0_DSS_Msk) > 7)			// If 9..16-bit data frame format
				data |= *(ssp->xfer->tx_buf++) << 8;
    }
		else																							// If default data send
      data = ssp->xfer->def_val;
		
    ssp->reg->DR = data;															// Activate send
    ssp->xfer->tx_cnt++;
  }

  while (ssp->reg->SR & SSP_SR_RNE)
	{
    data = ssp->reg->DR;															// Read data
		
    if (ssp->xfer->num > ssp->xfer->rx_cnt)
		{
      if (ssp->xfer->rx_buf)
			{
        *(ssp->xfer->rx_buf++) = (uint8_t)data;				// Put data into buffer
				
        if ((ssp->reg->CR0 & SSP_CR0_DSS_Msk) > 7)		// If 9..16-bit data frame format
          *(ssp->xfer->rx_buf++) = (uint8_t)(data >> 8);
      }
			
      ssp->xfer->rx_cnt++;
			
      if (ssp->xfer->rx_cnt == ssp->xfer->num)
			{																								// If all data received
        ssp->reg->IMSC &= ~(SSP_IMSC_TXIM | SSP_IMSC_RXIM | SSP_IMSC_RTIM | SSP_IMSC_RORIM);
        ssp->info->status.busy = 0;
				
        if (ssp->info->cb_event)
					ssp->info->cb_event (ARM_SPI_EVENT_TRANSFER_COMPLETE);
				
				break;
      }
    }
  }

  if (mis & SSP_MIS_RORMIS)
	{																										// Handle errors
																											// Overrun flag is set
    ssp->info->status.data_lost = 1;
		
    if (ssp->info->cb_event)
			ssp->info->cb_event (ARM_SPI_EVENT_DATA_LOST);
  }
}

// End SPI Interface

#if (RTE_SSP0)
static int32_t        SSP0_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SSPx_Initialize   (pSignalEvent, &SSP0_Resources); }
static int32_t        SSP0_Uninitialize        (void)                                              { return SSPx_Uninitialize (&SSP0_Resources); }
static int32_t        SSP0_PowerControl        (ARM_POWER_STATE state)                             { return SSPx_PowerControl (state, &SSP0_Resources); }
static int32_t        SSP0_Send                (const void *data, uint32_t num)                    { return SSPx_Send         (data, num, &SSP0_Resources); }
static int32_t        SSP0_Receive             (void *data, uint32_t num)                          { return SSPx_Receive      (data, num, &SSP0_Resources); }
static int32_t        SSP0_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SSPx_Transfer     (data_out, data_in, num, &SSP0_Resources); }
static uint32_t       SSP0_GetDataCount        (void)                                              { return SSPx_GetDataCount (&SSP0_Resources); }
static int32_t        SSP0_Control             (uint32_t control, uint32_t arg)                    { return SSPx_Control      (control, arg, &SSP0_Resources); }
static ARM_SPI_STATUS SSP0_GetStatus           (void)                                              { return SSPx_GetStatus    (&SSP0_Resources); }
void									SSP1_IRQHandler          (void)                                              { SSPx_IRQHandler          (&SSP0_Resources); }

// SPI0 Driver Control Block
ARM_DRIVER_SPI Driver_SPI0 =
{
  SSP_GetVersion,
  SSP_GetCapabilities,
  SSP0_Initialize,
  SSP0_Uninitialize,
  SSP0_PowerControl,
  SSP0_Send,
  SSP0_Receive,
  SSP0_Transfer,
  SSP0_GetDataCount,
  SSP0_Control,
  SSP0_GetStatus
};
#endif


#if (RTE_SSP1)
static int32_t        SSP1_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SSPx_Initialize   (pSignalEvent, &SSP1_Resources); }
static int32_t        SSP1_Uninitialize        (void)                                              { return SSPx_Uninitialize (&SSP1_Resources); }
static int32_t        SSP1_PowerControl        (ARM_POWER_STATE state)                             { return SSPx_PowerControl (state, &SSP1_Resources); }
static int32_t        SSP1_Send                (const void *data, uint32_t num)                    { return SSPx_Send         (data, num, &SSP1_Resources); }
static int32_t        SSP1_Receive             (void *data, uint32_t num)                          { return SSPx_Receive      (data, num, &SSP1_Resources); }
static int32_t        SSP1_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SSPx_Transfer     (data_out, data_in, num, &SSP1_Resources); }
static uint32_t       SSP1_GetDataCount        (void)                                              { return SSPx_GetDataCount (&SSP1_Resources); }
static int32_t        SSP1_Control             (uint32_t control, uint32_t arg)                    { return SSPx_Control      (control, arg, &SSP1_Resources); }
static ARM_SPI_STATUS SSP1_GetStatus           (void)                                              { return SSPx_GetStatus    (&SSP1_Resources); }
void									SSP2_IRQHandler          (void)                                              { SSPx_IRQHandler          (&SSP1_Resources); }

// SPI1 Driver Control Block
ARM_DRIVER_SPI Driver_SPI1 =
{
  SSP_GetVersion,
  SSP_GetCapabilities,
  SSP1_Initialize,
  SSP1_Uninitialize,
  SSP1_PowerControl,
  SSP1_Send,
  SSP1_Receive,
  SSP1_Transfer,
  SSP1_GetDataCount,
  SSP1_Control,
  SSP1_GetStatus
};
#endif
