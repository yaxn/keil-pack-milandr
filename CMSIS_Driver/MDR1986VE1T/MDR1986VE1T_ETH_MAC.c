#include "MDR1986VE1T_ETH_MAC.h"

#define ARM_ETH_MAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

#if (defined(RTE_Drivers_ETH_MAC0) && !RTE_ETH_MAC0)
	#error "Ethernet Media Access Control 0 not configured in RTE_Device.h!"
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION MAC_DriverVersion =
{
	ARM_ETH_MAC_API_VERSION,
	ARM_ETH_MAC_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES MAC_DriverCapabilities =
{
    0, /* 1 = IPv4 header checksum verified on receive */
    0, /* 1 = IPv6 checksum verification supported on receive */
    0, /* 1 = UDP payload checksum verified on receive */
    0, /* 1 = TCP payload checksum verified on receive */
    0, /* 1 = ICMP payload checksum verified on receive */
    0, /* 1 = IPv4 header checksum generated on transmit */
    0, /* 1 = IPv6 checksum generation supported on transmit */
    0, /* 1 = UDP payload checksum generated on transmit */
    0, /* 1 = TCP payload checksum generated on transmit */
    0, /* 1 = ICMP payload checksum generated on transmit */
    0, /* Ethernet Media Interface type */
    0, /* 1 = driver provides initial valid MAC address */
    1, /* 1 = callback event \ref ARM_ETH_MAC_EVENT_RX_FRAME generated */
    1, /* 1 = callback event \ref ARM_ETH_MAC_EVENT_TX_FRAME generated */
    0, /* 1 = wakeup event \ref ARM_ETH_MAC_EVENT_WAKEUP generated */
    0  /* 1 = Precision Timer supported */
};

static EMAC_CTRL emac_control = {0};

#define emac (emac_control)

//
//  Functions
//

static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val)
{
  uint32_t n;
	uint8_t tmp;
	
	/* Reverse bits in val */
	tmp = val & 0xFF;

	tmp = ((tmp & 0x55) << 1) | ((tmp >> 1) & 0x55);
	tmp = ((tmp & 0x33) << 2) | ((tmp >> 2) & 0x33);
	tmp = ((tmp & 0x0F) << 4) | ((tmp >> 4) & 0x0F);
	
	val = (val & ~0xFF) | tmp;
	
	crc32 ^= val;
	
  for (n = 8; n; n--)
    if (crc32 & 0x80000000)
		{
      crc32 <<= 1;
      crc32  ^= 0x04C11DB7;
    }
		else
      crc32 <<= 1;
	
  return crc32;
}

static uint32_t crc32_data (const uint8_t *data, uint32_t len)
{
  uint32_t crc;

  for (crc = 0xFFFFFFFF; len; len--)
    crc = crc32_8bit_rev (crc, *data++);
	
  return crc;
}

void FlushFIFO (void)
{
	uint32_t i, *pBuf;
	
	pBuf = (uint32_t *) MDR_ETHERNET1_BUF_BASE;
	
	for (i=0; i<(MDR_ETHERNET1_BUF_SIZE/4); i++)
		*pBuf++ = 0;
}

void EMAC_Reset (void)
{
	MDR_ETHERNET1->ETH_MAC_T = 0x78AB;
	MDR_ETHERNET1->ETH_MAC_M = 0x3456;
	MDR_ETHERNET1->ETH_MAC_H = 0x0012;
	
	MDR_ETHERNET1->ETH_HASH0 = 0x0000;
	MDR_ETHERNET1->ETH_HASH1 = 0x0000;
	MDR_ETHERNET1->ETH_HASH2 = 0x0000;
	MDR_ETHERNET1->ETH_HASH3 = 0x8000;
	
	MDR_ETHERNET1->ETH_IPG				= 0x0060;
	MDR_ETHERNET1->ETH_PSC				= 0x0050;
	MDR_ETHERNET1->ETH_BAG				= 0x0200;
	MDR_ETHERNET1->ETH_JitterWnd	= 0x0005;
	
	MDR_ETHERNET1->ETH_R_CFG =
		(0 << ETH_R_CFG_MCA_EN_Pos) |
		(0 << ETH_R_CFG_BCA_EN_Pos) |
		(1 << ETH_R_CFG_UCA_EN_Pos) |
		(1 << ETH_R_CFG_AC_EN_Pos) |
		(0 << ETH_R_CFG_EF_EN_Pos) |
		(0 << ETH_R_CFG_CF_EN_Pos) |
		(0 << ETH_R_CFG_LF_EN_Pos) |
		(0 << ETH_R_CFG_SF_EN_Pos) |
		(5 << ETH_R_CFG_EVNT_MODE_Pos) |
		(0 << ETH_R_CFG_MSB1st_Pos) |
		(0 << ETH_R_CFG_BE_Pos) |
		(0 << ETH_R_CFG_EN_Pos);
		
	MDR_ETHERNET1->ETH_X_CFG =
		(10 << ETH_X_CFG_RtryCnt_Pos) |
		(1 << ETH_X_CFG_IPG_EN_Pos) |
		(1 << ETH_X_CFG_CRC_EN_Pos)|
		(1 << ETH_X_CFG_PRE_EN_Pos) |
		(1 << ETH_X_CFG_PAD_EN_Pos) |
		(5 << ETH_X_CFG_EVNT_MODE_Pos) |
		(0 << ETH_X_CFG_MSB1st_Pos) |
		(0 << ETH_X_CFG_BE_Pos) |
		(0 << ETH_X_CFG_EN_Pos);
		
	MDR_ETHERNET1->ETH_G_CFGl =
		(128 << ETH_G_CFGl_ColWnd_Pos) |
		(0 << ETH_G_CFGl_PAUSE_EN_Pos) |
		(0 << ETH_G_CFGl_DTRM_EN_Pos) |
		(0 << ETH_G_CFGl_HD_EN_Pos) |
		(0 << ETH_G_CFGl_EXT_EN_Pos) |
		(ETH_G_CFGl_BUFF_MODE_LIN << ETH_G_CFGl_BUFF_MODE_Pos) |
		(0 << ETH_G_CFGl_RCLR_EN_Pos);
		
	MDR_ETHERNET1->ETH_G_CFGh =
		(0 << ETH_G_CFGh_XRST_Pos) |
		(0 << ETH_G_CFGh_RRST_Pos) |
		(0 << ETH_G_CFGh_DLB_Pos) |
		(1 << ETH_G_CFGh_DBG_RF_EN_Pos) |
		(1 << ETH_G_CFGh_DBG_XF_EN_Pos) |
		(0 << ETH_G_CFGh_DBG_MODE_Pos);
	
	MDR_ETHERNET1->ETH_IMR				= 0x0000;
	MDR_ETHERNET1->ETH_IFR				= 0xDFFF;
	
	MDR_ETHERNET1->ETH_Dilimiter 	= 4096;
	MDR_ETHERNET1->ETH_R_Head			= 0;
	MDR_ETHERNET1->ETH_X_Tail			= 4096;
	MDR_ETHERNET1->ETH_R_Tail			= 0;
	MDR_ETHERNET1->ETH_X_Head			= 4096;
	
	FlushFIFO ();
	MDR_ETHERNET1->ETH_G_CFGh &= ~(ETH_G_CFGh_RRST | ETH_G_CFGh_XRST);
}

static ARM_DRIVER_VERSION ARM_ETH_MAC_GetVersion (void)
{
	return MAC_DriverVersion;
}

static ARM_ETH_MAC_CAPABILITIES ARM_ETH_MAC_GetCapabilities (void)
{
	return MAC_DriverCapabilities;
}

static int32_t ARM_ETH_MAC_Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
{
	if (emac.flags & EMAC_FLAG_POWER)
    return ARM_DRIVER_ERROR;

  if (emac.flags & EMAC_FLAG_INIT)
		return ARM_DRIVER_OK;
	
  emac.flags = EMAC_FLAG_INIT;

  /* Register driver callback function */
  emac.cb_event = cb_event;

  return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_Uninitialize (void)
{
	if (!(emac.flags & EMAC_FLAG_INIT))
    return ARM_DRIVER_OK;

  if (emac.flags & EMAC_FLAG_POWER)
    return ARM_DRIVER_ERROR;
	
  emac.flags = 0;

  return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_PowerControl (ARM_POWER_STATE state)
{
	if (!(emac.flags & EMAC_FLAG_INIT))
    return ARM_DRIVER_ERROR;
	
  switch (state)
  {
    case ARM_POWER_OFF:
			if (!(emac.flags & EMAC_FLAG_POWER))
				break;
			
			emac.flags = EMAC_FLAG_INIT;
			
			/* Disable Rx, Tx and flush Rx, Tx queue */
			MDR_ETHERNET1->ETH_R_CFG &= ~ETH_R_CFG_EN;
			MDR_ETHERNET1->ETH_X_CFG &= ~ETH_X_CFG_EN;
			MDR_ETHERNET1->ETH_G_CFGh |= (ETH_G_CFGh_RRST | ETH_G_CFGh_XRST);
			FlushFIFO ();
			MDR_ETHERNET1->ETH_G_CFGh &= ~(ETH_G_CFGh_RRST | ETH_G_CFGh_XRST);
			
			/* Disable EMAC interrupts */
      NVIC_DisableIRQ (ETHERNET_IRQn);
			MDR_ETHERNET1->ETH_IMR = 0x0000;
			MDR_ETHERNET1->ETH_IFR = 0xDFFF;
			
			/* Disable EMAC peripheral clock */
			MDR_RST_CLK->ETH_CLOCK &= ~RST_CLK_ETH_CLOCK_ETH_CLK_EN;
		
			break;
    case ARM_POWER_LOW:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_POWER_FULL:
			if (emac.flags & EMAC_FLAG_POWER)
        break;
			
#ifdef RTE_ETH0_LINK_LED
			ClkEnPort (RTE_ETH0_LINK_LED_PORT);
			PortPinConfigure (RTE_ETH0_LINK_LED_PORT, RTE_ETH0_LINK_LED_PIN, PORT_PIN_OUT);
#endif
			
#ifdef RTE_ETH0_REC_LED
			ClkEnPort (RTE_ETH0_REC_LED_PORT);
			PortPinConfigure (RTE_ETH0_REC_LED_PORT, RTE_ETH0_REC_LED_PIN, PORT_PIN_OUT);
#endif
			
			/* Enable EMAC peripheral clock */
			MDR_RST_CLK->ETH_CLOCK |= RST_CLK_ETH_CLOCK_ETH_CLK_EN;
			
			/* Reset EMAC */
			EMAC_Reset ();
			
			emac.flags |= EMAC_FLAG_POWER;
			
			/* Enable ethernet interrupts */
      NVIC_ClearPendingIRQ (ETHERNET_IRQn);
      NVIC_EnableIRQ (ETHERNET_IRQn);
			
			MDR_ETHERNET1->ETH_IMR |= (ETH_IMR_RF_OK | ETH_IMR_XF_OK);
		
			break;
    default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
{
	if (!ptr_addr)
		return ARM_DRIVER_ERROR_PARAMETER;
	
	if (!(emac.flags & EMAC_FLAG_POWER))
		return ARM_DRIVER_ERROR;
	
	ptr_addr->b[0] = MDR_ETHERNET1->ETH_MAC_H >> 8;
  ptr_addr->b[1] = MDR_ETHERNET1->ETH_MAC_H;
  ptr_addr->b[2] = MDR_ETHERNET1->ETH_MAC_M >> 8;
  ptr_addr->b[3] = MDR_ETHERNET1->ETH_MAC_M;
  ptr_addr->b[4] = MDR_ETHERNET1->ETH_MAC_T >> 8;
  ptr_addr->b[5] = MDR_ETHERNET1->ETH_MAC_T;

  return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
{
	if (!ptr_addr)
		return ARM_DRIVER_ERROR_PARAMETER;
	
	if (!(emac.flags & EMAC_FLAG_POWER))
		return ARM_DRIVER_ERROR;
	
	MDR_ETHERNET1->ETH_MAC_H = (ptr_addr->b[0] <<  8) |  ptr_addr->b[1];
	MDR_ETHERNET1->ETH_MAC_M = (ptr_addr->b[2] <<  8) |  ptr_addr->b[3];
	MDR_ETHERNET1->ETH_MAC_T = (ptr_addr->b[4] <<  8) |  ptr_addr->b[5];
	
  return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr)
{
	uint16_t ht[4];
	uint32_t crc;

  if (!ptr_addr && num_addr)
		return ARM_DRIVER_ERROR_PARAMETER;
	
	if (!(emac.flags & EMAC_FLAG_POWER))
		return ARM_DRIVER_ERROR;
	
	if (num_addr == 0)
	{
		/* Disable multicast hash filtering */
		MDR_ETHERNET1->ETH_R_CFG &= ~ETH_R_CFG_MCA_EN;
		
		return ARM_DRIVER_OK;
	}
	
	/* Calculate 64-bit Hash table for MAC addresses */
  ht[0] = 0;
  ht[1] = 0;
  ht[2] = 0;
  ht[3] = 0;
  
  for ( ; num_addr; ptr_addr++, num_addr--)
	{
    crc = crc32_data (&ptr_addr->b[0], 6) >> 26;
    ht[crc >> 4] |= (1 << (crc & 0x0F));
  }
	
	MDR_ETHERNET1->ETH_HASH0 = ht[0];
	MDR_ETHERNET1->ETH_HASH1 = ht[1];
	MDR_ETHERNET1->ETH_HASH2 = ht[2];
	MDR_ETHERNET1->ETH_HASH3 = ht[3];
	
	/* Enable multicast hash filtering */
	MDR_ETHERNET1->ETH_R_CFG |= ETH_R_CFG_MCA_EN;
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
{
	uint16_t i, tmp, head, tail;
	uint32_t *src, *dst;
	uint16_t space[2];
	
	if (!frame || !len)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (!(emac.flags & EMAC_FLAG_POWER))
    return ARM_DRIVER_ERROR;
	
	head = MDR_ETHERNET1->ETH_X_Head;
	tail = MDR_ETHERNET1->ETH_X_Tail;

	if (head > tail)
	{
		space[0] = head-tail;
		space[1] = 0;
	}
	else
	{
		space[0] = MDR_ETHERNET1_BUF_SIZE - tail;
		space[1] = head - MDR_ETHERNET1->ETH_Dilimiter;
	}

	if (len > (space[0]+space[1]-8))
		return ARM_DRIVER_ERROR_BUSY;

	tmp = len;
	src = (uint32_t *)&frame[0];
	dst = (uint32_t *)(MDR_ETHERNET1_BUF_BASE + tail);
	
	*dst++ = tmp;
	space[0] -= 4;
	
	if ((uint16_t)dst > (MDR_ETHERNET1_BUF_SIZE-4))
		dst = (uint32_t *)(MDR_ETHERNET1_BUF_BASE + MDR_ETHERNET1->ETH_Dilimiter);

	tmp = (len+3)/4;

	if (len <= space[0])
	{
		for (i=0; i<tmp; i++)
			*dst++ = *src++;
	}
	else
	{
		tmp -= space[0]/4;
		
		for (i=0; i<(space[0]/4); i++)
			*dst++ = *src++;
		
		dst = (uint32_t *)(MDR_ETHERNET1_BUF_BASE + MDR_ETHERNET1->ETH_Dilimiter);
		
		for (i=0; i<tmp; i++)
			*dst++ = *src++;
	}
	
	if ((uint16_t)dst > (MDR_ETHERNET1_BUF_SIZE-4))
		dst = (uint32_t *)(MDR_ETHERNET1_BUF_BASE + MDR_ETHERNET1->ETH_Dilimiter);
	
	tmp = 0;
	
	*dst++ = tmp;
	
	if ((uint16_t)dst > (MDR_ETHERNET1_BUF_SIZE-4))
		dst = (uint32_t *)(MDR_ETHERNET1_BUF_BASE + MDR_ETHERNET1->ETH_Dilimiter);

	MDR_ETHERNET1->ETH_X_Tail = (uint16_t)dst;
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_ReadFrame (uint8_t *frame, uint32_t len)
{
	uint16_t space_start = 0, space_end = 0, tail, head;
	uint32_t *src, *dst;
	uint32_t size, i, buf;
	uint16_t tmp[2], length;
	
	if (!frame && len)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (!(emac.flags & EMAC_FLAG_POWER))
    return ARM_DRIVER_ERROR;
	
	if ((MDR_ETHERNET1->ETH_STAT & ETH_STAT_R_COUNT_Msk) == 0)
		return ARM_DRIVER_ERROR;
	
	tail = MDR_ETHERNET1->ETH_R_Tail;
	head = MDR_ETHERNET1->ETH_R_Head;

	if (tail > head)
	{
		space_end = tail-head;
		space_start = 0;
	}
	else
	{
		space_end = MDR_ETHERNET1->ETH_Dilimiter - head;
		space_start = tail;
	}

	src = (uint32_t *)(MDR_ETHERNET1_BUF_BASE + head);
		
	*((uint32_t *)tmp) = *src++;
	
	length = tmp[0]-4;
	
	if ((length < 14) || (length > 1514))
		return ARM_DRIVER_ERROR;
	
	space_end -= 4;
		
	if ((uint16_t)src > (MDR_ETHERNET1->ETH_Dilimiter - 1))
		src = (uint32_t *)MDR_ETHERNET1_BUF_BASE;
	
	dst = (uint32_t *)&frame[0];
		
	size = (tmp[0]+3)/4;
		
	if (tmp[0] <= space_end)
	{
		for (i=0; i<(size-1); i++)
			*dst++ = *src++;
		
		buf = *src++;
	}
	else
	{
		size = size - space_end/4;
			
		for (i=0; i<(space_end/4); i++)
			*dst++ = *src++;
			
		src = (uint32_t *)MDR_ETHERNET1_BUF_BASE;
			
		for (i=0; i<(size-1); i++)
			*dst++ = *src++;
		
		buf = *src++;
	}
		
	if ((uint16_t)src > (MDR_ETHERNET1->ETH_Dilimiter - 1))
		src = (uint32_t *)MDR_ETHERNET1_BUF_BASE;

	MDR_ETHERNET1->ETH_R_Head = (uint16_t)src;
	MDR_ETHERNET1->ETH_STAT -= (1 << ETH_STAT_R_COUNT_Pos);
	
#ifdef RTE_ETH0_REC_LED
	INV_BIT (RTE_ETH0_REC_LED_PORT->RXTX, RTE_ETH0_REC_LED_PIN);
#endif
	
	return length;
}

static uint32_t ARM_ETH_MAC_GetRxFrameSize (void)
{
	int32_t len;
	
	if (!(emac.flags & EMAC_FLAG_POWER))
    return 0;
	
	if ((MDR_ETHERNET1->ETH_STAT & ETH_STAT_R_COUNT_Msk) == 0)
		return 0;
	
	len = (uint16_t)(*((uint32_t *)(MDR_ETHERNET1_BUF_BASE + MDR_ETHERNET1->ETH_R_Head))) - 4;
	
	if ((len < 14) || (len > 1514))
		return 0;
	
	return len;
}

static int32_t ARM_ETH_MAC_GetRxFrameTime (ARM_ETH_MAC_TIME *time)
{
	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_ETH_MAC_GetTxFrameTime (ARM_ETH_MAC_TIME *time)
{
	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_ETH_MAC_Control (uint32_t control, uint32_t arg)
{
	if (!(emac.flags & EMAC_FLAG_POWER))
    return ARM_DRIVER_ERROR;
	
	switch (control)
	{
		case ARM_ETH_MAC_CONFIGURE:
			switch (arg & ARM_ETH_MAC_SPEED_Msk)
			{
				/* Not used in this driver */
				case ARM_ETH_MAC_SPEED_10M:
				case ARM_ETH_MAC_SPEED_100M:
					break;
				default:
					return ARM_DRIVER_ERROR_UNSUPPORTED;
			}
			
			switch (arg & ARM_ETH_MAC_DUPLEX_Msk)
      {
				/* Not used in this driver */
				case ARM_ETH_MAC_DUPLEX_HALF:
				case ARM_ETH_MAC_DUPLEX_FULL:
					break;
      }
			
			if (arg & ARM_ETH_MAC_LOOPBACK)
				MDR_ETHERNET1->ETH_G_CFGh |= ETH_G_CFGh_DLB;
			else
				MDR_ETHERNET1->ETH_G_CFGh &= ~ETH_G_CFGh_DLB;
			
			if (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX)
				MDR_ETHERNET1->ETH_X_CFG &= ~ETH_X_CFG_CRC_EN;
			else
				MDR_ETHERNET1->ETH_X_CFG |= ETH_X_CFG_CRC_EN;
			
			if (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX)
				return ARM_DRIVER_ERROR_UNSUPPORTED;
			
			if (arg & ARM_ETH_MAC_ADDRESS_BROADCAST)
				MDR_ETHERNET1->ETH_R_CFG |= ETH_R_CFG_BCA_EN;
			else
				MDR_ETHERNET1->ETH_R_CFG &= ~ETH_R_CFG_BCA_EN;

			if (arg & ARM_ETH_MAC_ADDRESS_MULTICAST)
				MDR_ETHERNET1->ETH_R_CFG |= ETH_R_CFG_MCA_EN;
			else
				MDR_ETHERNET1->ETH_R_CFG &= ~ETH_R_CFG_MCA_EN;

			if (arg & ARM_ETH_MAC_ADDRESS_ALL)
				return ARM_DRIVER_ERROR_UNSUPPORTED;
			
			break;
    case ARM_ETH_MAC_CONTROL_TX:
			if (arg != 0)
				MDR_ETHERNET1->ETH_X_CFG |= ETH_X_CFG_EN;
			else
				MDR_ETHERNET1->ETH_X_CFG &= ~ETH_X_CFG_EN;
				
			break;
    case ARM_ETH_MAC_CONTROL_RX:
			if (arg != 0)
				MDR_ETHERNET1->ETH_R_CFG |= ETH_R_CFG_EN;
			else
				MDR_ETHERNET1->ETH_R_CFG &= ~ETH_R_CFG_EN;
			
			break;
    case ARM_ETH_MAC_FLUSH:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_ETH_MAC_SLEEP:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_ETH_MAC_VLAN_FILTER:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
    default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_MAC_ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
{
	return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t ARM_ETH_MAC_PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
{
	uint32_t timeout = 0;
	
	MDR_ETHERNET1->ETH_MDIO_CTRL =
		(reg_addr << ETH_MDIO_CTRL_RG_A_Pos) |
		(ETH_MDIO_CTRL_DIV << ETH_MDIO_CTRL_DIV_Pos) |
		(phy_addr << ETH_MDIO_CTRL_PHY_A_Pos) |
		(1 << ETH_MDIO_CTRL_OP_Pos) |
		(1 << ETH_MDIO_CTRL_PRE_EN_Pos) |
		(1 << ETH_MDIO_CTRL_RDY_Pos);
	
	do
	{
		timeout++;
	}
	while (((MDR_ETHERNET1->ETH_MDIO_CTRL & ETH_MDIO_CTRL_RDY) == 0 ) && (timeout < PHY_READ_TO));
	
	if (timeout == PHY_READ_TO)
		return ARM_DRIVER_ERROR_TIMEOUT;
	
	*data = MDR_ETHERNET1->ETH_MDIO_DATA;
	
	return ARM_DRIVER_OK;
}

int32_t ARM_ETH_MAC_PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
	uint32_t timeout = 0;
	
	MDR_ETHERNET1->ETH_MDIO_DATA = data;
	
	MDR_ETHERNET1->ETH_MDIO_CTRL =
		(reg_addr << ETH_MDIO_CTRL_RG_A_Pos) |
		(ETH_MDIO_CTRL_DIV << ETH_MDIO_CTRL_DIV_Pos) |
		(phy_addr << ETH_MDIO_CTRL_PHY_A_Pos) |
		(0 << ETH_MDIO_CTRL_OP_Pos) |
		(1 << ETH_MDIO_CTRL_PRE_EN_Pos) |
		(1 << ETH_MDIO_CTRL_RDY_Pos);
	
	do
	{
		timeout++;
	}
	while (((MDR_ETHERNET1->ETH_MDIO_CTRL & ETH_MDIO_CTRL_RDY) == 0 ) && (timeout < PHY_WRITE_TO));
	
	if (timeout == PHY_WRITE_TO)
		return ARM_DRIVER_ERROR_TIMEOUT;
	
	return ARM_DRIVER_OK;
}

void ETHERNET_IRQHandler (void)
{
	uint16_t ifr, event = 0;
	
	ifr = MDR_ETHERNET1->ETH_IFR;
	MDR_ETHERNET1->ETH_IFR = ifr;
	
	if (ifr & ETH_IFR_RF_OK)
		event |= ARM_ETH_MAC_EVENT_RX_FRAME;
	
	if (ifr & ETH_IFR_XF_OK)
		event |= ARM_ETH_MAC_EVENT_TX_FRAME;
	
	/* Callback event notification */
  if (event && emac.cb_event)
		emac.cb_event (event);
}

// End ETH MAC Interface

ARM_DRIVER_ETH_MAC Driver_ETH_MAC0 =
{
    ARM_ETH_MAC_GetVersion,
    ARM_ETH_MAC_GetCapabilities,
    ARM_ETH_MAC_Initialize,
    ARM_ETH_MAC_Uninitialize,
    ARM_ETH_MAC_PowerControl,
    ARM_ETH_MAC_GetMacAddress,
    ARM_ETH_MAC_SetMacAddress,
    ARM_ETH_MAC_SetAddressFilter,
    ARM_ETH_MAC_SendFrame,
    ARM_ETH_MAC_ReadFrame,
    ARM_ETH_MAC_GetRxFrameSize,
    ARM_ETH_MAC_GetRxFrameTime,
    ARM_ETH_MAC_GetTxFrameTime,
    ARM_ETH_MAC_ControlTimer,
    ARM_ETH_MAC_Control,
    ARM_ETH_MAC_PHY_Read,
    ARM_ETH_MAC_PHY_Write
};
