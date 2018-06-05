#include "MDR1986VE1T_ETH_PHY.h"

#define ARM_ETH_PHY_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

#if ((defined(RTE_Drivers_ETH_PHY0) || defined(RTE_Drivers_ETH_MAC0)) && (!RTE_ETH_PHY0) && (!RTE_ETH_MAC0))
	#error "Ethernet Physical Interface Transceiver 0 not configured in RTE_Device.h!"
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION PHY_DriverVersion =
{
	ARM_ETH_PHY_API_VERSION,
	ARM_ETH_PHY_DRV_VERSION
};

//
// Functions
//

static ARM_DRIVER_VERSION ARM_ETH_PHY_GetVersion (void)
{
	return PHY_DriverVersion;
}

static int32_t ARM_ETH_PHY_Initialize (ARM_ETH_PHY_Read_t fn_read, ARM_ETH_PHY_Write_t fn_write)
{
	MDR_RST_CLK->ETH_CLOCK &=
		~(RST_CLK_ETH_CLOCK_PHY_BRG_Msk |
			RST_CLK_ETH_CLOCK_PHY_CLK_EN |
			RST_CLK_ETH_CLOCK_PHY_CLK_SEL_Msk);
		
	MDR_RST_CLK->ETH_CLOCK |=
		(RTE_ETH_PHY0_BRG << RST_CLK_ETH_CLOCK_PHY_BRG_Pos) |
		(RTE_ETH_PHY0_CLK_SEL << RST_CLK_ETH_CLOCK_PHY_CLK_SEL_Pos);
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_PHY_Uninitialize (void)
{	
	MDR_RST_CLK->ETH_CLOCK &=
		~(RST_CLK_ETH_CLOCK_PHY_BRG_Msk |
			RST_CLK_ETH_CLOCK_PHY_CLK_SEL_Msk);
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_PHY_PowerControl (ARM_POWER_STATE state)
{
	switch (state)
	{
		case ARM_POWER_OFF:
			MDR_RST_CLK->ETH_CLOCK &= ~RST_CLK_ETH_CLOCK_PHY_CLK_EN;
		
			break;
    case ARM_POWER_LOW:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_POWER_FULL:
			MDR_RST_CLK->ETH_CLOCK |= RST_CLK_ETH_CLOCK_PHY_CLK_EN;
		
			MDR_ETHERNET1->PHY_Control &= ~ETH_PHY_CONTROL_nRST;
		
			while ((MDR_ETHERNET1->PHY_Status & ETH_PHY_STATUS_READY) == 0);
		
			MDR_ETHERNET1->PHY_Control |= ETH_PHY_CONTROL_nRST;
		
			MDR_ETHERNET1->PHY_Control &= ~ETH_PHY_CONTROL_PHYADD_Msk;
			MDR_ETHERNET1->PHY_Control |= (ETH_PHY_ADD << ETH_PHY_CONTROL_PHYADD_Pos);
		
			break;
    default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	
	return ARM_DRIVER_OK;
}

static int32_t ARM_ETH_PHY_SetInterface (uint32_t interface)
{
	return ARM_DRIVER_OK; // Interface MII
}

static int32_t ARM_ETH_PHY_SetMode (uint32_t mode)
{
	uint16_t val;
	
	ARM_ETH_MAC_PHY_Read (ETH_PHY_ADDR, PHY_BCR, &val);
	
	val &= ~(PHY_BCR_DUPLEX_MODE | PHY_BCR_ISOLATE |
		PHY_BCR_AUTO_NEG_EN | PHY_BCR_SPEED_SEL | PHY_BCR_LOOPBACK);
	
	switch (mode & ARM_ETH_PHY_SPEED_Msk)
  {
		case ARM_ETH_PHY_SPEED_10M:
			val &= ~PHY_BCR_SPEED_SEL;
		
			break;
    case ARM_ETH_PHY_SPEED_100M:
			val |= PHY_BCR_SPEED_SEL;
		
			break;
    default:
			return ARM_DRIVER_ERROR_UNSUPPORTED;
	}

	switch (mode & ARM_ETH_PHY_DUPLEX_Msk)
  {
		case ARM_ETH_PHY_DUPLEX_HALF:
			val &= ~PHY_BCR_DUPLEX_MODE;
			MDR_ETHERNET1->ETH_G_CFGl |= ETH_G_CFGl_HD_EN;

			break;
    case ARM_ETH_PHY_DUPLEX_FULL:
			val |= PHY_BCR_DUPLEX_MODE;
			MDR_ETHERNET1->ETH_G_CFGl &= ~ETH_G_CFGl_HD_EN;
	}
	
	if (mode & ARM_ETH_PHY_AUTO_NEGOTIATE)
		val |= PHY_BCR_AUTO_NEG_EN;
	else
		val &= ~PHY_BCR_AUTO_NEG_EN;
	
	if (mode & ARM_ETH_PHY_LOOPBACK)
		val |= PHY_BCR_LOOPBACK;
	else
		val &= ~PHY_BCR_LOOPBACK;
	
	if (mode & ARM_ETH_PHY_ISOLATE)
		val |= PHY_BCR_ISOLATE;
	else
		val &= ~PHY_BCR_ISOLATE;
	
	ARM_ETH_MAC_PHY_Write (ETH_PHY_ADDR, PHY_BCR, val);
		
	return ARM_DRIVER_OK;
}

static ARM_ETH_LINK_STATE ARM_ETH_PHY_GetLinkState (void)
{	
	if (MDR_ETHERNET1->PHY_Status & ETH_PHY_STATUS_LED1)
	{
#ifdef RTE_ETH0_LINK_LED
		CLR_BIT (RTE_ETH0_LINK_LED_PORT->RXTX, RTE_ETH0_LINK_LED_PIN);
#endif
		return ARM_ETH_LINK_DOWN;
	}
	
#ifdef RTE_ETH0_LINK_LED
	SET_BIT (RTE_ETH0_LINK_LED_PORT->RXTX, RTE_ETH0_LINK_LED_PIN);
#endif
	return ARM_ETH_LINK_UP;
}

static ARM_ETH_LINK_INFO ARM_ETH_PHY_GetLinkInfo (void)
{
	ARM_ETH_LINK_INFO info;
	
	info.speed = (MDR_ETHERNET1->PHY_Status & ETH_PHY_STATUS_LED0) ?
		ARM_ETH_SPEED_10M : ARM_ETH_SPEED_100M;
	
	info.duplex = (MDR_ETHERNET1->PHY_Status & ETH_PHY_STATUS_LED3) ?
		ARM_ETH_DUPLEX_HALF : ARM_ETH_DUPLEX_FULL;
	
	return (info);
}

ARM_DRIVER_ETH_PHY Driver_ETH_PHY0 =
{
    ARM_ETH_PHY_GetVersion,
    ARM_ETH_PHY_Initialize,
    ARM_ETH_PHY_Uninitialize,
    ARM_ETH_PHY_PowerControl,
    ARM_ETH_PHY_SetInterface,
    ARM_ETH_PHY_SetMode,
    ARM_ETH_PHY_GetLinkState,
    ARM_ETH_PHY_GetLinkInfo,
};
