#ifndef __MDR1986VE1T_ETH_MAC_H
#define __MDR1986VE1T_ETH_MAC_H

#include <MDR1986VE1T.h>
#include "MDR1986VE1T_PORT.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#include <Driver_ETH_MAC.h>

#include "MDR1986VE1T_ETH_PHY.h"

/* EMAC Driver state flags */
#define EMAC_FLAG_INIT	(1 << 0)        // Driver initialized
#define EMAC_FLAG_POWER	(1 << 1)        // Driver power on

/* EMAC Driver Control Information */
typedef struct
{
  ARM_ETH_MAC_SignalEvent_t	cb_event;       // Event callback
  uint8_t										flags;          // Control and state flags
} EMAC_CTRL;

int32_t ARM_ETH_MAC_PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data);
int32_t ARM_ETH_MAC_PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data);

#endif
