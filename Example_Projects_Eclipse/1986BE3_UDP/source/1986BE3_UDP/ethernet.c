/*  1986BE3_UDP
     __   ____  ___    __ ___  ___   __   __
    | |\/| |  )| | )  / |( ( )( ( ) / /   \ \_/
    |_|  |_|_/ |_| \__|_| /_/ (_(_)(_)_)__/_/ \
 *//**
 *  \file  ethernet.c
 *  \brief  Data link layer for 1986BE3.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#include <string.h>
#include "MDR1986VE3.h"
#include "ethernet.h"
#include "lan-config.h"

/**
 *  \addtogroup lan
 *  \{
 */

#define PHY_A                     0x1CU
#define PHY_BSR_AUTO_NEG_RDY_Pos  5
#define PHY_BSR_AUTO_NEG_RDY      ( 1U << PHY_BSR_AUTO_NEG_RDY_Pos )

#define IFR_ERRORS                ( ETH_IFR_MISSED_F | ETH_IFR_OVF | ETH_IFR_SMB_ERR \
                                  | ETH_IFR_CRC_ERR  | ETH_IFR_SF  | ETH_IFR_LF \
                                  | ETH_IFR_XF_ERR   | ETH_IFR_LC  | ETH_IFR_CRS_LOST )
uint32_t ethernet_errpool = 0;

#define MDR_ETHERNET1_RX_FIFO     (( void *)( MDR_ETHERNET1_BUF_BASE ))
#define MDR_ETHERNET1_TX_FIFO     (( void *)( MDR_ETHERNET1_BUF_BASE + 4 ))

#define MDR_ETHERNET2_RX_FIFO     (( void *)( MDR_ETHERNET2_BUF_BASE ))
#define MDR_ETHERNET2_TX_FIFO     (( void *)( MDR_ETHERNET2_BUF_BASE + 4 ))

static void *MDR_ETHERNET_RX_FIFO[ eth__COUNT_ ] = { MDR_ETHERNET1_RX_FIFO, MDR_ETHERNET2_RX_FIFO };
static void *MDR_ETHERNET_TX_FIFO[ eth__COUNT_ ] = { MDR_ETHERNET1_TX_FIFO, MDR_ETHERNET2_TX_FIFO };

static MDR_ETHERNET_TypeDef *MDR_ETHERNET[ eth__COUNT_ ] = { MDR_ETHERNET1, MDR_ETHERNET2 };

#define _REPEAT( N ) for ( int _REPEAT_i = 0; _REPEAT_i < ( N ); _REPEAT_i++ )

const char const *ETHS[ eth__COUNT_ ] = { "eth1", "eth2" };

static void ethernet_configure( int ifc, mac_addr_t mac )
{
	MDR_ETHERNET_TypeDef *eth = MDR_ETHERNET[ ifc ];

	eth->PHY_Control = 0;
	_REPEAT( 10 ) __NOP();
	/* Reset PHY, set "AutoNegotioation" mode */
	eth->PHY_Control = (
	  ( PHY_A                     << ETH_PHY_CONTROL_PHYADD_Pos )
	| ( ETH_PHY_CONTROL_MODE_AUTO << ETH_PHY_CONTROL_MODE_Pos   )
	| ( ETH_PHY_CONTROL_nRST )
	);
	_REPEAT( 100000 ) {
		if ( eth->PHY_Status & ETH_PHY_STATUS_READY ) break;
	};
	eth->ETH_MAC_T = mac.u16[ 0 ];
	eth->ETH_MAC_M = mac.u16[ 1 ];
	eth->ETH_MAC_H = mac.u16[ 2 ];
	/* Enable receiver, accept broadcast and unicast packets */
	eth->ETH_R_CFG = ETH_R_CFG_EN | ETH_R_CFG_BCA_EN | ETH_R_CFG_UCA_EN;
	/* Enable transmitter, set PAD, PRE, CRC, pause, retry count */
	eth->ETH_X_CFG = ( ETH_X_CFG_EN
	| ETH_X_CFG_PAD_EN
	| ETH_X_CFG_PRE_EN
	| ETH_X_CFG_CRC_EN
	| ETH_X_CFG_IPG_EN
	| ( ETH_X_CFG_RtryCnt_Value << ETH_X_CFG_RtryCnt_Pos )
	);
	/* Set FIFO mode, collision window */
	eth->ETH_G_CFGl = (
	  ( ETH_G_CFGl_BUFF_MODE_FIFO << ETH_G_CFGl_BUFF_MODE_Pos )
	| ( ETH_G_CFGl_ColWnd_Value   << ETH_G_CFGl_ColWnd_Pos )
	);
	/* Support debug in FIFO mode */
	eth->ETH_G_CFGh = ETH_G_CFGh_DBG_XF_EN | ETH_G_CFGh_DBG_RF_EN;
}

void ethernet_init_clock( void )
{
	static bool done = false;

	if ( done ) return;
	done = true;

	/* Reset clocks */
	MDR_RST_CLK->ETH_CLOCK &= ~(
	  ( RST_CLK_ETH_CLOCK_PHY_CLK_EN  )
	| ( RST_CLK_ETH_CLOCK_ETH_CLK_EN  )
	| ( RST_CLK_ETH_CLOCK_ETH2_CLK_EN )
	| ( RST_CLK_ETH_CLOCK_PHY_CLK_SEL_Msk )
	| ( RST_CLK_ETH_CLOCK_PHY_BRG_Msk )
	| ( RST_CLK_ETH_CLOCK_ETH_BRG_Msk )
	);
	MDR_RST_CLK->HS_CONTROL |= RST_CLK_HS_CONTROL_HSE_ON2;
	/* Set PHY_CLK = HSE2, ETH_CLK = HCLK */
	MDR_RST_CLK->ETH_CLOCK |= (
	  ( RST_CLK_ETH_CLOCK_PHY_CLK_EN  )
	| ( RST_CLK_ETH_CLOCK_PHY_CLK_SEL_HSE2 << RST_CLK_ETH_CLOCK_PHY_CLK_SEL_Pos )
	| ( RST_CLK_ETH_CLOCK_PHY_BRG_PHY1_CLK << RST_CLK_ETH_CLOCK_PHY_BRG_Pos     )
	| ( 0UL                                << RST_CLK_ETH_CLOCK_ETH_BRG_Pos     )
	);
}

void ethernet_init( int ifc, mac_addr_t mac )
{
	switch ( ifc ) {

	case eth1:
		/* Enable clock */
		MDR_RST_CLK->ETH_CLOCK |= RST_CLK_ETH_CLOCK_ETH_CLK_EN;
		/* Clear buffer */
		memset(( void *) MDR_ETHERNET1_BUF_BASE, 0xFF, MDR_ETHERNET1_BUF_SIZE );
		goto L_INIT_1;

	case eth2:
		/* Enable clock */
		MDR_RST_CLK->ETH_CLOCK |= RST_CLK_ETH_CLOCK_ETH2_CLK_EN;
		/* Clear buffer */
		memset(( void *) MDR_ETHERNET2_BUF_BASE, 0xFF, MDR_ETHERNET2_BUF_SIZE );

	L_INIT_1:
		/* Configure interface */
		ethernet_configure( ifc, mac );
		break;
	}
}

bool ethernet_autonegotiation( int ifc )
{
	MDR_ETHERNET_TypeDef *eth = MDR_ETHERNET[ ifc ];

	eth->ETH_MDIO_CTRL = (
	  ETH_MDIO_CTRL_RDY | ETH_MDIO_CTRL_PRE_EN | ETH_MDIO_CTRL_OP
	| ( PHY_A   << ETH_MDIO_CTRL_PHY_A_Pos )
	| ( PHY_BSR << ETH_MDIO_CTRL_RG_A_Pos )
	);
	_REPEAT( 500 ) {
		if ( eth->ETH_MDIO_CTRL & ETH_MDIO_CTRL_RDY ) break;
	}
	return ( eth->ETH_MDIO_DATA & PHY_BSR_AUTO_NEG_RDY );
}

bool ethernet_tx_buffer_almost_full( int ifc )
{
	return ( MDR_ETHERNET[ ifc ]->ETH_STAT & ETH_STAT_X_AFULL );
}

void ethernet_transmit_frame( int ifc, uint8_t *frame, uint16_t sz )
{
	uint32_t *fifo = MDR_ETHERNET_TX_FIFO[ ifc ];
	uint32_t *f = ( void *) frame;

	/* Send a frame using FIFO */
	*fifo = sz;
	for ( uint16_t i = ( sz + 3 ) >> 2; i > 0; i-- ) *fifo = *f++;
	*fifo = 0;  /* Unspecified action */
}

bool ethernet_receive( int ifc, uint8_t *frame )
{
	MDR_ETHERNET_TypeDef *eth = MDR_ETHERNET[ ifc ];

	uint32_t status = eth->ETH_IFR;
	eth->ETH_IFR = status;
	if (( ethernet_errpool = status & IFR_ERRORS ) != 0 ) {
		LAN_DEBUGF( "%s: ERROR POOL (IFR): %#lx.\n", ETHS[ ifc ], ethernet_errpool );
	}
	if ( eth->ETH_STAT & ETH_STAT_R_COUNT_Msk ) {
		/* Frame is received */
		uint32_t *f = ( void *) frame;
		uint32_t *fifo = MDR_ETHERNET_RX_FIFO[ ifc ];
		uint16_t sz = *fifo & 0xFFFF;
		/* Get a frame using FIFO */
		if ( sz <= ETHERNET_FRAME_LIMIT ) {
			for ( uint16_t i = ( sz + 3 ) >> 2; i > 0; i-- ) *f++ = *fifo;
			eth->ETH_STAT = 0;  /* R_COUNT -= 1 */
			ethernet_handle_frame( ifc, frame, sz );
		} else {
			/* Ignore too long frames */
			eth->ETH_STAT = 0;  /* R_COUNT -= 1 */
			LAN_DEBUGF( "%s: RX ERROR: Too long frame (%u).\n", ETHS[ ifc ], sz );
		}
		return ( true );
	}
	return ( false );
}

/** \} */
