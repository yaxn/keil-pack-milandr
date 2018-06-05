/*  1986BE3_UDP
     __   ____  ___    __ ___  ___   __   __
    | |\/| |  )| | )  / |( ( )( ( ) / /   \ \_/
    |_|  |_|_/ |_| \__|_| /_/ (_(_)(_)_)__/_/ \
 *//**
 *  \file  ethernet.h
 *  \brief  Data link layer for 1986BE3.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#ifndef ETHERNET_H
#define ETHERNET_H

#include <stdint.h>
#include <stdbool.h>

/**
 *  \addtogroup lan
 *  \{
 */

#ifdef __cplusplus
extern "C" {
#endif

#define ETHERNET_FRAME_LIMIT     1518
#define ETHERNET_PAYLOAD_LIMIT   1500

/**
 *  \brief Ethernet interface identifiers.
 */
enum { eth1, eth2, eth__COUNT_ };

extern const char const *ETHS[ eth__COUNT_ ];

/**
 *  \brief MAC address type.
 */
typedef union {
	uint8_t  u8[ 6 ];
	uint16_t u16[ 3 ];
} __attribute__((packed)) mac_addr_t;

#define MAC_ADDR( a, b, c, d, e, f )  { u8: { a, b, c, d, e, f }}

#define MAC_EMPTY      MAC_ADDR( 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 )
#define MAC_BROADCAST  MAC_ADDR( 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF )

/**
 *  \brief Pool of last error flags (from IFR).
 */
extern uint32_t ethernet_errpool;

/**
 *  \brief Initialize clocks of Ethernet interfaces.
 *  \param ifc Interface ID.
 */
void ethernet_init_clock( void );

/**
 *  \brief Initialize controller of Ethernet interface.
 *  \param ifc Interface ID.
 */
void ethernet_init( int ifc, mac_addr_t mac );

/**
 *  \brief Check auto-negotiation is completed.
 *  \param ifc Interface ID.
 *  \return Completion flag.
 */
bool ethernet_autonegotiation( int ifc );

/**
 *  \brief Get a packet from the receive buffer, handle a frame into ethernet_handle_frame().
 *  \param ifc Interface ID.
 *  \return Something-was-received flag.
 */
bool ethernet_receive( int ifc, uint8_t *frame );

/**
 *  \brief Prototype of a callback function for handling Ethernet frame.
 *  \param ifc Interface ID.
 *  \param data Ethernet frame.
 *  \param sz Ethernet frame size.
 */
void ethernet_handle_frame( int ifc, uint8_t *frame, uint16_t sz );

/**
 *  \brief Check the transmit buffer is almost full.
 *  \param ifc Interface ID.
 *  \return Almost-full flag.
 */
bool ethernet_tx_buffer_almost_full( int ifc );

/**
 *  \brief Send Ethernet frame.
 *  \param ifc Interface ID.
 *  \param frame Ethernet frame.
 *  \param sz Frame size.
 */
void ethernet_transmit_frame( int ifc, uint8_t *frame, uint16_t sz );

#ifdef __cplusplus
}
#endif

/** \} */

#endif /* ETHERNET_H */
