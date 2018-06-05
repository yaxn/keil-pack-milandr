/*  1986BE3_UDP
     __   ____  ___    __ ___  ___   __   __
    | |\/| |  )| | )  / |( ( )( ( ) / /   \ \_/
    |_|  |_|_/ |_| \__|_| /_/ (_(_)(_)_)__/_/ \
 *//**
 *  \file  lan.h
 *  \brief  UDP, ARP and ping for 1986BE3.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#ifndef LAN_H
#define LAN_H

#include <stdint.h>
#include <stdbool.h>

/**
 *  \defgroup lan UDP, ARP and ping for 1986BE3.
 *
 *  Usage example:
 *
 *  \code
 *  int main()
 *  {
 *      // Initialize Ethernet controller
 *      lan_init( eth1, MY_MAC, MY_IP, MY_IP_MASK );
 *
 *      // Wait for completion of auto-negotiation...
 *      while ( !lan_autoneg( eth1 ));
 *
 *      while ( 1 ) {
 *          // Receive packets, handle a payload into udp_handle_data()
 *          lan_receive( eth1 );
 *          // Check lan_errno here...
 *
 *          // If the transmit buffer has some free space...
 *          if ( lan_able_transmit( eth1 )) {
 *              // Send data using UDP datagram
 *              udp_transmit( eth1, MY_PORT, TARGET_PORT, TARGET_IP, data, sz );
 *              // Check lan_errno here...
 *          }
 *      }
 *  }
 *
 *  void udp_handle_data( int ifc, uint16_t s_port, uint16_t t_port, ip_addr_t s_ip, uint8_t *data, uint16_t sz )
 *  {
 * 	   // Handle a payload here...
 *  }
 *
 *  \endcode
 *  \{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief IP address type.
 */
typedef uint32_t ip_addr_t;

#define IP_ADDR( a, b, c, d ) ( \
  (( ip_addr_t )( a )       ) \
| (( ip_addr_t )( b ) <<  8 ) \
| (( ip_addr_t )( c ) << 16 ) \
| (( ip_addr_t )( d ) << 24 ) \
)

/**
 *  \brief LAN error identifiers.
 */
enum {
	/* all functions */
	LAN_ERR_NONE,
	LAN_ERR_INTERFACE,
	/* udp_transmit */
	LAN_ERR_NO_MAC,
	LAN_ERR_TOO_LONG_PAYLOAD,
	/* lan_receive */
	LAN_ERR_FRAME_TOO_SHORT,
	LAN_ERR_ARP_TOO_SHORT,
	LAN_ERR_IP_TOO_SHORT,
	LAN_ERR_IP_CHECKSUM,
	LAN_ERR_IP_TOTAL_LEN,
	LAN_ERR_UDP_CHECKSUM,
	LAN_ERR_UDP_LEN,
};

/**
 *  \brief Last error.
 */
extern int lan_errno;

/**
 *  \brief Initialize controller of Ethernet interface.
 *  \param ifc Interface ID.
 *  \param mac Interface MAC address.
 *  \param ip Interface IP address.
 *  \param mask Interface subnet mask.
 */
void lan_init( int ifc, mac_addr_t mac, ip_addr_t ip, ip_addr_t mask );

/**
 *  \brief Check auto-negotiation is completed.
 *  \param ifc Interface ID.
 *  \return Completion flag.
 */
bool lan_autoneg( int ifc );

/**
 *  \brief Get a packet from the receive buffer, handle a payload into udp_handle_data().
 *
 *  Check the result of this operation in lan_erron variable.
 *
 *  \param ifc Interface ID.
 *  \return Something-was-received flag.
 */
bool lan_receive( int ifc );

/**
 *  \brief Prototype of a callback function for handling data received into UDP packet.
 *  \param ifc Interface ID.
 *  \param s_port Sender UDP port.
 *  \param t_port Target UDP port.
 *  \param s_ip Sender IP address.
 *  \param data Payload.
 *  \param sz Payload size.
 */
void udp_handle_data( int ifc, uint16_t s_port, uint16_t t_port, ip_addr_t s_ip, uint8_t *data, uint16_t sz );

/**
 *  \brief Check the transmit buffer has free space.
 *  \param ifc Interface ID.
 *  \return Has-some-space flag.
 */
bool lan_able_transmit( int ifc );

/**
 *  \brief Send UDP packet.
 *
 *  Check the result of this operation in lan_erron variable:
 *  - LAN_ERR_NONE              Packet was passed into the transmit buffer.
 *  - LAN_ERR_NO_MAC            Packet was not sent because the target IP address is not found in the ARP cache,
 *                              but an ARP request was sent instead. Try again later.
 *  - LAN_ERR_INTERFACE         Wrong interface ID.
 *  - LAN_ERR_TOO_LONG_PAYLOAD  Wrong payload size.
 *
 *  \param ifc Interface ID.
 *  \param s_port Sender UDP port.
 *  \param t_port Target UDP port.
 *  \param t_ip Target IP address.
 *  \param data Payload.
 *  \param sz Payload size.
 */
void udp_transmit( int ifc, uint16_t s_port, uint16_t t_port, ip_addr_t t_ip, uint8_t *data, uint16_t sz );

/**
 *  \brief Reply with UDP packet.
 *
 *  Check the result of this operation in lan_erron variable, see udp_transmit.
 *
 *  \param ifc Interface ID.
 *  \param sz Payload size.
 */
void udp_reply( int ifc, uint16_t sz );

/**
 *  \brief Send ARP packet.
 *  \param ifc Interface ID.
 *  \param t_ip Target IP address.
 */
void arp_transmit( int ifc, ip_addr_t ip );

#ifdef __cplusplus
}
#endif

/** \} */

#endif /* LAN_H */
