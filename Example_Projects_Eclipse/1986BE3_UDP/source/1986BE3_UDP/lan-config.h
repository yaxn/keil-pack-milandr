/*  1986BE3_UDP
     __   ____  ___    __ ___  ___   __   __
    | |\/| |  )| | )  / |( ( )( ( ) / /   \ \_/
    |_|  |_|_/ |_| \__|_| /_/ (_(_)(_)_)__/_/ \
 *//**
 *  \file  lan-config.h
 *  \brief  Config for 1986BE3_UDP.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#ifndef LAN_CONFIG_H
#define LAN_CONFIG_H

/**
 *  \brief Handle ICMP echo packets (ping).
 */
#define LINK_LAN_PING

/**
 *  \brief Output debug messages.
 */
#define DEBUG_LAN

/**
 *  \brief Max number of attempts to transmit a frame.
 */
#define ETH_X_CFG_RtryCnt_Value  3

/**
 *  \brief Collision Window (in bit times * 4).
 */
#define ETH_G_CFGl_ColWnd_Value  32

/**
 *  \brief ARP cache size.
 */
#define ARP_CACHE_SIZE  32

/**
 *  \brief Time-to-live value of datagram.
 */
#define IP_TTL  64


#ifdef DEBUG_LAN
#include <stdio.h>

/**
 *  \brief Debug printf().
 */
#define LAN_DEBUGF( ... ) do { \
	printf( __VA_ARGS__ ); \
} while ( 0 )

#else
#define LAN_DEBUGF( ... )

#endif /* DEBUG_LAN */

#endif /* LAN_CONFIG_H */
