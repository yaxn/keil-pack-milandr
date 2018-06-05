/*  1986BE3_UDP
     __   ____  ___    __ ___  ___   __   __
    | |\/| |  )| | )  / |( ( )( ( ) / /   \ \_/
    |_|  |_|_/ |_| \__|_| /_/ (_(_)(_)_)__/_/ \
 *//**
 *  \file  lan.c
 *  \brief  UDP, ARP and ping for 1986BE3.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#include <string.h>
#include "MDR1986VE3.h"
#include "ethernet.h"
#include "lan-config.h"
#include "lan.h"

/**
 *  \addtogroup lan
 *  \{
 */

#define SWAP16( x ) ( \
  (( uint16_t )( x ) & 0x00FF ) << 8 \
| (( uint16_t )( x ) & 0xFF00 ) >> 8 \
)

#define HTONS  SWAP16
#define NTOHS  SWAP16

#define htons( x )  (( uint16_t )__REV16( x ))
#define ntohs( x )  htons( x )


#define ETH_TYPE_IP   HTONS( 0x0800 )
#define ETH_TYPE_ARP  HTONS( 0x0806 )

/**
 *  \brief Type of Ethernet frame header.
 */
typedef struct {
	mac_addr_t t_mac;
	mac_addr_t s_mac;
	uint16_t   type;
	uint8_t    data[];
} __attribute__((packed)) eth_frame_t;


#define ARP_HW_TYPE_ETH   HTONS( 0x0001 )
#define ARP_PROT_TYPE_IP  HTONS( 0x0800 )
#define ARP_OPER_REQUEST  HTONS( 1 )
#define ARP_OPER_REPLY    HTONS( 2 )

/**
 *  \brief Type of ARP header.
 */
typedef struct {
	uint16_t   hw_type;
	uint16_t   prot_type;
	uint8_t    hw_len;
	uint8_t    prot_len;
	uint16_t   oper;
	mac_addr_t s_mac;
	ip_addr_t  s_ip;
	mac_addr_t t_mac;
	ip_addr_t  t_ip;
} __attribute__((packed)) arp_packet_t;

/**
 *  \brief Type of ARP cache entry.
 */
typedef struct {
	ip_addr_t  ip;
	mac_addr_t mac;
} arp_cache_entry_t;


#define IP_VERSION           4
#define IP_IHL               5
#define IP_VERSION__IHL      (( IP_VERSION << 4 ) + IP_IHL )

#define IP_PROT_ICMP         1
#define IP_PROT_UDP          17

#define IP_FRAG_0            0x80
#define IP_FRAG_DF           0x40
#define IP_FRAG_MD           0x20
#define IP_FRAG_DETECT_MASK  ( ~( IP_FRAG_0 | IP_FRAG_DF ))

/**
 *  \brief Type of IP header.
 */
typedef struct {
	uint8_t   version__ihl;
	uint8_t   dscp__ecn;
	uint16_t  total_len;
	uint16_t  f_id;
	uint16_t  f_flags__offset;
	uint8_t   ttl;
	uint8_t   prot;
	uint16_t  chsum;
	ip_addr_t s_ip;
	ip_addr_t t_ip;
	uint8_t   data[];
} __attribute__((packed)) ip_packet_t;


#define ICMP_TYPE_ECHO_REPLY  0
#define ICMP_TYPE_ECHO_REQ    8

/**
 *  \brief Type of ICMP echo header.
 */
typedef struct {
	uint8_t  type;
	uint8_t  code;
	uint16_t chsum;
	uint16_t id;
	uint16_t seq;
	uint8_t  data[];
} __attribute__((packed)) icmp_echo_packet_t;


#define UDP_PAYLOAD_LIMIT  ( ETHERNET_PAYLOAD_LIMIT - sizeof( ip_packet_t ) - sizeof( udp_packet_t ))

/**
 *  \brief Type of UDP header.
 */
typedef struct {
	uint16_t s_port;
	uint16_t t_port;
	uint16_t len;
	uint16_t chsum;
	uint8_t  data[];
} __attribute__((packed)) udp_packet_t;


/**
 *  \brief MAC addresses of interfaces.
 */
static mac_addr_t mac_addr[ eth__COUNT_ ];

/**
 *  \brief IP addresses of interfaces.
 */
static ip_addr_t ip_addr[ eth__COUNT_ ];

/**
 *  \brief Subnet masks of interfaces.
 */
static ip_addr_t ip_mask[ eth__COUNT_ ];

/**
 *  \brief Broadcast IP addresses of interfaces.
 */
static ip_addr_t ip_broadcast[ eth__COUNT_ ];

/**
 *  \brief Temporary buffer for Ethernet frame.
 */
static uint8_t frame_buffer[ ETHERNET_FRAME_LIMIT ];

/**
 *  \brief ARP table size.
 */
#define ARP_TABLE_SIZE  ( ARP_CACHE_SIZE + 1 )

/**
 *  \brief ARP table.
 */
static arp_cache_entry_t arp_cache[ ARP_TABLE_SIZE ] = {{ ip: 0, mac: MAC_BROADCAST }, };

/**
 *  \brief Current position in ARP table.
 */
static uint8_t arp_cache_pos = 1;

int lan_errno = LAN_ERR_NONE;

#ifdef DEBUG_LAN
#define assert_eth( _ifc, ... ) do { \
	if ( _ifc >= eth__COUNT_ ) { \
		LAN_DEBUGF( "ERROR: %s (%d): Wrong interface ID (%d).\n", __FUNCTION__, __LINE__, _ifc ); \
		lan_errno = LAN_ERR_INTERFACE; \
		return __VA_ARGS__; \
	} \
} while ( 0 )

#else
#define assert_eth( ifc, ... )

#endif /* DEBUG_LAN */

static uint32_t checksum( uint32_t sum, uint8_t *buf, uint16_t sz )
{
	/* Checksum all whole words */
	while ( sz > 1 ) {
		sum += (( uint16_t ) buf[ 0 ] << 8 ) | buf[ 1 ];
		if ( sum > 0xFFFF ) sum -= 0xFFFF;
		buf += 2;
		sz  -= 2;
	}
	/* Checksum a single byte if it's left over */
	if ( sz ) {
		sum += ( uint16_t ) buf[ 0 ] << 8;
		if ( sum > 0xFFFF ) sum -= 0xFFFF;
	}
	return sum;
}

static uint16_t wrapsum( uint32_t sum )
{
	sum = ~sum & 0xFFFF;
	return htons( sum );
}

void lan_init( int ifc, mac_addr_t mac, ip_addr_t ip, ip_addr_t mask )
{
	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc );

	mac_addr[ ifc ] = mac;
	ip_addr [ ifc ] = ip;
	ip_mask [ ifc ] = mask;
	ip_broadcast[ ifc ] = ip_addr[ ifc ] | ~ip_mask[ ifc ];

	ethernet_init_clock();
	ethernet_init( ifc, mac );
}

bool lan_autoneg( int ifc )
{
	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc, false );

	return ethernet_autonegotiation( ifc );
}

bool lan_receive( int ifc )
{
	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc, false );

	return ethernet_receive( ifc, frame_buffer );
}

bool lan_able_transmit( int ifc )
{
	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc, false );

	return ( !ethernet_tx_buffer_almost_full( ifc ));
}

static void eth_transmit( int ifc, mac_addr_t t_mac, uint16_t type, uint16_t sz )
{
	eth_frame_t *frame = ( void *) frame_buffer;

	frame->t_mac = t_mac;
	frame->s_mac = mac_addr[ ifc ];
	frame->type = type;
	ethernet_transmit_frame( ifc, frame_buffer, sz + sizeof( eth_frame_t ));
}

static void eth_reply( int ifc, uint16_t sz )
{
	eth_frame_t *frame = ( void *) frame_buffer;

	frame->t_mac = frame->s_mac;
	frame->s_mac = mac_addr[ ifc ];
	ethernet_transmit_frame( ifc, frame_buffer, sz + sizeof( eth_frame_t ));
}

static int arp_search_cache( ip_addr_t ip )
{
	for ( int i = 1; i < ARP_TABLE_SIZE; i++ ) {
		if ( arp_cache[ i ].ip == ip ) return ( i );
	}
	return ( 0 );  /* MAC_BROADCAST */
}

void arp_transmit( int ifc, ip_addr_t t_ip )
{
	arp_packet_t *p = ( void *)((( eth_frame_t *) frame_buffer )->data );

	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc );

	p->hw_type = ARP_HW_TYPE_ETH;
	p->prot_type = ARP_PROT_TYPE_IP;
	p->hw_len = sizeof( mac_addr_t );
	p->prot_len = sizeof( ip_addr_t );
	p->oper = ARP_OPER_REQUEST;
	p->s_mac = mac_addr[ ifc ];
	p->s_ip = ip_addr[ ifc ];
	p->t_mac = ( mac_addr_t ) MAC_EMPTY;
	p->t_ip = t_ip;
	eth_transmit( ifc, ( mac_addr_t ) MAC_BROADCAST, ETH_TYPE_ARP, sizeof( arp_packet_t ));
}

static void arp_handle( int ifc, arp_packet_t *p, uint16_t sz )
{
	if ( sz < sizeof( arp_packet_t )) {
		LAN_DEBUGF( "%s: RX ERROR: Too short ARP packet (%u).\n", ETHS[ ifc ], sz );
		lan_errno = LAN_ERR_ARP_TOO_SHORT;
		/* Ignore */
		return;
	}
	if (( p->hw_type == ARP_HW_TYPE_ETH )
	&& ( p->prot_type == ARP_PROT_TYPE_IP )
	&& ( p->t_ip == ip_addr[ ifc ])) {
		switch ( p->oper ) {

		case ARP_OPER_REQUEST:
			p->oper = ARP_OPER_REPLY;
			p->t_mac = p->s_mac;
			p->s_mac = mac_addr[ ifc ];
			p->t_ip = p->s_ip;
			p->s_ip = ip_addr[ ifc ];
			eth_reply( ifc, sizeof( arp_packet_t ));
			break;

		case ARP_OPER_REPLY:
			if ( !arp_search_cache( p->s_ip )) {
				arp_cache[ arp_cache_pos ].ip = p->s_ip;
				arp_cache[ arp_cache_pos ].mac = p->s_mac;
				++arp_cache_pos;
				if ( arp_cache_pos == ARP_TABLE_SIZE ) arp_cache_pos = 1;
			}
			break;
		}
	}
}

static void ip_reply( int ifc, uint16_t sz )
{
	ip_packet_t *p = ( void *)((( eth_frame_t *) frame_buffer )->data );
	sz += sizeof( ip_packet_t );

	p->total_len = htons( sz );
	p->chsum = 0;
	p->t_ip = p->s_ip;
	p->s_ip = ip_addr[ ifc ];
	p->chsum = wrapsum( checksum( 0, ( void *) p, sizeof( ip_packet_t )));
	eth_reply( ifc, sz );
}

#ifdef LINK_LAN_PING

static void icmp_handle( int ifc, icmp_echo_packet_t *p, uint16_t sz )
{
	if (( sz >= sizeof( icmp_echo_packet_t )) && ( p->type == ICMP_TYPE_ECHO_REQ )) {
		p->type = ICMP_TYPE_ECHO_REPLY;
		/* If you wanna save a few ticks just use a dirty hack
		p->chsum += ICMP_TYPE_ECHO_REQ; */
		p->chsum = 0;
		p->chsum = wrapsum( checksum( 0, ( void *) p, sz ));
		ip_reply( ifc, sz );
	}
}
#endif /* LINK_LAN_PING */

static void ip_handle( int ifc, ip_packet_t *p, uint16_t sz )
{
	if ( sz < sizeof( ip_packet_t )) {
		LAN_DEBUGF( "%s: RX ERROR: Too short IP packet (%u).\n", ETHS[ ifc ], sz );
		lan_errno = LAN_ERR_IP_TOO_SHORT;
		/* Ignore */
		return;
	}
	if (( p->version__ihl != IP_VERSION__IHL )                                /* IPv4, header length is 5 * 32 bit, */
	|| (( p->f_flags__offset & IP_FRAG_DETECT_MASK ) != 0 )) {                /* not fragmented */
		LAN_DEBUGF( "%s: RX WARNING: IP packet rejected.\n", ETHS[ ifc ]);
		/* Ignore */
		return;
	}
	if ( wrapsum( checksum( 0, ( void *) p, sizeof( ip_packet_t ))) != 0 ) {  /* IP header checksum should be zero */
		LAN_DEBUGF( "%s: RX ERROR: Wrong IP header checksum (%#x).\n", ETHS[ ifc ], p->chsum );
		lan_errno = LAN_ERR_IP_CHECKSUM;
		/* Ignore */
		return;
	}
	if (( p->t_ip == ip_addr[ ifc ]) || ( p->t_ip == ip_broadcast[ ifc ])) {
		udp_packet_t *udp;
		uint16_t chsum, calc;
		uint16_t len = ntohs( p->total_len );

		if ( sz < len ) {
			LAN_DEBUGF( "%s: RX ERROR: Wrong IP packet Total Length (%u), expected (%u).\n", ETHS[ ifc ], len, sz );
			lan_errno = LAN_ERR_IP_TOTAL_LEN;
			/* Ignore */
			return;
		}
		if ( sz > len ) {
			LAN_DEBUGF( "%s: RX WARNING: Wrong IP packet Total Length (%u), expected (%u).\n", ETHS[ ifc ], len, sz );
		}
		sz = len - sizeof( ip_packet_t );
		switch ( p->prot ) {

#ifdef LINK_LAN_PING
		case IP_PROT_ICMP:
			icmp_handle( ifc, ( icmp_echo_packet_t *)( p->data ), sz );
			break;

#endif
		case IP_PROT_UDP:
			udp = ( void *)( p->data );
			len = ntohs( udp->len );

			if ( sz < len ) {
				LAN_DEBUGF( "%s: RX ERROR: Wrong UDP packet Length (%u), expected (%u).\n", ETHS[ ifc ], len, sz );
				lan_errno = LAN_ERR_UDP_LEN;
				/* Ignore */
				return;
			}
			sz = len - sizeof( udp_packet_t );
			chsum = udp->chsum;
			if ( chsum != 0 ) {  /* UDP checksum is used */
				udp->chsum = 0;
				calc = wrapsum(  /* Calculate checksum, including pseudo header, UDP header and data */
				  checksum(
				    checksum(
				      ( uint32_t ) len + IP_PROT_UDP
				    , ( uint8_t *) &p->s_ip, 2 * sizeof( ip_addr_t ))
				  , p->data, len )
				);
				if ( chsum != calc ) {
					LAN_DEBUGF( "%s: RX ERROR: Wrong UDP packet checksum.\n", ETHS[ ifc ]);
					lan_errno = LAN_ERR_UDP_CHECKSUM;
					/* Ignore */
					return;
				}
			}
			udp_handle_data( ifc, ntohs( udp->s_port ), ntohs( udp->t_port ), p->s_ip, udp->data, sz );
			break;
		}
	}
}

void ethernet_handle_frame( int ifc, uint8_t *frame, uint16_t sz )
{
	eth_frame_t *f = ( void *) frame;

	if ( sz < sizeof( eth_frame_t ) + sizeof( uint32_t )) {  /* + CRC-32 */
		LAN_DEBUGF( "%s: RX ERROR: Too short frame (%u).\n", ETHS[ ifc ], sz );
		lan_errno = LAN_ERR_FRAME_TOO_SHORT;
		/* Ignore */
		return;
	}
	sz -= sizeof( eth_frame_t ) + sizeof( uint32_t );
	switch ( f->type ) {

	case ETH_TYPE_ARP:
		arp_handle( ifc, ( void *)( f->data ), sz );
		break;

	case ETH_TYPE_IP:
		ip_handle( ifc, ( void *)( f->data ), sz );
		break;
	}
}

void udp_transmit( int ifc, uint16_t s_port, uint16_t t_port, ip_addr_t t_ip, uint8_t *data, uint16_t sz )
{
	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc );

	ip_packet_t *p = ( void *)((( eth_frame_t *) frame_buffer )->data );
	udp_packet_t *udp = ( void *)( p->data );
	int arp_cache_inx = 0;

	if ( sz > UDP_PAYLOAD_LIMIT ) {
		/* Too long data */
		LAN_DEBUGF( "%s: TX ERROR: Too long payload (%u).\n", ETHS[ ifc ], sz );
		lan_errno = LAN_ERR_TOO_LONG_PAYLOAD;
		return;
	}
	if ( t_ip != ip_broadcast[ ifc ]) {
		if (( arp_cache_inx = arp_search_cache( t_ip )) == 0 ) {
			arp_transmit( ifc, t_ip );  /* Resolve target MAC address */
			LAN_DEBUGF( "%s: TX WARNING: Target MAC address is unknown, the packet hasn't been sent.\n", ETHS[ ifc ]);
			lan_errno = LAN_ERR_NO_MAC;
			return;
		}
	}
	memcpy( udp->data, data, sz );
	sz += sizeof( udp_packet_t );

	udp->s_port = htons( s_port );
	udp->t_port = htons( t_port );
	udp->len = htons( sz );
	udp->chsum = 0;
	p->s_ip = ip_addr[ ifc ];
	p->t_ip = t_ip;
	udp->chsum = wrapsum(  /* Calculate checksum, including pseudo header, UDP header and data */
	  checksum(
	    checksum(
	      ( uint32_t ) sz + IP_PROT_UDP
	    , ( uint8_t *) &p->s_ip, 2 * sizeof( ip_addr_t ))
	  , p->data, sz )
	);
	sz += sizeof( ip_packet_t );

	p->version__ihl = IP_VERSION__IHL;
	p->dscp__ecn = 0;
	p->total_len = htons( sz );
	p->f_id = 0;
	p->f_flags__offset = 0;
	p->ttl = IP_TTL;
	p->prot = IP_PROT_UDP;
	p->chsum = 0;
	p->chsum = wrapsum( checksum( 0, ( void *) p, sizeof( ip_packet_t )));

	eth_transmit( ifc, arp_cache[ arp_cache_inx ].mac, ETH_TYPE_IP, sz );
}

void udp_reply( int ifc, uint16_t sz )
{
	lan_errno = LAN_ERR_NONE;
	assert_eth( ifc );

	ip_packet_t *p = ( void *)((( eth_frame_t *) frame_buffer )->data );
	udp_packet_t *udp = ( void *)( p->data );

	if ( sz <= UDP_PAYLOAD_LIMIT ) {
		uint16_t s_port = udp->s_port;
		udp->s_port = udp->t_port;
		udp->t_port = s_port;

		sz += sizeof( udp_packet_t );
		udp->len = htons( sz );
		udp->chsum = 0;
		udp->chsum = wrapsum(  /* Calculate checksum, including pseudo header, UDP header and data */
		  checksum(
		    checksum(
		      ( uint32_t ) sz + IP_PROT_UDP
		    , ( uint8_t *) &p->s_ip, 2 * sizeof( ip_addr_t ))
		  , p->data, sz )
		);
		ip_reply( ifc, sz );
	} else {
		/* Too long data */
		LAN_DEBUGF( "%s: TX ERROR: Too long payload (%u).\n", ETHS[ ifc ], sz );
		lan_errno = LAN_ERR_TOO_LONG_PAYLOAD;
	}
}

/** \} */
