/*---------------------------------------------------------------------------*/
#ifndef IP64_CONF_H
#define IP64_CONF_H
/*---------------------------------------------------------------------------*/
#include "test_ip64-eth-interface.h"
/*---------------------------------------------------------------------------*/

#define IP64_CONF_UIP_FALLBACK_INTERFACE 	test_ip64_eth_interface
#define IP64_CONF_INPUT                  	test_ip64_eth_interface_input
#include "init_net.h"
#define IP64_CONF_DHCP                   1
#define IP64_CONF_ETH_DRIVER             ksz8863_ip64_driver
/*---------------------------------------------------------------------------*/
#endif /* IP64_CONF_H */
