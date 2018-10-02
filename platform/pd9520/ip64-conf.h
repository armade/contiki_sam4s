/*---------------------------------------------------------------------------*/
#ifndef IP64_CONF_H
#define IP64_CONF_H
/*---------------------------------------------------------------------------*/
#include "ip64-eth-interface.h"
/*---------------------------------------------------------------------------*/
#define IP64_CONF_UIP_FALLBACK_INTERFACE ip64_eth_interface
#define IP64_CONF_INPUT                  ip64_eth_interface_input
#include "init_net.h"
#define IP64_CONF_ETH_DRIVER             ksz8863_ip64_driver
/*---------------------------------------------------------------------------*/
#endif /* IP64_CONF_H */
