/**
* @file     set_get_ip.c
* @brief    set_get_ip.c
* @author   Jerry.Chen <jerry@sctmes.com>
* @date     02/26/2018
*/

#ifndef __SET_GET_IP_H__
#define __SET_GET_IP_H__


#define IFNAME "eth0"
//#define HOST "192.168.0.204"
#define ifreq_offsetof(x)  offsetof(struct ifreq, x)

int set_ip(in_addr_t *);
int get_ip(in_addr_t *);

#endif // __MISC_H__
