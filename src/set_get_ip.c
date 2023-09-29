/**
* @file     set_get_ip.c
* @brief    set_get_ip.c
* @author   Jerry.Chen <jerry@sctmes.com>
* @date     02/26/2018
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>             /* offsetof */
#include <net/if.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <sys/ioctl.h>


#if __GLIBC__ >=2 && __GLIBC_MINOR >= 1
#include <netpacket/packet.h>
#include <net/ethernet.h>
#else
#include <asm/types.h>
#include <linux/if_ether.h>
#endif


#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <unistd.h>

#include "set_get_ip.h"

int set_ip(in_addr_t *in_ip)
{
        struct ifreq ifr;
        struct sockaddr_in sai;
        int sockfd;                     /* socket fd we use to manipulate stuff with */
        int selector, ret;
        unsigned char mask;

        char *p;

        /* Create a channel to the NET kernel. */
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        /* get interface name */
        strncpy(ifr.ifr_name, IFNAME, IFNAMSIZ);

        memset(&sai, 0, sizeof(struct sockaddr));
        sai.sin_family = AF_INET;
        sai.sin_port = 0;

        sai.sin_addr.s_addr = *in_ip;

        p = (char *) &sai;
        memcpy( (((char *)&ifr + ifreq_offsetof(ifr_addr) )),
                        p, sizeof(struct sockaddr));

        ret = ioctl(sockfd, SIOCSIFADDR, &ifr);
        if(ret != 0) {
            printf("set_ip->ioctl SIOCSIFADDR faile\n");
            close(sockfd);
            return -1;
        }
        
        ret = ioctl(sockfd, SIOCGIFFLAGS, &ifr);
        if(ret != 0) {
            printf("set_ip->ioctl SIOCGIFFLAGS faile\n");
            close(sockfd);
            return -1;
        }

        ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
        // ifr.ifr_flags &= ~selector;  // unset something

        ret = ioctl(sockfd, SIOCSIFFLAGS, &ifr);
        if(ret != 0) {
            printf("set_ip->ioctl SIOCSIFFLAGS faile\n");
            close(sockfd);
            return -1;
        }

        close(sockfd);
        return 0;
}


int get_ip(in_addr_t *out_ip)
{
    struct ifaddrs *ifaddr, *ifa;
    int family, s, ret;
    char host[NI_MAXHOST];
    struct in_addr oup;

    if (getifaddrs(&ifaddr) == -1) 
    {
        printf("get_ip->getifaddrs \n");
        return -1;
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) 
    {
        if (ifa->ifa_addr == NULL)
            continue;

        s=getnameinfo(ifa->ifa_addr,sizeof(struct sockaddr_in),host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if((strcmp(ifa->ifa_name,IFNAME)==0)&&(ifa->ifa_addr->sa_family==AF_INET))
        {
            if (s != 0)
            {
                printf("get_ip->getnameinfo() failed: %s\n", gai_strerror(s));
                return -1;
            }
            printf("get_ip:Interface : <%s>\n",ifa->ifa_name );
            printf("get_ip:Address : <%s>\n", host);

            ret = inet_aton(host, &oup);
            if(ret != 1){
                printf("get_ip->inet_aton ip invaid\n");
                freeifaddrs(ifaddr);
                return -1;
            }

            *out_ip = oup.s_addr;

            freeifaddrs(ifaddr);
            return 0;
        }
    }

    return -1;
}

