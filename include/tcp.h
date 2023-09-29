/**
* @file     tcp.h
* @brief    This file is the header file of the tcp functionalities
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     20/09/2018
*/

#ifndef __TCP_H__
#define __TCP_H__

#include <sys/time.h>
#include "log_controller.h"

#define NB_TCP_CONNECTIONS 8
#define RECV_TIMEOUT_IN_USEC 100

struct tcp_context;

typedef struct _tcp_context {
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 
    long recv_timeout_in_usec;
    int tcp_listen_sockfd;
    int port; /*Tcp Port*/
    int nb_tcp_connections; /*the number of tcp connections*/

    int (*tcp_init)(struct _tcp_context *, int port, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
    int (*tcp_listen)(struct _tcp_context *);
    int (*tcp_accept)(struct _tcp_context *);
    int (*tcp_peek)(struct _tcp_context *, int);
    int (*tcp_read)(struct _tcp_context *, int, uint8_t *, int);
    int (*tcp_write)(struct _tcp_context *, int, uint8_t *, int);
    void (*tcp_flush)(struct _tcp_context *, int);
    void (*tcp_close)(struct _tcp_context *, int);
    void (*tcp_deinit)(struct _tcp_context *, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);

}tcp_context;

int tcp_init(tcp_context *tcp_ctx, int port, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);
int tcp_listen(tcp_context *tcp_ctx);
int tcp_accept(tcp_context *tcp_ctx);
int tcp_peek(tcp_context *tcp_ctx, int client_sockfd);
int tcp_read(tcp_context *tcp_ctx, int client_sockfd, uint8_t *buffer, int count);
int tcp_write(tcp_context *tcp_ctx, int client_sockfd, uint8_t *buffer, int count);
void tcp_flush(tcp_context *tcp_ctx, int client_sockfd);
void tcp_close(tcp_context *tcp_ctx, int client_sockfd);
void tcp_deinit(tcp_context *tcp_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param);

#endif
