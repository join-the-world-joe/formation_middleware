/**
* @file     tcp.c
* @brief    This file implements the tcp functionalities
* @author   Joe.Zhong <wenxing.cn@gmail.com>
* @date     20/09/2018
*/


#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "log_controller.h"

#include "channel.h"
#include "registers.h"
#include "tcp.h"

#include "modbus_tcp.h"

//#define MODBUS_TCP_PORT 1502 //for debug purpose

/**
* @fn               tcp_init
* @param [in]       log_ctx
*                   log context
* @param [in]       tcp_ctx
*                   tcp context
* @param [in]       port
*                   tcp port
* @return           =0, success; <0, fail   
* @brief            to initializa tcp server
*/
int tcp_init(tcp_context *tcp_ctx, int port, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int ret = 0, cmd;
    int reusable = -1;
    int tcp_listen_fd = -1;
    struct sockaddr_in addr;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!_log_controller_ctx || !_log_controller_ioctl_param || port < 1 || port > 65535 || !tcp_ctx) {
        printf("tcp_init, incorrect parameters\n");
        return -1;
    }

    if(port != MODBUS_TCP_PORT){
        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp port is not 502");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);
    }

    tcp_listen_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(tcp_listen_fd < 0) {
        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp_init->socket, fail");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);
        return -1;
    }

    //fcntl( tcp_listen_fd, F_SETFD, 1);

    reusable = 1;
    ret = setsockopt(tcp_listen_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&reusable, sizeof(reusable));
    if(ret < 0) {

        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp_init, fail to set address reusable to be True");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);

        close(tcp_listen_fd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    ret = bind(tcp_listen_fd, (struct sockaddr *)&addr, sizeof(addr));
    if(ret < 0) {

        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp_init, can't bind socket to any address");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);

        close(tcp_listen_fd);
    }
    
    tcp_ctx->port = port;
    tcp_ctx->_log_controller_ctx = _log_controller_ctx;
    tcp_ctx->_log_controller_ioctl_param = _log_controller_ioctl_param;
    tcp_ctx->tcp_listen_sockfd = tcp_listen_fd;
    tcp_ctx->nb_tcp_connections = NB_TCP_CONNECTIONS;
    tcp_ctx->recv_timeout_in_usec = RECV_TIMEOUT_IN_USEC;

    tcp_ctx->tcp_init = tcp_init;
    tcp_ctx->tcp_deinit = tcp_deinit;
    tcp_ctx->tcp_accept = tcp_accept;
    tcp_ctx->tcp_flush = tcp_flush;
    tcp_ctx->tcp_listen = tcp_listen;
    tcp_ctx->tcp_peek = tcp_peek;
    tcp_ctx->tcp_read = tcp_read;
    tcp_ctx->tcp_write = tcp_write;
    tcp_ctx->tcp_close = tcp_close;

    return ret;
}


/**
* @fn               tcp_deinit
* @param [in]       log_ctx
*                   log context
* @param [in]       tcp_ctx
*                   tcp context
* @return           =0, success; <0, fail   
* @brief            to do something opposite to initialization
*/
void tcp_deinit(tcp_context *tcp_ctx, log_controller_context *log_controller_ctx, log_controller_ioctl_parameters *log_controller_ioctl_param)
{
    int cmd;
    log_controller_context *_log_controller_ctx = log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param = log_controller_ioctl_param;

    if(!tcp_ctx) {
        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp_deinit, incorrect parameters");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);
        return ;
    }

    close(tcp_ctx->tcp_listen_sockfd);
    tcp_ctx->port = -1;
    tcp_ctx->tcp_listen_sockfd = -1;

    return ;
}


/**
* @fn               tcp_listen
* @param [in]       tcp_ctx
*                   tcp context
* @return           =0, success; <0, fail   
* @brief            to listen from tcp pool
*/
int tcp_listen(tcp_context *tcp_ctx)
{
    int ret = 0, cmd;
    tcp_context *_tcp_ctx = tcp_ctx;
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 

    if(!_tcp_ctx) {
        printf("tcp_listen, incorrect parameters\n");
        return -1;
    }

    _log_controller_ctx = _tcp_ctx->_log_controller_ctx;
    _log_controller_ioctl_param = _tcp_ctx->_log_controller_ioctl_param;

    ret = listen(_tcp_ctx->tcp_listen_sockfd , _tcp_ctx->nb_tcp_connections);
    if(ret < 0) {

        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp_listen, can't listen to tcp connection");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);

        return -1;
    }

    return ret;
}


/**
* @fn               tcp_accept
* @param [in]       tcp_ctx
*                   tcp context
* @return           =0, success; <0, fail   
* @brief            to accept a client that is in tcp pool
*/
int tcp_accept(tcp_context *tcp_ctx)
{
    int ret = 0, cmd;
    tcp_context *_tcp_ctx = tcp_ctx;
    int tcp_client_sockfd = -1;
    socklen_t socket_addr_len;
    struct sockaddr_in client_addr;
    log_controller_context *_log_controller_ctx;
    log_controller_ioctl_parameters *_log_controller_ioctl_param; 

    if(!tcp_ctx) {
        printf("tcp_listen, incorrect parameters\n");
        return -1;
    }

    _log_controller_ctx = _tcp_ctx->_log_controller_ctx;
    _log_controller_ioctl_param = _tcp_ctx->_log_controller_ioctl_param;

    socket_addr_len = sizeof(client_addr);
    memset(&client_addr, 0, sizeof(client_addr));
    tcp_client_sockfd = accept(_tcp_ctx->tcp_listen_sockfd , (struct sockaddr *)&client_addr, &socket_addr_len);
    if(tcp_client_sockfd < 0) {

        cmd = LOG_CONTROLLER_IOCTL_CMD_LOG_MESSAGE;
        strcpy(_log_controller_ioctl_param->log_message_param.in.message, "tcp_accept, unable to achieve client address");
        _log_controller_ioctl_param->log_message_param.in.level = LOG_LEVEL_DEBUG;
        _log_controller_ctx->ops.ioctl(_log_controller_ctx, cmd, _log_controller_ioctl_param);

        return -1;
    }

    printf("new tcp connection:  %s:%d on socket %d\n", inet_ntoa(client_addr.sin_addr), client_addr.sin_port, tcp_client_sockfd);

    return tcp_client_sockfd;
}


/**
* @fn               tcp_peek
* @param [in]       tcp_ctx
*                   tcp context
* @param [in]       client_sockfd
*                   a client's socket file descriptor
* @return           =0, success; <0, fail   
* @brief            to accept a client that is in tcp pool
*/
int tcp_peek(tcp_context *tcp_ctx, int client_sockfd)
{
    int count = -1;
    char buf[65535];

    if(!tcp_ctx || client_sockfd < 0) {
        printf("tcp_peek, incorrect parameters\n");
        return -1;
    }

    count = recv(client_sockfd, buf, 65535, MSG_PEEK);

    return count;
}


/**
* @fn               tcp_read
* @param [in]       tcp_ctx
*                   tcp context
* @param [in]       client_sockfd
*                   a client's socket file descriptor
* @param [out]      buffer
*                   tcp buffer
* @param [in]       count
*                   the number of bytes
* @return           =0, success; <0, fail   
* @brief            to read a message from
*/
int tcp_read(tcp_context *tcp_ctx, int client_sockfd, uint8_t *buffer, int count)
{
    int read_count = -1;

    if(!tcp_ctx || client_sockfd < 0 || !buffer || count < 0) {
        printf("tcp_read, incorrect parameters");
        return -1;
    }

    read_count = recv(client_sockfd, buffer, count, 0);
    if(read_count < 1) {
        printf("tcp_read, fail\n");
        return -1;
    }

    return read_count;
}


/**
* @fn               tcp_flush
* @param [in]       tcp_ctx
*                   tcp context
* @param [in]       client_sockfd
*                   a client's socket file descriptor
* @return           =0, success; <0, fail   
* @brief            to flush tcp buffer
*/
void tcp_flush(tcp_context *tcp_ctx, int client_sockfd)
{
    char buf[65535];
    uint32_t count = -1;
    
    if(!tcp_ctx || client_sockfd < 0) {
        printf("tcp_flush, incorrect parameters\n");
        return ;
    }

    count = tcp_peek(tcp_ctx, client_sockfd);

    if(count > 0)
        tcp_read(tcp_ctx, client_sockfd, buf, count);

    return ;
}


/**
* @fn               tcp_write
* @param [in]       tcp_ctx
*                   tcp context
* @param [in]       client_sockfd
*                   a client's socket file descriptor
* @param [in]       buffer
*                   tcp buffer
* @param [in]       count
*                   number of bytes to write
* @return           =0, success; <0, fail   
* @brief            to write a message to tcp
*/
int tcp_write(tcp_context *tcp_ctx, int client_sockfd, uint8_t *buffer, int count)
{
    int ret = 0;

    if(!tcp_ctx || client_sockfd < 0 || !buffer || count < 0) {
        printf("tcp_write, incorrect parameters");
        return -1;
    }

    //printf("tcp write client = %d count = %d\n", client_sockfd, count);

    ret = send(client_sockfd, buffer, count, MSG_NOSIGNAL);
    if(ret < 0) {
        printf("tcp_write, fail\n");
    }

    return ret;
}


/**
* @fn               tcp_write
* @param [in]       tcp_ctx
*                   tcp context
* @param [in]       client_sockfd
*                   a client's socket file descriptor
* @return           =0, success; <0, fail   
* @brief            to close a tcp connection
*/
void tcp_close(tcp_context *tcp_ctx, int client_sockfd)
{
    if(!tcp_ctx || client_sockfd < 0) {
        printf("tcp_close, incorrect parameters\n");
        return ;
    }

    close(client_sockfd);
}

#if 0
/*
* test program
*/

void main(void)
{
    int i;
    int ret = 0;
    int count = -1;
    int retries = 10;
    char buffer[10];
    logging_ctx log_ctx;
    tcp_context tcp_ctx;
    int client_sockfd = -1;
    pthread_mutex_t log_mutex;
    socklen_t addrlen;
    struct sockaddr_in client_addr;

    
    ret = logging_init(&log_ctx, "test.log", &log_mutex);
    if(ret<0) {
        printf("logging_init fail\n");
        return ;
    }

    ret = tcp_init(&log_ctx, &tcp_ctx, MODBUS_TCP_PORT);
    if(ret < 0) {
        printf("tcp_init fail\n");
        return ;
    }

    ret = tcp_listen(&tcp_ctx);
    if(ret < 0) {
        printf("tcp_listen fail\n");
        return ;
    }

    addrlen = sizeof(client_addr);
    memset(&client_addr, 0, sizeof(client_addr));
    client_sockfd = accept(tcp_ctx.tcp_listen_sockfd, (struct sockaddr *)&client_addr, &addrlen);

    printf("New connection from %s:%d on socket %d\n", inet_ntoa(client_addr.sin_addr), client_addr.sin_port, client_sockfd);

    count = tcp_peek(&tcp_ctx, client_sockfd);

    ret = tcp_read(&tcp_ctx, client_sockfd, buffer, count);

    for(i=0; i<count; i++) {
        printf("%c", buffer[i]);
    }
    printf("\n");

    tcp_write(&tcp_ctx, client_sockfd, buffer, count);

    return ;
}
#endif

