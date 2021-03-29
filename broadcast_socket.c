#include <arpa/inet.h>
#include <errno.h>
//#include <glib.h>
#include <inttypes.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "broadcast_socket.h"

#define BROADCAST_DEFAULT_IFACE "usb0"
#define BROADCAST_DEFAULT_ADDR "255.255.255.255"
#define BROADCAST_DEFAULT_PORT 49100
int sockfd = 0;
struct sockaddr_in server;
extern int errno;

int8_t broadcast_socket_create(char* ip, uint16_t port, char* iface)
{
    printf("%s: create broadcast socket: ip: %s, port: %d, interface: %s\n", __FUNCTION__, ip, port, iface);

    int8_t ret = 1;

    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, ip, &(server.sin_addr));
    //server.sin_addr.s_addr = htonl(INADDR_ANY);

    printf("family: %d, port: %d, addr: %s\n", server.sin_family, port, ip);
    printf("family: %d, port: %d, addr: %ul\n", server.sin_family, server.sin_port, server.sin_addr.s_addr);

    if (-1 == (sockfd = socket(AF_INET, SOCK_DGRAM, 0))) {
        printf("%d: create socket failed!: %s\n", __LINE__, strerror(errno));
        ret = -1;
    } else {
        printf("socket with sockfd: %d created\n", sockfd);

        int broadcastEnable = 1;

        if (-1 == setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) {
            printf("%d: setsockopt(BROADCAST) failed: %s\n", __LINE__, strerror(errno));
            broadcast_socket_close();
            ret = -1;
        } else {
            printf("setsockopt BROADCAST set\n");

            if (-1 == setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, iface, strlen(iface) + 1)) {
                printf("%d: setsockopt(BINDTODEVICE) failed: %s\n", __LINE__, strerror(errno));
                broadcast_socket_close();
                ret = -1;
            } else {
                printf("setsockopt BINDTODEVICE set\n");
            }
        }
    }

    return ret;
}

void broadcast_socket_close(void)
{
    printf("%s: close positioning socket\n", __FUNCTION__);

    if (0 != sockfd) {
        close(sockfd);
        sockfd = 0;
    }
}

int8_t broadcast_socket_send(void* buf, uint16_t len)
{
    printf("%s: sending %d bytes to socket\n", __FUNCTION__, len);

    int8_t ret = -1;
    int numbytes;

    if ((NULL != buf) && (0 <= len)) {
        if (0 != sockfd) {
            if (-1 == (numbytes = sendto(sockfd, buf, len, 0, (struct sockaddr*)&server, sizeof(server)))) {
                printf("%d: send failed: sendto() failed: %s\n", __LINE__, strerror(errno));
            } else {
                printf("sent %d bytes\n", numbytes);
                ret = 1;
            }
        } else {
            printf("%d: send failed: no socket\n", __LINE__);
        }
    } else {
        printf("%d: send failed: buffer check failed\n", __LINE__);
    }

    return ret;
}

#if 0

int main(int argc, char *argv[])
{
	char *addr = BROADCAST_DEFAULT_ADDR;
	uint16_t port = BROADCAST_DEFAULT_PORT;
	char *iface = BROADCAST_DEFAULT_IFACE;
	
	if( 4 == argc ) {
		addr = argv[1];
		port = atoi(argv[2]);
		iface = argv[3];
	} else {
		printf("Create socket with default parameters\n");
	}
	
	if( 0 > broadcast_socket_create(addr, port, iface)) {
		printf("socket not created\n");
		return -1;
    }
	
	char msg[] = "Hallo";
	if( 0 < broadcast_socket_send((void*)msg, strlen(msg)+1)) {
        printf("message sent to socket (len=%d)\n", strlen(msg));
    } else {
        printf("send to socket failed!");
		return -1;
    }
	
	return 1;
}

#endif /*0*/
