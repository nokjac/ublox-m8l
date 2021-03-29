#ifndef _BROADCAST_SOCKET_H_
#define _BROADCAST_SOCKET_H_

int8_t broadcast_socket_create(char* ip, uint16_t port, char* iface);
void broadcast_socket_close(void);
int8_t broadcast_socket_send(void* buf, uint16_t len);

#endif /* _BROADCAST_SOCKET_H_ */
