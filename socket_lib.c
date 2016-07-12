/*------------------------------------------------------------------------------
 * socket_lib.c
 * 
 * author   : Guangli Dong
 *
 * history  : 2016/07/07 new
 *
 *----------------------------------------------------------------------------*/

/* includes ------------------------------------------------------------------*/
#include "socket_lib.h"

#ifdef WIN32
/* creat_server_socket() for windows
 * returns:
 *  -1          -> error
 * non-negative -> ok
 */
extern socket_t creat_server_socket(const char *IP, int PORT)
{
    /* local variables */
    socket_t sock;
    struct sockaddr_in servaddr;
    WORD    wVersionRequested;
    WSADATA wsaData;
    int err, opt, ret;
    
    /* 1. setup socket lib version */
    wVersionRequested = MAKEWORD(2, 0);
    err = WSAStartup(wVersionRequested, &wsaData);
    if(err != 0)
    {
        printf("Socket2.0 initialise failed, exit!\n");
        return -1;
    }
    if( LOBYTE(wsaData.wVersion) != 2 || HIBYTE( wsaData.wVersion) != 0)
    {
        printf("Socket version error, exit !\n");
        WSACleanup();
        return -1;
    }

    /* 2. creat socket */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == INVALID_SOCKET)
    {
        printf("Creat socket failed, exit!\n");
        return -1;
    }

    /* 3. enable address reuse */
    opt = 1;
    ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, 
        sizeof(opt));

    /* 4. bind socket with local address */    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(IP);
    servaddr.sin_port = htons(PORT);
    bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr));

    /* 5. set sock to listen mode */
    if( listen(sock, 10) == SOCKET_ERROR)
    {
        printf("Socket listen error, exit !\n");
        closesocket(sock);
        WSACleanup();
        return -1;
    }

    /* 6. return socket */
    return sock;
}

/* creat_client_socket() for windows
 * returns:
 *  -1          -> error
 * non-negative -> ok
 */
extern socket_t creat_client_socket(const char *IP, int PORT)
{
    /* local variables */
    socket_t sock;
    struct sockaddr_in servaddr;
    WORD    wVersionRequested;
    WSADATA wsaData;
    int err;


    /* 1. setup socket lib version */
    wVersionRequested = MAKEWORD(2, 0);
    err = WSAStartup(wVersionRequested, &wsaData);
    if(err != 0)
    {
        printf("Socket2.0 initialise failed, exit!\n");
        return -1;
    }
    if( LOBYTE(wsaData.wVersion) != 2 || HIBYTE( wsaData.wVersion) != 0)
    {
        printf("Socket version error, exit !\n");
        WSACleanup();
        return -1;
    }

    /* 2. creat socket */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == INVALID_SOCKET)
    {
        printf("Creat socket failed, exit!\n");
        return -1;
    }

    /* 3. connet socket with server address */    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(IP);
    servaddr.sin_port = htons(PORT);
    if( connect(sock, (struct sockaddr*)&servaddr, sizeof(servaddr)) 
        == SOCKET_ERROR)
    {
        printf("Connect server error!\n");
        closesocket(sock);
        WSACleanup();
        return -1;
    }
    
    /* 4. return socket */
    return sock;
}

extern void close_server_socket(socket_t sock)
{
    closesocket(sock);
    WSACleanup();
}

extern void close_client_socket(socket_t sock)
{
    close_server_socket(sock);
}

#else
/* creat_server_socket() for linux
 * returns:
 *  -1          -> error
 * non-negative -> ok
 */
extern socket_t creat_server_socket(const char *IP, int PORT)
{
    /* local variable */
    socket_t sock, ret;
    int opt;
    struct sockaddr_in servaddr;

    /* 1. open socket */
    if( (sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        printf("creat socket error: %s(errno: %d)\n", strerror(errno), errno);
        return sock;
    }

    /* 2. enable address reuse */
    opt = 1;
    ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* 3. convert server address and port */
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(inet_addr(IP));
    servaddr.sin_port = htons(PORT);

    /* 4. bind socket with address and port */
    if( ret = bind (sock, (struct sockaddr*)&servaddr, sizeof(servaddr))
            == -1) {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }

    /* 5. set socket to listen mode */
    if( listen(sock, 10) == -1) {
        printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }

    /* 6. return created socket */
    return sock;
}

/* creat_client_socket() for linux
 * returns:
 *  -1          -> error
 * non-negative -> ok
 */
extern socket_t creat_client_socket(const char *IP, int PORT)
{
    /* local variables */
    socket_t sock;
    struct sockaddr_in servaddr;

    /* 1. open socket */
    if( (sock = socket(AF_INET, SOCK_STREAM, 0)) <0 ) {
        printf("creat socket error: %s(errno: %d)\n", strerror(errno), errno);
        return sock;
    }

    /* 2. set server address and port to be connected */
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    if( inet_pton(AF_INET, IP, &servaddr.sin_addr) <=0 ) {
        printf("inet_pton error for %s\n", IP);
        return -1;
    }

    /* 3. connect client socket with server address and port */
    if( connect(sock, (struct sockaddr*)&servaddr, sizeof(servaddr)) <0 ) {
        printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
        return -1;
    }

    /* 4. return created socket */
    return sock;
}

extern void close_server_socket(socket_t sock)
{
    close(sock);
}

extern void close_client_socket(socket_t sock)
{
    close_server_socket(sock);
}

#endif // WIN32

