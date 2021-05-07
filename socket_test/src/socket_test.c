/*https://wiki.netbsd.org/examples/socket_programming/*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>




#define ADRESS_PORT           54503
#define ADRESS_IP             0x0102030405
#define MAXPENDING            5
#define BUFFSIZE              21

#define SERVER_SOCKET         1
#define CLIENT_SOCKET         0

#define TRUE                  1
#define FALSE                 0
#define START                 11
#define DIVIDER               ":"


int make_socket ( uint16_t port, int type, const char * server_IP )
{
    int sock;
    struct hostent *    hostinfo = NULL;
    struct sockaddr_in  server_address;

    /* Create the socket. */
    sock = socket ( PF_INET, SOCK_RAW, IPPROTO_RAW );
    if (sock < 0) {
        perror ( "socket" );
        exit ( 1 );
    }

    /* Give the socket a name. */
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons ( port );
    server_address.sin_addr.s_addr = inet_addr(server_IP);

    /* Establish the connection to the server */
    if (connect(sock, (struct sockaddr *) &server_address, sizeof(server_address)) < 0) {
        printf("connect() failed\n"); //print error
    }

/*    if ( type == SERVER_SOCKET ) {
        server_address.sin_addr.s_addr = htonl(ADRESS_IP);
        if ( bind ( sock, ( struct sockaddr * ) &server_address, sizeof ( server_address ) ) < 0 ) {
            perror ( "bind" );
            exit ( 1 );
        }

        if ( listen(sock, MAXPENDING) < 0 ) {
            printf("listen() failed");
        }
    } else if ( type == CLIENT_SOCKET ) {
        server_address.sin_addr.s_addr = inet_addr(server_IP);

         Establish the connection to the server
        if (connect(sock, (struct sockaddr *) &server_address, sizeof(server_address)) < 0) {
            printf("connect() failed\n"); //print error
        }
    }*/
    return sock;
}

void close_socket (int socket)
{
    close (socket);
}

char * clean_data( const char * data )
{
    int count;
    char * ptr_data      = NULL;
    char * result_data   = NULL;
    char * temp_ptr_data = NULL;
    int len;
    int write_info, ifone;

    ptr_data = strstr (data, DIVIDER);
    ptr_data =& ptr_data[strlen(DIVIDER)];

    temp_ptr_data = malloc ( strlen (ptr_data) );
    strcpy (temp_ptr_data, ptr_data);
    result_data = (char *) strsep (&temp_ptr_data, DIVIDER);
    printf ("%i, %i, %s", strlen (data), strlen (ptr_data), result_data);
    return result_data;
}

void send_data ( int socket, const char * data )
{
    int sent_bytes, all_sent_bytes;
    int err_status;
    int sendstrlen;

    sendstrlen = strlen ( data );
    all_sent_bytes = 0;
    int i =0;
    for (i=0; i < 100; i++)
    {
		sent_bytes = write ( socket, data, sendstrlen);
		all_sent_bytes = all_sent_bytes + sent_bytes;
		printf ("\t !!! Sent data: %s --- \n", data);
		usleep(100000);
    }
}

int main(void) {
	puts("Hello World!!!"); /* prints Hello World!!! */
/*    int sock = make_socket(ADRESS_PORT, CLIENT_SOCKET, "10.10.10.10");
    send_data (sock, "Some data to be sent");
    close_socket(sock);*/
	int bpf = Mw_TL_OpenBPFDevice();
	send_data (bpf, "Some data to be sent");
	return EXIT_SUCCESS;
}
