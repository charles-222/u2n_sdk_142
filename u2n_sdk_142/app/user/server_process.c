/* coap -- simple implementation of the Constrained Application Protocol (CoAP)
 *         as defined in draft-ietf-core-coap
 *
 * Copyright (C) 2010--2014 Olaf Bergmann <bergmann@tzi.org>
 *
 * This file is part of the CoAP library libcoap. Please see
 * README for terms of use. 
 */

#include "esp_common.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"


#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
//#include <sys/select.h>  // charles222
#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <netdb.h>
#include <sys/stat.h>
#include <errno.h>
#include <signal.h>

//#include "config.h"

//*******************************  // charles222
//#include "debug.h"
#include "time.h"
#include "server_process.h"
//#include "uart_process.h"
#include "uart.h"
#include "user_main.h"	// charles222 add


#define AI_PASSIVE		0x0001
#define AI_CANONNAME	0x0002
#define AI_NUMERICHOST	0x0004

# define fprintf(fd, ...) printf(__VA_ARGS__)
//*******************************

//#define COAP_RESOURCE_CHECK_TIME 2 // ENABLE_PROCESS


#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif


/* temporary storage for dynamic resource representations */
static int quit = 0;


char UartToNetBuffer[U2N_BUF_SIZE];
char NetToUartBuffer[N2U_BUF_SIZE];

//-----------------------------------
//u8 coap_put_PTU_substate_Lock_flag = 0;   // 0: Unlock; 1: Lock
//-----------------------------------

int8 ptx_fw_ver[4] 		= {0, 0, 0, 0};


unsigned char NetTempBuffer[TCP_MSS + 20]; // maximum segment size (MSS). normally max payload is 1460, adding 20 in case //[256]; //[1500];  // charles222_s1

#define MAX_PAYLOAD_TO_NET			1400 //1000 //500		// bytes
#define SOCKET_RETRY_THRESHOLD	5
#define SEND_TO_NET_EVERY_xx_MS 	30 //50



//*********************************************
// move forward one position
//*********************************************
void move_position_forward_n2u(int *position)
{
	*position += 1;
	if(*position == N2U_BUF_SIZE){
		*position = 0;
	}
}


//*********************************************
// move forward one position
//*********************************************
void move_position_forward_u2n(int *position)
{
	*position += 1;
	if(*position == U2N_BUF_SIZE){
		*position = 0;
	}
}


#if 1
//*********************************************
// move forward xx position
//*********************************************
void move_position_forward_u2n_xx_steps(int *position, int step)
{
	*position += step;
	if(*position >= U2N_BUF_SIZE){
		*position -= U2N_BUF_SIZE;
	}
}
#endif


//*****************************************************************************
// put_net_read_data_to_buffer
// return: 
//			void
//*****************************************************************************
// put received Net data to buffer
// in this function, end will not catch up start. If catched, it will back off.
// or we can change to advance the start-positon
#define INDEX_BACK_OFF
void put_net_read_data_to_buffer(char *Buffer, int count)
{
	int i, old_value;
	for(i = 0; i < count; i++){
		
		if(debug_level_1) printf("net[%d]=%02x, ", i, Buffer[i]);	// debug only

		NetToUartBuffer[n2u_end] = Buffer[i];

		old_value = n2u_end;
		
		move_position_forward_n2u(&n2u_end);

	  #ifdef INDEX_BACK_OFF	  // end position back-off, data will be old; new data are gone
		if(n2u_end == n2u_start){
			if(debug_level_release) printf("2uF ");	// net to Uart buffer is full
			n2u_end = old_value;		// back off
			break;
		}
	  #else  // start position advances, data will be new; old data are gone
		if(n2u_end == n2u_start){
			if(debug_level_1) printf("\n  Warning: NetToUartBuffer is full \n");	
			move_position_forward_n2u(&n2u_start);		// advance
			//break;
		}
	  #endif
	}
}

#ifdef LOOPBACK_TEST
#ifdef TCP_CLIENT_MODE
	uint8 u2n_start_0_found_f = 0;
	int u2n_total_count = 0;
	uint8 u2n_from_0_to_9 = 0;
#endif
#endif


// don't forward u2n_start until it is sent succesfully
int prepare_data_to_be_sent_to_net(unsigned char *buffer, int end, int max_payload){
	int i = 0;
	int local_start = u2n_start;
	
	while(local_start != end  && i < max_payload){
		buffer[i] = UartToNetBuffer[local_start];

		#ifdef LOOPBACK_TEST
		#ifdef TCP_CLIENT_MODE
			if(UartToNetBuffer[local_start] == '0'){
				u2n_start_0_found_f = 1;
			}
			if(u2n_start_0_found_f == 1){
				if(UartToNetBuffer[local_start] == u2n_from_0_to_9 + 0x30){	// '0', '1', '2', ...
					u2n_total_count++;
				}
				u2n_from_0_to_9++;
				u2n_from_0_to_9 = u2n_from_0_to_9 % 10;
			}
		#endif
		#endif

		move_position_forward_u2n(&local_start);
		i++;
	}

	#ifdef LOOPBACK_TEST
	#ifdef TCP_CLIENT_MODE
		printf("\n --> to NET count: %d ", u2n_total_count);
	#endif
	#endif

	return i;
}



//********************************************************************************************************************
// Client Client Client Client Client Client Client Client Client Client Client Client Client Client Client Client 
// Client Client Client Client Client Client Client Client Client Client Client Client Client Client Client Client 
//********************************************************************************************************************

#ifdef TCP_CLIENT_MODE

uint8 bindFailFlag = 1;

void tcp_client_write_to_net(int fd);


void tcp_ip_process(void *pvParameters)		// Client  Client  Client  Client  Client  Client
{
	fd_set readfds, writefds;
	struct timeval tv, *timeout;
	int result;
	//server_addr->sin_addr.s_addr = htonl(INADDR_ANY);
	int i;
	int length;
	struct sockaddr_in client_addr;  	// charles222 added /* client's address */
    int addr_len = sizeof(struct sockaddr_in); 	// charles222 added
	int listenfd;
	struct sockaddr_in server_addr; 	/* address of Server, remote or itself */
	my_event_t uart_event;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;  // charles222
	int local_end;

	printf("\n[Wifi2UART] - welcome to tcp_ip_process Client task!\n");

	bzero(&server_addr, sizeof(struct sockaddr_in));
	server_addr.sin_family =AF_INET;
	//server_addr->sin_port = htons(atoi(servicePort));	// uint16_t htons(uint16_t hostshort);
	//server_addr->sin_addr.s_addr = inet_addr(remoteIP);
	server_addr.sin_port = htons(atoi("80"));	// uint16_t htons(uint16_t hostshort);
	server_addr.sin_addr.s_addr = inet_addr("192.168.4.1");



	while ( !quit ) {

		if(timer_count_for_socket_retry >= SOCKET_RETRY_THRESHOLD){
			timer_count_for_socket_retry = 0;
			// if socket failed, below socket process will be retried per TIME_UP_FOR_SOCKET_S
			if(bindFailFlag == 1){	//do{

				if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){	// *** SOCK_STREAM
					if(debug_level_release) printf("\n  TCP Client socket Error \n");
					bindFailFlag = 1;
				}else{
					if(debug_level_release) printf("\n  TCP Client socket() OK. fd is %d  \n", listenfd);	
					// will fd increase in the while loop ?  so need to close it when fail.
					if(connect(listenfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
						if(debug_level_release) printf("\n  connect error, will retry %d seconds later... \n", SOCKET_RETRY_THRESHOLD);
						close(listenfd);
						bindFailFlag = 1;
						//sleep(10);	// second
						//continue;
					}else{
						if(debug_level_release) printf("\n  This Client connects to Server OK - IP: %s  port: %d \n", inet_ntoa(server_addr.sin_addr), htons(server_addr.sin_port)); //argv[1], argv[2]);
						
						bindFailFlag = 0;	// ***************  OK
					}
				}
			}
		}
		if(bindFailFlag == 1){
			vTaskDelay(1000 /*MY_DELAY_MS*/ / portTICK_RATE_MS);
			continue;
		}

		
		FD_ZERO(&readfds);
		FD_SET(listenfd, &readfds);
		
		FD_ZERO(&writefds);		// has the FD_CLR effect

		if(u2n_end != u2n_start){		// save some power, otherwise CPU will keep running
			FD_SET(listenfd, &writefds);
	  	}
	

		/* set timeout if there is a pdu to send before our automatic timeout occurs */
		tv.tv_usec = SEND_TO_NET_EVERY_xx_MS * 1000;	// 50ms		// 115200/10 * 0.05 = 576 bytes
		tv.tv_sec = 0;
		timeout = &tv;


		//**********************************************************************************************
		result = select( listenfd + 1 /*FD_SETSIZE*/, &readfds, &writefds, 0, timeout );
		//uint8 station_status = wifi_station_get_connect_status();  // STATION_IDLE(0), STATION_CONNECTING(1), STATION_GOT_IP (5)
		//if(debug_level_ON) printf("[coap] select_() result:%d, sta_status:%d\n", result, station_status);
		//if(debug_level_ON) printf("[coap] select_() ticks:%u, rtc:%u\n", xTaskGetTickCount(), system_get_rtc_time());
		if(debug_level_1) printf(" ticks:%u - ", xTaskGetTickCount());  // charles222 note: tick return 32 bits, good

		if ( result < 0 ) {		/* error */
		  	//if (errno != EINTR)  // ENABLE_PROCESS
		  	//	perror("select"); // ENABLE_PROCESS
		  	if(debug_level_ON) printf(" select_() Error !!!\n");
			
		} else if ( result > 0 ) {	/* read from socket */
		
		  	if ( FD_ISSET(listenfd, &readfds ) ) {		// read **********

				length = recvfrom(listenfd, NetTempBuffer, sizeof(NetTempBuffer), 0, (struct sockaddr *)&client_addr, (socklen_t*)&addr_len);

				if(debug_level_release) printf("\n from Net  %d B ", length);
				
				if(length <= 0){
					bindFailFlag = 1;	// **************************  to reconnect to Server
					close(listenfd);
					continue;
				}
				
				put_net_read_data_to_buffer(NetTempBuffer, length);  // n2u_end updated in this function

				uart_event.event = UART_EVENT_TX_SEND;
				uart_event.my_x2x_end = n2u_end;
				xQueueSendFromISR(my_xQueueUart, (void *)&uart_event, &xHigherPriorityTaskWoken);
		  	}


			if (FD_ISSET(listenfd, &writefds)) {		// write **********
				tcp_client_write_to_net(listenfd);
			}
			
		} else {		// == 0	/* timeout */ 

		  #if 0
			// every SEND_TO_NET_EVERY_xx_MS
			
		  	//if(debug_level_release) printf(" t ");
		  	portENTER_CRITICAL();
				local_end = u2n_end;
			portEXIT_CRITICAL();

		  	//while(u2n_start != local_end){	// if net connection is down, may run here forever
		  	if(u2n_start != local_end){
				length = prepare_data_to_be_sent_to_net(NetTempBuffer, local_end, MAX_PAYLOAD_TO_NET);
				if (write(listenfd, NetTempBuffer, length) < 0) {
		            printf(" to Net, send fail \n");
		        }else{
					portENTER_CRITICAL();
						move_position_forward_u2n_xx_steps(&u2n_start, length);
					portEXIT_CRITICAL();
					printf(" to Net, send %d bytes OK \n", length);
				}

		  	}
		  #endif
		}

	}	// while
	
	return; //return 0;  // charles222
}



void tcp_client_write_to_net(int fd)
{	
	int i, ret, temp_pos, local_end, length;

	portENTER_CRITICAL();
		local_end = u2n_end;
	portEXIT_CRITICAL();

	if(local_end == u2n_start){
		//if(debug_1_f) printf("\n returned - write_net_to_fd_buf \n ");
		return; // nothing to write
	}
	
	length = prepare_data_to_be_sent_to_net(NetTempBuffer, local_end, MAX_PAYLOAD_TO_NET);
	if(debug_level_1) printf("\n prepare bytes to Net: %d  ", length);


	if ( (ret = write(fd, NetTempBuffer, length)) < 0){
		//?? //tcflush(connectTable[index], TCOFLUSH);
		ret = 0; // in case
		if(debug_level_release) printf(" \n\n *** write fd (UART data to Net) error. write:%d bytes to fd:%d ***\n\n ", length, fd);
	}


	portENTER_CRITICAL();
		move_position_forward_u2n_xx_steps(&u2n_start, ret);
	portEXIT_CRITICAL();
	
	//if(debug_level_release) printf("-Nd:%d ", first_ret);		// this printf may be postponed until next select() is selected.  It needs to wait for 'ret' value...
	if(debug_level_release) printf("\n --> Net  %d B ", ret);		// this printf may be postponed until next select() is selected.  It needs to wait for 'ret' value...

}









//********************************************************************************************************************
// Server Server Server Server Server Server Server Server Server Server Server Server Server Server Server Server
// Server Server Server Server Server Server Server Server Server Server Server Server Server Server Server Server
//********************************************************************************************************************

#else	// --> below code for TCP Server  --  TCP Server  --  TCP Server  --  TCP Server  --  TCP Server

#define MAX_PENDING_CONNECTIONS		3	// This argument defines the maximum length to which the queue of pending connections for sockfd may grow.
#define TIME_UP_FOR_SOCKET_S		10	// sec
#define WRITE_BYTE_COUNT_NET_EVERY_TIME 256


int connectTable[MAX_ACTIVE_CONNECTIONS];	// -1 means available/empty
int connectTimeoutTable[MAX_ACTIVE_CONNECTIONS];  // unit: sec

void tcp_server_write_to_net(void);

//****************************************************************************
// find and return the Max value
//****************************************************************************
int findMaxFd(int *fd_table, int size, int fd_1, int fd_2)
{
	int temp = 0, i;
	
	for(i = 0; i < size ; i++){
		if(fd_table[i] > temp){
			temp = fd_table[i];
		}
	}
	if(fd_1 > temp){
		temp = fd_1;
	}
	if(fd_2 > temp){
		temp = fd_2;
	}
	return temp;
}

//*****************************************************************************
// return:
//			-1	: no more available space for new item
//			else: first available position
//*****************************************************************************
int findFirstAvailablePosition(int *fd_table, int size)
{
	int i;

	for(i = 0; i < size; i++){
		if(fd_table[i] == -1){		// empty
			return i;
		}
	}
	return -1;	// no more space
}


//*****************************************************************************
// return:
//			longest idle one's position
//*****************************************************************************
int find_longest_idle_one(int *table, int size)
{
	int i;
	int position = 0;
	int max;

	max = table[0];
	
	for(i = 0; i < size; i++){
		if(table[i] > max){		// empty
			max = table[i];
			position = i;
		}
	}
	return position;	// no more space
}


//*****************************************************************************
// insert to the first available place
//*****************************************************************************
void insertFd(int fd, int *fd_table, int position)
{
	fd_table[position] = fd;
}

//*****************************************************************************
//	reset connection timeout to 0 sec
//*****************************************************************************
void clearConnectionTimeout(int *table, int position)
{
	table[position] = 0;
}

//*****************************************************************************
//  set table content to -1
//*****************************************************************************
void terminateConnection(int index)
{
	close(connectTable[index]);
	connectTable[index] = -1;
}

//*****************************************************************************
// clear_u2n_buffer
// return: 
//			void
//*****************************************************************************
void clear_u2n_buffer(void)
{
	u2n_end = u2n_start = 0;
}


void tcp_ip_process(void *pvParameters)		// Server    Server    Server
{
	fd_set readfds, writefds;
	struct timeval tv, *timeout;
	int result;
	//server_addr->sin_addr.s_addr = htonl(INADDR_ANY);
	int length;
	struct sockaddr_in client_addr;  	// charles222 added /* client's address */
    int addr_len = sizeof(struct sockaddr_in); 	// charles222 added
    uint8 bindFailFlag = 1;
	int listenfd;
	struct sockaddr_in server_addr; 	/* address of Server, remote or itself */
	my_event_t uart_event;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;  // charles222

	printf("\n[Wifi2UART] - welcome to tcp_ip_process Server task!\n");

	bzero(&server_addr, sizeof(struct sockaddr_in));
	server_addr.sin_family =AF_INET;
	server_addr.sin_port = htons(atoi("80"));	// uint16_t htons(uint16_t hostshort);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("192.168.0.102");
	int fd, maxfd, connfd, ret_select, position;
	uint8 anyConnectionIsMade;

		
	for(fd = 0; fd < MAX_ACTIVE_CONNECTIONS; fd++){
		connectTable[fd] = -1;	// -1 means available/empty
	}
	
	while(1){	
		if(timer_count_for_socket_retry >= SOCKET_RETRY_THRESHOLD){	// if transparent mode
			timer_count_for_socket_retry = 0;
			// if socket failed, below socket process will be retried per TIME_UP_FOR_SOCKET_S
			if(bindFailFlag == 1){	//do{
				if ((listenfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){	// *** SOCK_STREAM
					if(debug_level_release) printf(" \n  TCP Server socket Error \n");
				}else{
					if(debug_level_release) printf("\n  TCP Server socket() OK. listenfd is %d \n", listenfd);		
							// will fd increase in the while loop ?  so need to close it when fail.
					if(bind(listenfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
						if(debug_level_release) printf("\n  bind Error, will retry later...  \n");
						close(listenfd);
						bindFailFlag = 1;
						//sleep(10);	// second
						//continue;
					}else{
						if(debug_level_release) printf("\n  bind IP OK: %s  port: %d \n", inet_ntoa(server_addr.sin_addr), htons(server_addr.sin_port)); //argv[1], argv[2]);	
						if(listen(listenfd, MAX_PENDING_CONNECTIONS) < 0){
							if(debug_level_release) printf("\n  listen Error, will retry... \n");
							close(listenfd);
							bindFailFlag = 1;
						}else{
							if(debug_level_release) printf("\n  listenfd OK: Fd#:%d  \n", listenfd);
							
							bindFailFlag = 0;	//*********** OK
						}
					}
				}
			}
			
		}

		if(bindFailFlag == 1){
			vTaskDelay(1000 /*MY_DELAY_MS*/ / portTICK_RATE_MS);
			continue;
		}

    //while(1){	
    
    	//****************************************  // *** below block to process &readfds can NOT be commented out
		//if(debug_1_f) printf("\n*** before select(), the last ret_select is %d *** \n", ret_select);
		FD_ZERO(&readfds);		// has the FD_CLR effect
		FD_ZERO(&writefds);		// has the FD_CLR effect
		
		for(fd = 0; fd < MAX_ACTIVE_CONNECTIONS; fd++){
			if(connectTable[fd] != -1){
				FD_SET(connectTable[fd],&readfds);	
			}
		}

	  	if(u2n_end != u2n_start){		// save some power, otherwise CPU will keep running
			for(fd = 0; fd < MAX_ACTIVE_CONNECTIONS; fd++){
				if(connectTable[fd] != -1){
					FD_SET(connectTable[fd], &writefds);	
					//break;	// to make things simple, if the first connection is ready to write, we will write to ALL connections
				}
			}
	  	}
		
		FD_SET(listenfd, &readfds);
		//**************************************************************************************
		
		maxfd = findMaxFd(connectTable, MAX_ACTIVE_CONNECTIONS, listenfd, 0);
		
		//***************************************************************************
		/* set timeout if there is a pdu to send before our automatic timeout occurs */
		tv.tv_usec = SEND_TO_NET_EVERY_xx_MS * 1000;	// 50ms		// 115200/10 * 0.05 = 576 bytes
		tv.tv_sec = 0;
		timeout = &tv;
		// below will return -1 (interrupted by timer), or 0 (tv, if set, timeout)
	  #if 1
	  	// set timeout to continue so that if(u2n_end != u2n_start) will be checked again every xxx ms
	    if((ret_select = select(maxfd + 1, &readfds, &writefds, NULL, timeout)) <= 0) continue; // 'tv' will be modify after this call  
	  #else
	  	if((ret_select = select(maxfd + 1, &readfds, &writefds, NULL, /*&tv*/ NULL)) <= 0) continue; // 'tv' will be modify after this call  
	  #endif
		if(debug_level_1) printf("\n\n************ number selected: %d ************* \n", ret_select );


		// *** case A: read Data form UART, and 1). send to TCP client or 2). do AT command parsing

		// *** case B: read Data from TCP client ,and send to UART
		for(fd = 0; fd < MAX_ACTIVE_CONNECTIONS; fd++){
			if(connectTable[fd] != -1){
		
				if(FD_ISSET(connectTable[fd], &readfds)){
					clearConnectionTimeout(connectTimeoutTable, fd);

					length = read(connectTable[fd], NetTempBuffer, sizeof(NetTempBuffer));
					
					if(debug_level_release) printf("\n Fd %d from Net  %d B ", connectTable[fd], length);
					
					if(length <= 0){		
						if(debug_level_release) printf(" \n read empty --> close cnnection. Fd %d \n", connectTable[fd]);
						//FD_CLR(connectTable[fd],&readfds);	// *****
						terminateConnection(fd);
					}else{

						//insert_to_n2u_buf(NetTempBuffer, length);
						put_net_read_data_to_buffer(NetTempBuffer, length);  // n2u_end updated in this function

						uart_event.event = UART_EVENT_TX_SEND;
						uart_event.my_x2x_end = n2u_end;
						xQueueSendFromISR(my_xQueueUart, (void *)&uart_event, &xHigherPriorityTaskWoken);

						//if(debug_level_release) printf(" \n Fd %d to UART %d B ", connectTable[fd], length);
					}
				}
				
			}	// if(connectTable[fd] != -1)
		}	// for

		
		// *** case C: TCP client connect to this Server
		// case C is put at last (after case A and B)to avoid read block in case B? (really ?) because we call insertFd(...) below
		if(FD_ISSET(listenfd, &readfds)){	
		
			if((position = findFirstAvailablePosition(connectTable, MAX_ACTIVE_CONNECTIONS)) == -1){
				//if(debug_level_release) printf("\n  No more cnnection is allowed. Get longest idle one to use \n" );
				
				#if 0
				if((connfd = accept(listenfd,(struct sockaddr*)&client_addr, (socklen_t*)&addr_len)) < 0){
					if(debug_level_release) printf("\n  Error at Socket's accept \n");
					continue;
				}

				close(connfd);	// stupid way!!!  // Will close new accepted connection
				
				continue;
				#endif

				position = find_longest_idle_one(connectTimeoutTable, MAX_ACTIVE_CONNECTIONS);
				terminateConnection(position);
				if(debug_level_release) printf("\n  No more cnnection is allowed. Get longest idle one to use at position:%d \n", position );
			}
			
			if((connfd = accept(listenfd,(struct sockaddr*)&client_addr, (socklen_t*)&addr_len)) < 0){
				if(debug_level_release) printf("\n  Error at Socket's accept \n");
				continue;
			}

			#if 0
			if(debug_level_release){
				if ((write(connfd, welcome_msg, sizeof(welcome_msg))) != sizeof(welcome_msg)){
					tcflush(connfd, TCOFLUSH);
					if(debug_level_release) printf(" \n *** write connfd (Welcome to server!) error *** \n ");
				}
			}
			#endif
			
			// take care of interrupt, must clear timeout before insert connectTable
			clearConnectionTimeout(connectTimeoutTable, position);
			insertFd(connfd, connectTable, position);		// *****, will set connectTable[available] = xxx
			//FD_SET(connfd,&readfds);	// *****

			struct station_info * station = wifi_softap_get_station_info();
			while(station){
				printf("\n bssid : "MACSTR", ip : "IPSTR"\n", MAC2STR(station->bssid), IP2STR(&station->ip));
				station = STAILQ_NEXT(station, next);
			}
			wifi_softap_free_station_info();	// free it
			
			if(debug_level_release) printf(" \n  client has cnnected Fd#:%d from %s : %d \n", connfd, inet_ntoa(client_addr.sin_addr), htons(client_addr.sin_port));
			
		}


		// *** case E: write to fd to send to NET
		anyConnectionIsMade = 0;
		
		if(bindFailFlag == 1){
			clear_u2n_buffer();
		}else{
			for(fd = 0; fd < MAX_ACTIVE_CONNECTIONS; fd++){
				if(connectTable[fd] != -1){
					anyConnectionIsMade = 1;
					// todo: only send to those which are ready
					// todo: each client should have a dedicate buffer
					// todo: UART interrupt send data to UART process to parse and send to clients' dedicate buffer to Net 
					if(FD_ISSET(connectTable[fd], &writefds)){		
						if(debug_level_release) printf("\n wFd:%d ",connectTable[fd]);	
						tcp_server_write_to_net();
						break;	// make above function is called only once
					}
				}
			}
			if(anyConnectionIsMade == 0){
				clear_u2n_buffer();
			}
		}
		
				
    }	//while(1)

	return; //return 0;  // charles222
}



void tcp_server_write_to_net(void)
{	
	int i, ret, temp_pos, index, local_end, length;
	int first_hit = 0, first_ret = 0;

	portENTER_CRITICAL();
		local_end = u2n_end;
	portEXIT_CRITICAL();

	if(local_end == u2n_start){		// will not happen because outside --> if(u2n_end != u2n_start){		// save some power, otherwise CPU will keep running
		//if(debug_level_release) printf("N-");		// test
		return; // nothing to write
	}
	
	length = prepare_data_to_be_sent_to_net(NetTempBuffer, local_end, MAX_PAYLOAD_TO_NET);
	if(debug_level_1) printf("\n prepare bytes to Net: %d  ", length);

	if(0 /*bindFailFlag != 0*/){		// will not happen because caller has checked the Flag is 0
		if(debug_level_release) printf(" \n *** Will send %d bytes to NULL because of socket failure. *** \n", length);
		first_ret = length;	// dummy write to null
	}else{
		// to make things simple, use the first OK one for all others
		for(index = 0; index < MAX_ACTIVE_CONNECTIONS; index++){
			if(connectTable[index] != -1){
				//if(debug_level_release) printf("\n write fd %d ", connectTable[index]);
				if ( (ret = write(connectTable[index], NetTempBuffer, length)) <= 0){
					//?? //tcflush(connectTable[index], TCOFLUSH);
					if(debug_level_release) printf("\n ret:%d, write fd %d err:%d B \n", ret, connectTable[index], length);
				}else{
					if(debug_level_release) printf("\n --> Fd %d Net  %d B ", connectTable[index], ret);	
					if(first_hit == 0){
						first_hit = 1;
						first_ret = ret;
						length = first_ret;
					}
				}
			}
		}
	}

	portENTER_CRITICAL();
		move_position_forward_u2n_xx_steps(&u2n_start, first_ret);
	portEXIT_CRITICAL();
	
	//if(debug_level_release) printf("\n --> Net  %d B ", first_ret);		// this printf may be postponed until next select() is selected.  It needs to wait for 'ret' value...

}


#endif	// #ifdef TCP_CLIENT_MODE




void clear_ALL_dirty(void)
{
}



//*************************************************************************
// parameters:
// get_all: 
//			0: to report only those which are all_dirty
//			1: to report ALL
//*************************************************************************
void prepare_new_ALL_report(uint8 get_all)
{
}



