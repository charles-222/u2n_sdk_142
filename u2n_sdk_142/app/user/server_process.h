#ifndef _SERVER_PROCESS_H_
#define _SERVER_PROCESS_H_

#include "uart.h"

#define COAP_NEW_PARAMETERS_ENABLE  	// charles222

#define U2N_BUF_SIZE 		4096 //2048 //512 //128  
#define N2U_BUF_SIZE 		4096 //2048

#define MAX_ACTIVE_CONNECTIONS		5 	// Max coucurrent TCP connection for TCP Server


#define RESPOND_BUF_SIZE	20  // end with 0 to form a string

#define NOT_AVAILABLE 		-1

char UartToNetBuffer[U2N_BUF_SIZE];
char NetToUartBuffer[N2U_BUF_SIZE];
int connectTimeoutTable[MAX_ACTIVE_CONNECTIONS];



#define DIVISION_FACTOR				1 //1000 //1 //1000		// mW to W; mV to V;...	// 1: mW; 1000: W

void tcp_ip_process(void *pvParameters);
void set_ALL_dirty(void);
void clear_ALL_dirty(void);
void prepare_new_ALL_report(uint8 get_all);


int8 ptx_fw_ver[4];

uint8 bindFailFlag;




#endif /* _CONFIG_H_ */
