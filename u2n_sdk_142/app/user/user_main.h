

#ifndef __USER_MAIN_H__
#define __USER_MAIN_H__

#include <stdlib.h>
#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



//*****************************************************
//#define WIFI_AUTO_CONNECTION_ENABLE			// charles222 add, to use AES (EBC, 32 bytes key) and TATUNG BASE64 for AP auto connection
//#define WIFI_AUTO_CONNECTION_SMART_CONFIG_ENABLE	// WARNing: make it exclusive to SMART_CONFIG_TEST_ENABLE
//*****************************************************

#define TCP_SERVER_CLIENT_SYSTEM
#define TCP_CLIENT_MODE		// if not defined, it will become a TCP Server
//#define LOOPBACK_TEST

#if 0
#define MY_OTA_UPGRADE_ENABLE				// charles222

#define UART_COMMAND_RESENT_ENABLE		// charles222 add, to enable UART command resent mechanism if no response is received

#define SMART_CONFIG_TEST_ENABLE	// WARNing: make it exclusive to WIFI_AUTO_CONNECTION_SMART_CONFIG_ENABLE
#endif

#define debug_level_1  				0  		// charles222_s2
#define debug_level_3  				0
#define debug_level_5  				0
#define debug_level_ON 				0 //1
#define debug_level_RETRANSMIT 		0 //1  		// observe retransmit
#define debug_level_RETRANSMIT_OFF 	0  		// observe retransmit

#define debug_level_release			1	// for FW release, minimum debug messages

#define debug_uart_level_1 		0
#define debug_uart_level_ON 	0 //1


#define POINTER_LENGTH		6  // 8
#define PASSWORD_LENGTH		16
#define BASE_64_LENGTH		22
#define AES_KEY_LENGTH		32
#define USER_DATA_SECTOR_START 0x7C //0x7D, don't work  // see page 20 in my_Flash_Map_2A-ESP8266....pdf	// each sector is 4K bytes, start sector of the 3 sectors which are used for flash read/write protection.

typedef struct{
	uint8 key[AES_KEY_LENGTH];		// AES Key
	uint8 gw_ap_mac[6];				// GW's AP MAC Address
	uint8 pointer[POINTER_LENGTH];	// GW SSID Pointer. Currently it's fixed
}auto_conn_smart_config_struct;

auto_conn_smart_config_struct user_parameters_to_flash;		// alignment of 4 bytes
auto_conn_smart_config_struct user_parameters_from_flash;	// alignment of 4 bytes

uint8 company_code[1];
uint8 wifi_fw_ver[2];

#define U2N_BUF_SIZE 		4096 //2048 //512 //128  
#define N2U_BUF_SIZE 		4096 //2048

#define MAX_ACTIVE_CONNECTIONS		5 	// Max coucurrent TCP connection for TCP Server


#define RESPOND_BUF_SIZE	20  // end with 0 to form a string

#define NOT_AVAILABLE 		-1

char UartToNetBuffer[U2N_BUF_SIZE];
char NetToUartBuffer[N2U_BUF_SIZE];
int connectTimeoutTable[MAX_ACTIVE_CONNECTIONS];

int u2n_start,  u2n_end;
int n2u_start,  n2u_end;



void tcp_ip_process(void *pvParameters);


uint8 timer_count_for_socket_retry;


#define NO_AES_KEY_AND_GW_MAC


#define AES_KEY_32_BYTES_STRING		"+AZ9eT1mO+V7mxOu3s0WfRqte4EqfQq9"
#define POINTER_6_BYTES_STRING		"&%***%"



#define TIVA_COMPANY_CODE 	0x01
#define WIFI_FW_VER_H	 	0x00
#define WIFI_FW_VER_L	 	0x01
// WiFi FW version release:
// 0x0001		// released on 20160818


#endif

