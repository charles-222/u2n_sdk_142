/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: user_main.c
 *
 * Description: entry file of user application
 *
 * Modification history:
 *     2014/12/1, v1.0 create this file.
*******************************************************************************/
#include "user_main.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "uart.h"	// charles222 added

//#include "ets_sys.h" 
//#include "freertos/osapi.h" 
//#include "user_interface.h"
//#include "esp_timer.h" 

#include "ssl/ssl_crypto.h"


void task2(void *pvParameters);
void task3(void *pvParameters);
void task4_UDP(void *pvParameters);

void my_AES_CBC_test(void);
void my_int_length_test(void);


char UartToNetBuffer[U2N_BUF_SIZE];
char NetToUartBuffer[N2U_BUF_SIZE];
int connectTimeoutTable[MAX_ACTIVE_CONNECTIONS];

int u2n_start,  u2n_end;
int n2u_start,  n2u_end;

uint8 PTU_WIFI_MAC[6];

uint8 gw_ap_mac_addr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};	



//**********************  xx ms soft timer start  ***************************
unsigned int my_xx_ms_timer_count = 0;	// 4 bytes
os_timer_t my_timer_t;
uint8 global_60_sec_time_up_f = 0;
uint8 timer_count_for_socket_retry = 0;

#define TIME_OF_FIRST_SCAN				7	//3		// seconds	// don't do it before first reconnect attempt
#define TIME_OF_FIRST_QUERY				5		// seconds
#define MY_TIMER_PERIOD_MS				20
#define CMD_RESEND_PERIOD_MS			2000	// todo, change to 200 ms to meet spec
#define CMD_RESEND_COUNT_MAX			2
#define CMD_SEND_EVERY_XX_SEC			30 //10
#define PTU_POWER_DIRTY_EVERY_XX_SEC	10


void my_xx_ms_timer_process(void * dummy) 	// MY_TIMER_PERIOD_MS ms timer
{ 
	my_event_t uart_event;
	uint8 i;

	// every 1 seconds
	if(my_xx_ms_timer_count % ( 1 * (1000/MY_TIMER_PERIOD_MS)) == 0){
		timer_count_for_socket_retry ++;

		// if tcp_server_mode_connection_timeout_s is 0, never timeout
		// TCP Server connection timeout depends on traffic on both direction (UART-> Net and NET to UART)
		#ifndef TCP_CLIENT_MODE		// if Server mode
			// connection timeout handling for TCP Server mode
			for(i = 0; i < MAX_ACTIVE_CONNECTIONS; i++){
				connectTimeoutTable[i]++;
				//if(debug_1_f) printf(" \n ************************** test, timeout[0] is %d *** \n", connectTimeoutTable[0]);
				if(connectTimeoutTable[i] >= 3600 * 24){	// one day
					connectTimeoutTable[i] = 3600 * 24;	// hold it to avoid overrun in above ++
					
					#if 0
					if(connectTable[i] != -1){		// under connection
						if(debug_5_f) printf("\n *** connectTimeoutTable[%d] is %d *** \n", i, connectTimeoutTable[i]);
						if(debug_on_f) printf("\n [Wifi2UART] connection timeout, close Fd#:%d \n", connectTable[i]);
						terminateConnection(i);
					}
					#endif
				}
			}
		#endif	
	}

	// every 30 seconds
	if(my_xx_ms_timer_count % (CMD_SEND_EVERY_XX_SEC * (1000/MY_TIMER_PERIOD_MS)) == TIME_OF_FIRST_QUERY * (1000/MY_TIMER_PERIOD_MS)){

		//printf("\n[every %u sec] timer count:%u; now:%u \r\n", CMD_SEND_EVERY_XX_SEC, my_xx_ms_timer_count, now);
		printf("\n[%u s] \n", CMD_SEND_EVERY_XX_SEC);

	}

	
	my_xx_ms_timer_count++;
	
}
//**********************  one second soft timer end  ***************************


//********************************************************
//********************************************************
#ifdef ERASE_FLASH_USER_PARAMETER_AREA

void erase_flash_user_parameter_area_process(void)
{
	uint8 the_result;
	
	// 4K * 4 = 16K user program area

	printf("\n");
	printf("------------- erase user parameter area 16K flash ------------- \n");
	
	the_result = spi_flash_erase_sector(USER_DATA_SECTOR_START + 0);	// return 0 means OK
	printf("erase sector %02x, result:%d (0: OK)\n", USER_DATA_SECTOR_START + 0, the_result);
	the_result = spi_flash_erase_sector(USER_DATA_SECTOR_START + 1);	// return 0 means OK
	printf("erase sector %02x, result:%d (0: OK)\n", USER_DATA_SECTOR_START + 1, the_result);
	the_result = spi_flash_erase_sector(USER_DATA_SECTOR_START + 2);	// return 0 means OK
	printf("erase sector %02x, result:%d (0: OK)\n", USER_DATA_SECTOR_START + 2, the_result);
	the_result = spi_flash_erase_sector(USER_DATA_SECTOR_START + 3);	// return 0 means OK
	printf("erase sector %02x, result:%d (0: OK)\n", USER_DATA_SECTOR_START + 3, the_result);

	the_result = system_param_load(USER_DATA_SECTOR_START, 0, &user_parameters_from_flash, sizeof(user_parameters_from_flash));
	printf("read_result:%d (1: OK; 0: Fail), Pointer:%02x %02x %02x, Key:%02x %02x %02x, bytes read:%d \n\n", 
				the_result,	// 1 is OK
				user_parameters_from_flash.pointer[0], user_parameters_from_flash.pointer[1], user_parameters_from_flash.pointer[2],
				user_parameters_from_flash.key[0], user_parameters_from_flash.key[1], user_parameters_from_flash.key[2], sizeof(user_parameters_from_flash));

}
#endif


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}



/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{	
	//char mac[6];
	uint8 i, check_result;

	#ifndef TCP_CLIENT_MODE		// if Server mode
		for(i = 0; i < MAX_ACTIVE_CONNECTIONS; i++){
			connectTimeoutTable[i] = 0;
		}
	#endif
	
	// print to UART0 before below command...
	uart_init_new();  // charles222, enable UART0 and UART1  // will print to UART1 afterward...
	
	//my_AES_CBC_test();		// charles222 add, test only
	//my_int_length_test();	// charles222 add, test only
	
    //printf("2016_0530_1, SDK version:%s\n", system_get_sdk_version()); 	// SDK 1.0.4
    printf("2016_0919 -- u2n  u2n  u2n  -- 9600 -------------------------------\n"); 
	printf("WiFi module FW version:%02x%02x \n", WIFI_FW_VER_H, WIFI_FW_VER_L);
	//printf("SDK version:%s\n", system_get_sdk_version());


	wifi_get_macaddr(STATION_IF, PTU_WIFI_MAC);
	printf("Station MAC:%02x%02x%02x%02x%02x%02x\n", PTU_WIFI_MAC[0],PTU_WIFI_MAC[1],PTU_WIFI_MAC[2],PTU_WIFI_MAC[3],PTU_WIFI_MAC[4],PTU_WIFI_MAC[5]);	//printf("Station MAC(%d): %s\n", len, PTU_WIFI_MAC);
	

    /* need to set opmode before you set config */

  	#if 1
		#ifdef TCP_SERVER_CLIENT_SYSTEM
			#ifdef TCP_CLIENT_MODE
				printf("TCP Client --> set to STATION MODE \n"); 	
				wifi_set_opmode(STATION_MODE);
			#else
				printf("TCP Server --> set to SOFTAP MODE \n"); 	
				wifi_set_opmode(SOFTAP_MODE);
			#endif
		#else
			printf("set to STATION MODE \n"); 	
			wifi_set_opmode(STATION_MODE);
		#endif
	#else
		printf("set to STATION + AP MODE \n");
    	wifi_set_opmode(STATIONAP_MODE);  	// ***** Station + AP
	#endif
		

	//ETS_UART_INTR_DISABLE();
	
	#ifdef TCP_SERVER_CLIENT_SYSTEM
		#ifdef TCP_CLIENT_MODE		// Client  Client  Client  Client
		
			struct station_config *config = (struct station_config *)zalloc(sizeof(struct station_config));
		    sprintf(config->ssid, "zigbee123");		// charles222
		    sprintf(config->password, "12345678");	// charles222
		    //sprintf(config->ssid, "Tiva_ASUS");		// charles222
	    	//sprintf(config->password, "tiva8835");	// charles222
		    /* need to sure that you are in station mode first,
		     * otherwise it will be failed. */
		    wifi_station_set_config(config);  // Set the configuration parameters of the WiFi station and save them to the flash.
		    free(config);
			
		#else						// Server  Server  Server  Server
		
			struct softap_config *config = (struct softap_config *)zalloc(sizeof(struct softap_config));
		    sprintf(config->ssid, "zigbee123");		// charles222
		    sprintf(config->password, "12345678");	// charles222
		    config->authmode = AUTH_WPA_WPA2_PSK;
			config->ssid_len = 9;
			config->max_connection = 5;
		    wifi_softap_set_config(config);  // Set the configuration parameters of the WiFi station and save them to the flash.
		    free(config);
			
		#endif
	#else
	    struct station_config *config = (struct station_config *)zalloc(sizeof(struct station_config));
	    sprintf(config->ssid, "Tiva_ASUS");		// charles222
	    sprintf(config->password, "tiva8835");	// charles222
	    /* need to sure that you are in station mode first,
	     * otherwise it will be failed. */
	    wifi_station_set_config(config);  // Set the configuration parameters of the WiFi station and save them to the flash.
	    free(config);
	#endif

	//ETS_UART_INTR_ENABLE();
	
	#if 0
	struct station_config config_test[5];  // test
	int num = wifi_station_get_ap_info(config_test);
	printf("user_init_(), recorded AP number: %d \n", num);
	#endif
	

  	// below are three tasks for test only
    //xTaskCreate(task2, 					"tsk2", 				256, NULL, 2, NULL);  // TCP Client
    //xTaskCreate(task3, 					"tsk3", 				256, NULL, 3, NULL);
	//xTaskCreate(task4_UDP, 				"tsk4_UDP", 			256, NULL, 4, NULL);
	
	// charles222, after making StackDepth from 256 to 512, the crash problem is solved (under heavy traffic on both UART and Wifi) 
	// Each real time task has its own stack memory area so the context can be saved by simply pushing processor registers onto the task stack.

	xTaskCreate(tcp_ip_process, 	"tcp_ip_process", 	2048 /*512*/, 	NULL, 6, NULL);	// priority: 6

	xTaskCreate(my_uart_task, 		"my_uart_task", 		2048 /*512*/  /*256*/, 	NULL, 5, NULL);
  
	os_timer_disarm(&my_timer_t);     
	os_timer_setfn(&my_timer_t, my_xx_ms_timer_process, NULL);     
	os_timer_arm(&my_timer_t, MY_TIMER_PERIOD_MS, 1); 	// in ms

}







#if 0
void task2(void *pvParameters)
{
    printf("[task2] - welcome to TCP Client!\r\n");

    while (1) {
        int recbytes;
        int sin_size;
        int str_len;
        int sta_socket;

        struct sockaddr_in local_ip;
        struct sockaddr_in remote_ip;

        sta_socket = socket(PF_INET, SOCK_STREAM, 0);

        if (-1 == sta_socket) {
            //close(sta_socket);
            printf("C > socket fail!\n");
			vTaskDelay(4000/portTICK_RATE_MS);
            continue;
        }

        printf("C > socket ok: %d \n", sta_socket);
        bzero(&remote_ip, sizeof(struct sockaddr_in));
        remote_ip.sin_family = AF_INET;
        remote_ip.sin_addr.s_addr = inet_addr(remote_tcp_server_ip);
        remote_ip.sin_port = htons(remote_tcp_server_port);

        if (0 != connect(sta_socket, (struct sockaddr *)(&remote_ip), sizeof(struct sockaddr))) {
            printf("C > connect fail! will close socket\n");
			close(sta_socket);
            vTaskDelay(4000 / portTICK_RATE_MS);
            continue;
        }

        printf("C > connect ok!\n");
        char *pbuf = (char *)zalloc(1024);
		//sprintf(pbuf, "%s\n", "client_send info");
        sprintf(pbuf, "%s\n", "client_send info: charles222");

        if (write(sta_socket, pbuf, strlen(pbuf) + 1) < 0) {
            printf("C > send fail\n");
        }

        printf("C > send success\n");
        free(pbuf);

        char *recv_buf = (char *)zalloc(128);
        while ((recbytes = read(sta_socket , recv_buf, 128)) > 0) {  // blocked ??? charles222: always here
        	recv_buf[recbytes] = 0;
            printf("C > read data success: %d byte(s)\nC > %s\n", recbytes, recv_buf);
        }
        free(recv_buf);

        if (recbytes <= 0) {
		    close(sta_socket);
            printf("C > read data fail!\n");
        }
    }
}


void task3(void *pvParameters)
{
	
    printf("[task3] - welcome to TCP Server!\r\n");  // charles222
	
    while (1) {
        struct sockaddr_in server_addr, client_addr;
        int server_sock, client_sock;
        socklen_t sin_size;
        bzero(&server_addr, sizeof(struct sockaddr_in));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(local_tcp_server_port);  // charles222

        int recbytes;

        do {
            if (-1 == (server_sock = socket(AF_INET, SOCK_STREAM, 0))) {
                printf("S > socket error\n");
				vTaskDelay(4000/portTICK_RATE_MS);
                break;
            }

            printf("S > create socket ok: %d\n", server_sock);

            if (-1 == bind(server_sock, (struct sockaddr *)(&server_addr), sizeof(struct sockaddr))) {
                printf("S > bind fail, will close socket\n");
				close(server_sock);
				vTaskDelay(4000/portTICK_RATE_MS);
                break;
            }

            printf("S > bind port: %d\n", ntohs(server_addr.sin_port));

            if (-1 == listen(server_sock, 5)) {
                printf("S > listen fail\n");
                break;
            }

            printf("S > listen ok\n");

            sin_size = sizeof(client_addr);

            for (;;) {
                printf("S > wait client\n");

                if ((client_sock = accept(server_sock, (struct sockaddr *) &client_addr, &sin_size)) < 0) {
                    printf("S > accept fail\n");
                    continue;
                }

                printf("S > Client from %s %d\n", inet_ntoa(client_addr.sin_addr), htons(client_addr.sin_port));

                char *recv_buf = (char *)zalloc(128);
                while ((recbytes = read(client_sock , recv_buf, 128)) > 0) { 	//blocked ??? 
                	recv_buf[recbytes] = 0;
                    printf("S > read data success: %d byte(s)\nS > %s\n", recbytes, recv_buf);
                }
                free(recv_buf);

                if (recbytes <= 0) {
                    printf("S > read data fail!\n");
                    close(client_sock);
                }
            }
        } while (0);
    }
}


void task4_UDP(void *pvParameters)
{
	
    printf("[task4] - welcome to UDP task!\r\n");  // charles222

	#define UDP_DATA_LEN 128  // charles222
	LOCAL int32 sock_fd;
	struct sockaddr_in server_addr, from;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY;
	server_addr.sin_port = htons(local_udp_server_port);
	server_addr.sin_len = sizeof(server_addr);
	int ret;  // charles222
	uint8 *udp_msg = malloc(UDP_DATA_LEN);  // charles222, unsigned int???

	// Create socket
	do{
		sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sock_fd == -1) {
			printf("ESP8266 UDP task > failed to create sock!\n");
			vTaskDelay(1000/portTICK_RATE_MS);
		}
	}while(sock_fd == -1);
	
	printf("ESP8266 UDP task > socket OK!\n");

	// Bind a local port
	do{
		ret = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
		if (ret != 0) {
			printf("ESP8266 UDP task > captdns_task failed to bind sock!\n");
			vTaskDelay(1000/portTICK_RATE_MS);
		}
	}while(ret != 0);
	
	printf("ESP8266 UDP task > bind OK!\n");

	// Receiving and transmission of UDP data
	int nNetTimeout = 2000; // 2 second, charles222
	
	while(1){
		memset(udp_msg, 0, UDP_DATA_LEN);
		memset(&from, 0, sizeof(from));
		setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&nNetTimeout, sizeof(int));
		int fromlen = sizeof(struct sockaddr_in);
		ret = recvfrom(sock_fd, (uint8 *)udp_msg, UDP_DATA_LEN, 0,(struct sockaddr*)&from,(socklen_t *)&fromlen);
		if (ret > 0) {
			printf("ESP8266 UDP task > recv %d Bytes from Port %d %s\n",ret, 
				ntohs(from.sin_port), inet_ntoa(from.sin_addr));
			
			sendto(sock_fd,(uint8*)udp_msg, ret, 0, (struct sockaddr *)&from, fromlen);
			printf("ESP8266 UDP task > sendto_() send %d Bytes out\n", ret);
		}
	}
	
	if(udp_msg){
		free(udp_msg);
		udp_msg = NULL;
	}
	
	close(sock_fd);

}
#endif


