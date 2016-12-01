/*
 *  Copyright (C) 2014 -2016  Espressif System
 *
 */

#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//#include "../libcoap/uart_process.h"
#include "uart.h"
//#include "../libcoap/config.h"
#include "../user/server_process.h"
//#include "../libcoap/resource.h"
#include "../user/user_main.h"	// charles222 add


extern unsigned int my_xx_ms_timer_count;

int8 htc_fw_ver_received = 0;
int8 htc_spec_ver_received = 0;

//char uart_can_parse_now_f = 0;

enum {
    UART_EVENT_RX_CHAR,
    UART_EVENT_MAX
};

typedef struct _os_event_ {
    uint32 event;
    uint32 param;
} os_event_t;


xTaskHandle xUartTaskHandle;
xQueueHandle xQueueUart;
xQueueHandle my_xQueueUart;

LOCAL STATUS
uart_tx_one_char(uint8 uart, uint8 TxChar)
{
    while (true) {
        uint32 fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart) , TxChar);
    return OK;
}

STATUS uart_tx_string(uint8 uart, uint8 *TxString)
{
	int i = 0;
  	while(TxString[i] != 0){
		uart_tx_one_char(uart, TxString[i]);
		i++;
  	}
	return OK;
}


LOCAL void
uart1_write_char(char c)
{
    if (c == '\n') {
        uart_tx_one_char(UART1, '\r');
        uart_tx_one_char(UART1, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_one_char(UART1, c);
    }
}

LOCAL void
uart0_write_char(char c)
{
    if (c == '\n') {
        uart_tx_one_char(UART0, '\r');
        uart_tx_one_char(UART0, '\n');
    } else if (c == '\r') {
    } else {
        uart_tx_one_char(UART0, c);
    }
}

LOCAL void
uart_rx_intr_handler_ssc(void *arg)
{
    /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
      * uart1 and uart0 respectively
      */
    os_event_t e;
    portBASE_TYPE xHigherPriorityTaskWoken;

    uint8 RcvChar;
    uint8 uart_no = 0;

    if (UART_RXFIFO_FULL_INT_ST != (READ_PERI_REG(UART_INT_ST(uart_no)) & UART_RXFIFO_FULL_INT_ST)) {
        return;
    }

    RcvChar = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;

    WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);

    e.event = UART_EVENT_RX_CHAR;
    e.param = RcvChar;

    xQueueSendFromISR(xQueueUart, (void *)&e, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#if 0
LOCAL void
uart_config(uint8 uart_no, UartDevice *uart)
{
    if (uart_no == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } else {
        /* rcv_buff size if 0x100 */
        _xt_isr_attach(ETS_UART_INUM, uart_rx_intr_handler_ssc, NULL);
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    }

    uart_div_modify(uart_no, UART_CLK_FREQ / (uart->baut_rate));

    WRITE_PERI_REG(UART_CONF0(uart_no), uart->exist_parity
                   | uart->parity
                   | (uart->stop_bits << UART_STOP_BIT_NUM_S)
                   | (uart->data_bits << UART_BIT_NUM_S));

    //clear rx and tx fifo,not ready
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);

    if (uart_no == UART0) {
        //set rx fifo trigger
        WRITE_PERI_REG(UART_CONF1(uart_no),
                       ((0x01 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S));
    } else {
        WRITE_PERI_REG(UART_CONF1(uart_no),
                       ((0x01 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S));
    }

    //clear all interrupt
    WRITE_PERI_REG(UART_INT_CLR(uart_no), 0xffff);
    //enable rx_interrupt
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_FULL_INT_ENA);
}
#endif

LOCAL void
uart_task(void *pvParameters)
{
    os_event_t e;

    for (;;) {
        if (xQueueReceive(xQueueUart, (void *)&e, (portTickType)portMAX_DELAY)) {
            switch (e.event) {
                case UART_EVENT_RX_CHAR:
                    //printf("%c", e.param);
                    break;

                default:
                    break;
            }
        }
    }

    vTaskDelete(NULL);
}




#ifdef LOOPBACK_TEST
#ifdef TCP_CLIENT_MODE
	uint8 n2u_start_0_found_f = 0;
	int n2u_total_count = 0;
	uint8 n2u_from_0_to_9 = 0;
#endif
#endif

void my_uart_task(void *pvParameters)
{
	my_event_t uart_event;
	uint8 payload_zero[6] = {0, 0, 0, 0, 0, 0};
	int i;
	
    printf("[uart] - welcome to my_uart_task!\r\n");
	
	for (;;) {
        if (xQueueReceive(my_xQueueUart, (void *)&uart_event, (portTickType)portMAX_DELAY)) { // block indefinitely
        	switch(uart_event.event){
				
				#if 0
				case UART_EVENT_RX_DATA_PROCESS:
					if(debug_uart_level_1) printf("\n Q0- ");  //"Q- "); //printf("-xQ-- ");
        			// use local u2n_end index passed from ISR to avoid contention on u2n_end
        			#ifdef HTC_CMD_ENABLE
					  HTC_command_parse(UartToNetBuffer, &u2n_start, uart_event.my_x2x_end /*u2n_end*/, 0 /*dummy*/);
					#else
					  //AT_command_parse(UartToNetBuffer, &u2n_start, uart_event.my_x2x_end /*u2n_end*/, 0 /*dummy*/);
					#endif
					
					break;
				#endif
				
				case UART_EVENT_TX_SEND:
					i = 0;
					while(n2u_start != uart_event.my_x2x_end){
						uart_tx_one_char(UART0, NetToUartBuffer[n2u_start]);

						#ifdef LOOPBACK_TEST
						#ifdef TCP_CLIENT_MODE
							if(NetToUartBuffer[n2u_start] == '0'){
								n2u_start_0_found_f = 1;
							}
							if(n2u_start_0_found_f == 1){
								if(NetToUartBuffer[n2u_start] == n2u_from_0_to_9 + 0x30){	// '0', '1', '2', ...
									n2u_total_count++;
								}
								n2u_from_0_to_9++;
								n2u_from_0_to_9 = n2u_from_0_to_9 % 10;
							}
						#endif
						#endif

						move_position_forward_n2u(&n2u_start);
						i++;
					}
					
					if(debug_level_release) printf("\n Q-%d ", i);

					#ifdef LOOPBACK_TEST
					#ifdef TCP_CLIENT_MODE
						printf("\n <-- to UART count: %d ", n2u_total_count);
					#endif
					#endif
					
					break;
					
				default:
					if(debug_level_release) printf("***** Wrong uart_event.event ***** \n");
        	}
        	
        }
	}
#if 0	
    while (1){
		if(uart_can_parse_now_f){
			uart_can_parse_now_f = 0;
			if(debug_uart_level_ON) printf("\n-parsing-----");
			AT_command_parse(UartToNetBuffer, &u2n_start, &u2n_end, 0 /*dummy*/);
		}else{
			vTaskDelay(100);
			if(debug_uart_level_ON) printf("\n-delay-----");
		}
    }
#endif
}


void HTC_command_tx(uint8 cmd, uint8 *payload)
{

}


void HTC_command_tx_all(void)
{}



#if 0
void ICACHE_FLASH_ATTR
uart_init(void)
{
    while (READ_PERI_REG(UART_STATUS(0)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S));

    while (READ_PERI_REG(UART_STATUS(1)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S));

    UART_ConfigTypeDef uart;

    uart.baut_rate    = BIT_RATE_74880;
    uart.data_bits    = UART_WordLength_8b;
    uart.flow_ctrl    = USART_HardwareFlowControl_None;
    // uart.exist_parity = PARITY_DIS;
    uart.parity       = USART_Parity_None;
    uart.stop_bits    = USART_StopBits_1;

    uart_config(UART0, &uart);
    uart_config(UART1, &uart);

    os_install_putc1(uart1_write_char);

    _xt_isr_unmask(1 << ETS_UART_INUM);

    xQueueUart = xQueueCreate(32, sizeof(os_event_t));

    xTaskCreate(uart_task, (uint8 const *)"uTask", 512, NULL, tskIDLE_PRIORITY + 2, &xUartTaskHandle);
}
#endif

//=================================================================

void
UART_SetWordLength(UART_Port uart_no, UART_WordLength len)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_BIT_NUM, len, UART_BIT_NUM_S);
}

void
UART_SetStopBits(UART_Port uart_no, UART_StopBits bit_num)
{
    SET_PERI_REG_BITS(UART_CONF0(uart_no), UART_STOP_BIT_NUM, bit_num, UART_STOP_BIT_NUM_S);
}

void
UART_SetLineInverse(UART_Port uart_no, UART_LineLevelInverse inverse_mask)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_LINE_INV_MASK);
    SET_PERI_REG_MASK(UART_CONF0(uart_no), inverse_mask);
}

void
UART_SetParity(UART_Port uart_no, UART_ParityMode Parity_mode)
{
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_PARITY | UART_PARITY_EN);

    if (Parity_mode == USART_Parity_None) {
    } else {
        SET_PERI_REG_MASK(UART_CONF0(uart_no), Parity_mode | UART_PARITY_EN);
    }
}

void
UART_SetBaudrate(UART_Port uart_no, uint32 baud_rate)
{
    uart_div_modify(uart_no, UART_CLK_FREQ / baud_rate);
}

//only when USART_HardwareFlowControl_RTS is set , will the rx_thresh value be set.
void
UART_SetFlowCtrl(UART_Port uart_no, UART_HwFlowCtrl flow_ctrl, uint8 rx_thresh)
{
    if (flow_ctrl & USART_HardwareFlowControl_RTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_U0RTS);
        SET_PERI_REG_BITS(UART_CONF1(uart_no), UART_RX_FLOW_THRHD, rx_thresh, UART_RX_FLOW_THRHD_S);
        SET_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF1(uart_no), UART_RX_FLOW_EN);
    }

    if (flow_ctrl & USART_HardwareFlowControl_CTS) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_UART0_CTS);
        SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    } else {
        CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_TX_FLOW_EN);
    }
}

void
UART_WaitTxFifoEmpty(UART_Port uart_no) //do not use if tx flow control enabled
{
    while (READ_PERI_REG(UART_STATUS(uart_no)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S));
}

void
UART_ResetFifo(UART_Port uart_no)
{
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);
}

void
UART_ClearIntrStatus(UART_Port uart_no, uint32 clr_mask)
{
    WRITE_PERI_REG(UART_INT_CLR(uart_no), clr_mask);
}

void
UART_SetIntrEna(UART_Port uart_no, uint32 ena_mask)
{
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), ena_mask);
}

void
UART_intr_handler_register(void *fn, void *arg)
{
    _xt_isr_attach(ETS_UART_INUM, fn, arg);
}

void
UART_SetPrintPort(UART_Port uart_no)
{
    if (uart_no == 1) {
        os_install_putc1(uart1_write_char);
    } else {
        os_install_putc1(uart0_write_char);
    }
}

void
UART_ParamConfig(UART_Port uart_no,  UART_ConfigTypeDef *pUARTConfig)
{
    if (uart_no == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } else {
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    }

    UART_SetFlowCtrl(uart_no, pUARTConfig->flow_ctrl, pUARTConfig->UART_RxFlowThresh);
    UART_SetBaudrate(uart_no, pUARTConfig->baud_rate);

    WRITE_PERI_REG(UART_CONF0(uart_no),
                   ((pUARTConfig->parity == USART_Parity_None) ? 0x0 : (UART_PARITY_EN | pUARTConfig->parity))
                   | (pUARTConfig->stop_bits << UART_STOP_BIT_NUM_S)
                   | (pUARTConfig->data_bits << UART_BIT_NUM_S)
                   | ((pUARTConfig->flow_ctrl & USART_HardwareFlowControl_CTS) ? UART_TX_FLOW_EN : 0x0)
                   | pUARTConfig->UART_InverseMask);

    UART_ResetFifo(uart_no);
}

void
UART_IntrConfig(UART_Port uart_no,  UART_IntrConfTypeDef *pUARTIntrConf)
{

    uint32 reg_val = 0;
    UART_ClearIntrStatus(uart_no, UART_INTR_MASK);
    reg_val = READ_PERI_REG(UART_CONF1(uart_no)) & ~((UART_RX_FLOW_THRHD << UART_RX_FLOW_THRHD_S) | UART_RX_FLOW_EN) ;

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_TOUT_INT_ENA) ?
                ((((pUARTIntrConf->UART_RX_TimeOutIntrThresh)&UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S) | UART_RX_TOUT_EN) : 0);

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_RXFIFO_FULL_INT_ENA) ?
                (((pUARTIntrConf->UART_RX_FifoFullIntrThresh)&UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) : 0);

    reg_val |= ((pUARTIntrConf->UART_IntrEnMask & UART_TXFIFO_EMPTY_INT_ENA) ?
                (((pUARTIntrConf->UART_TX_FifoEmptyIntrThresh)&UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S) : 0);

    WRITE_PERI_REG(UART_CONF1(uart_no), reg_val);
    CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_INTR_MASK);
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), pUARTIntrConf->UART_IntrEnMask);
}


LOCAL void
uart0_rx_intr_handler(void *para)
{
    /* uart0 and uart1 intr combine togther, when interrupt occur, see reg 0x3ff20020, bit2, bit0 represents
    * uart1 and uart0 respectively
    */
    uint8 RcvChar;
    uint8 uart_no = UART0;//UartDev.buff_uart_no;
    uint8 fifo_len = 0;
    uint8 buf_idx = 0;
#if 0
    //uint8 fifo_tmp[32] = {0};  // I saw length: 48
    uint8 fifo_tmp[128] = {0};
#endif
	my_event_t uart_event;
	//uint8 carriage_return_found = 0;	// charles222, '/r' found
	uint8 temp;
	int old_value;

    uint32 uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;  // charles222

    while (uart_intr_status != 0x0) {
        if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)) {
            //printf("FRM_ERR\r\n");
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
        } else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) {
            //printf("full\r\n");
            fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
			
			#if 0
			if(debug_uart_level_1) printf("-full:%d ", fifo_len);
			#endif
			
            buf_idx = 0;

            //while (buf_idx < fifo_len) {
            //    uart_tx_one_char(UART0, READ_PERI_REG(UART_FIFO(UART0)) & 0xFF);
            //    buf_idx++;
            //}

			while (buf_idx < fifo_len) {
                //uart_tx_one_char(UART0, READ_PERI_REG(UART_FIFO(UART0)) & 0xFF);	// charles222, don't echo to UART0
                
				temp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
				
				//printf("%c", temp);  // print to UART1
				UartToNetBuffer[u2n_end] = temp;

				old_value = u2n_end;
				
				u2n_end += 1;
				if(u2n_end == U2N_BUF_SIZE){
					u2n_end = 0;
				}

				if(u2n_end == u2n_start){
					//if(debug_level_release) printf("2nF ");		// uart 2 net buffer is Full
					u2n_end = old_value;		// back off
					break;
				}
				
                buf_idx++;
            }

            WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
        } else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) {
        
            //printf("\n-tout-\n"); //printf("tout\r\n");  // charles222
            
            fifo_len = (READ_PERI_REG(UART_STATUS(UART0)) >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
			
			#if 0
			if(debug_uart_level_1) printf("-len:%d ", fifo_len); //printf("tout\r\n");  // charles222
			#endif
			
            buf_idx = 0;

			#if 0
            while (buf_idx < fifo_len) {
                //uart_tx_one_char(UART0, READ_PERI_REG(UART_FIFO(UART0)) & 0xFF);	// charles222, don't echo to UART0
                
				uint8 temp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
				
				//printf("%c", temp);  // print to UART1
				fifo_tmp[buf_idx] = temp;

				//if(temp == '\r')  carriage_return_found = 1;
				
                buf_idx++;
            }

			put_uart_read_data_to_buffer(fifo_tmp, fifo_len);  // u2n_end updated in this function *********************
			#endif

			while (buf_idx < fifo_len) {
                //uart_tx_one_char(UART0, READ_PERI_REG(UART_FIFO(UART0)) & 0xFF);	// charles222, don't echo to UART0
                
				uint8 temp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;


				UartToNetBuffer[u2n_end] = temp;

				old_value = u2n_end;
				
				u2n_end += 1;
				if(u2n_end == U2N_BUF_SIZE){
					u2n_end = 0;
				}

				if(u2n_end == u2n_start){
					//if(debug_level_release) printf("2nF ");		// uart 2 net buffer is Full
					u2n_end = old_value;		// back off
					break;
				}
				
                buf_idx++;
            }			
			
            WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
        } else if (UART_TXFIFO_EMPTY_INT_ST == (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST)) {
            printf("empty\n\r");
            WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
            CLEAR_PERI_REG_MASK(UART_INT_ENA(UART0), UART_TXFIFO_EMPTY_INT_ENA);
        } else {
            //skip
        }

        uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;
    }

	// charles222, stupid to put printf in ISR, just try...
	//if(debug_uart_level_ON) printf("[I]");	//("\n-UART ISR-----");

}


void
uart_init_new(void)
{
    UART_WaitTxFifoEmpty(UART0);
    UART_WaitTxFifoEmpty(UART1);

    UART_ConfigTypeDef uart_config;
    uart_config.baud_rate 	= BIT_RATE_9600; 	//BIT_RATE_115200;	// charles222, PIC uses 115200 rate // BIT_RATE_74880;
    uart_config.data_bits   = UART_WordLength_8b;
    uart_config.parity      = USART_Parity_None;
    uart_config.stop_bits   = USART_StopBits_1;
    uart_config.flow_ctrl   = USART_HardwareFlowControl_None;
    uart_config.UART_RxFlowThresh = 120;
    uart_config.UART_InverseMask = UART_None_Inverse;
    UART_ParamConfig(UART0, &uart_config);
	UART_ParamConfig(UART1, &uart_config);	// charles222, add this line for UART1 config

    UART_IntrConfTypeDef uart_intr;
    uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA;
    uart_intr.UART_RX_FifoFullIntrThresh = 10;
    uart_intr.UART_RX_TimeOutIntrThresh = 2;
    uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
    UART_IntrConfig(UART0, &uart_intr);
	//UART_IntrConfig(UART1, &uart_intr);		// charles222, will cause reboot ...

    UART_SetPrintPort(UART1); //UART_SetPrintPort(UART0);  // charles222, UART1 for debug
    UART_intr_handler_register(uart0_rx_intr_handler, NULL);

	//************* charles222
	// 16 may not enough. After 100ms delay by vTaskDelay, 32 is not enough
	// Because there is 100ms delay by vTaskDelay, more Queue is needed for incoming data from UART interrupt
    my_xQueueUart = xQueueCreate(/*64*/ 32 /*16*/, sizeof(my_event_t));
    // I send the following 4 commands at the same time, and the Events in Queue can go up to 26 !!!
    // in average, every 11 byte UART ISR will add an Evevt to Queue...
    //AT+PTU_INFO=Vpa_Set 270 Vpa 24026 Ipa 400 Acpwr 9313 Itx 948 Temp 30 XS  92.22 XL 287.81
	//AT+PRU_INFO_SET=116B Vset 9000 Vmin 7000 Vmax 15000 PP 0
	//AT+PRU_INFO_RECT=116B Vrect 7901 Irect 49 Vout 5000 CP 01 CHCP 0 Mdisc 0
	//AT+PRU_INFO_SET=116B Vset 9000 Vmin 7000 Vmax 15000 PP 0
	//************************
	
    ETS_UART_INTR_ENABLE();

    /*
    UART_SetWordLength(UART0,UART_WordLength_8b);
    UART_SetStopBits(UART0,USART_StopBits_1);
    UART_SetParity(UART0,USART_Parity_None);
    UART_SetBaudrate(UART0,74880);
    UART_SetFlowCtrl(UART0,USART_HardwareFlowControl_None,0);
    */

}


//**********************************************************************************************************
// in case too many garbage data are received, need to parse out these garbage before they overflow buffer 
//**********************************************************************************************************
uint8 buffer_got_xx_bytes(int byte)
{
	if((u2n_end - u2n_start) >= byte){
		return 1;
	}

	if((u2n_end - u2n_start) < 0){
		if((u2n_end + U2N_BUF_SIZE - u2n_start) >= byte){  // half buffer size
			return 1;
		}
	}

	return 0;
}


