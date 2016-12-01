/*
 *  Copyright (C) 2010 -2011  Espressif System
 *
 */

#ifndef __UART_H__
#define __UART_H__

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "freertos/queue.h"



#define ETS_UART_INTR_ENABLE()  _xt_isr_unmask(1 << ETS_UART_INUM)
#define ETS_UART_INTR_DISABLE() _xt_isr_mask(1 << ETS_UART_INUM)
#define UART_INTR_MASK          0x1ff
#define UART_LINE_INV_MASK      (0x3f<<19)

#define UART_EVENT_RX_DATA_PROCESS			0
//#define UART_EVENT_TX_PTU_SUBSTATE_0		1
//#define UART_EVENT_TX_PTU_SUBSTATE_1		2
#define UART_EVENT_TX_SEND					3
#define UART_EVENT_TX_PUT_LOCK_UNLOCK		4

#define UART_EVENT_TX_PUT_BLOCK_ALLOW		11

#define UART_EVENT_TX_RESEND				31  



#define BLE_ADDR_LEN						6	// BLE address

typedef struct _my_event_ {
    uint8 event;
    int my_x2x_end;  // buffer's end index. n2u or u2n
    //uint8 e_message;	// message, or command OP code
    //uint8 address[BLE_ADDR_LEN];
} my_event_t;

xQueueHandle my_xQueueUart;

#define BUFFER_HALF_FULL (U2N_BUF_SIZE >> 1)

void my_uart_task(void *pvParameters);

uint8 buffer_got_xx_bytes(int byte);
void HTC_command_tx(uint8 cmd, uint8 *payload);
void HTC_command_tx_all(void);
void HTC_command_tx_step_by_step_test(void);

int8 htc_fw_ver_received;
int8 htc_spec_ver_received;


#define CHARGER_STEP_INITIAL_STEP				0	// wait restart command
#define CHARGER_STEP_VERSION_STEP				5	// version
#define CHARGER_STEP_PTU_OFF_STEP				10
#define CHARGER_STEP_PTU_ON_STEP				20
#define CHARGER_STEP_PTU_LOCK_STEP				30
#define CHARGER_STEP_PTU_PUT_PRU_STEP			40	// prompt user to put PRU; wait PRU detected
#define CHARGER_STEP_PTU_UNLOCK_STEP			50	
#define CHARGER_STEP_CONNECT_STEP				55	// wait for PTU PRU charge start/connect
#define CHARGER_STEP_MEASURE_STEP				60	// ID, and PTU and PRU measure power, volt, current, temperature
#define CHARGER_STEP_PTU_REMOVE_PRU_STEP		70	// prompt user to remove PRU; wait PTU PRU charge stop
#define CHARGER_STEP_PASS_STEP					100


typedef enum {
    UART_WordLength_5b = 0x0,
    UART_WordLength_6b = 0x1,
    UART_WordLength_7b = 0x2,
    UART_WordLength_8b = 0x3
} UART_WordLength;

typedef enum {
    USART_StopBits_1   = 0x1,
    USART_StopBits_1_5 = 0x2,
    USART_StopBits_2   = 0x3,
} UART_StopBits;

typedef enum {
    UART0 = 0x0,
    UART1 = 0x1,
} UART_Port;

typedef enum {
    USART_Parity_None = 0x2,
    USART_Parity_Even = 0x0,
    USART_Parity_Odd  = 0x1
} UART_ParityMode;

typedef enum {
    PARITY_DIS = 0x0,
    PARITY_EN  = 0x2
} UartExistParity;

typedef enum {
    BIT_RATE_300     = 300,
    BIT_RATE_600     = 600,
    BIT_RATE_1200    = 1200,
    BIT_RATE_2400    = 2400,
    BIT_RATE_4800    = 4800,
    BIT_RATE_9600    = 9600,
    BIT_RATE_19200   = 19200,
    BIT_RATE_38400   = 38400,
    BIT_RATE_57600   = 57600,
    BIT_RATE_74880   = 74880,
    BIT_RATE_115200  = 115200,
    BIT_RATE_230400  = 230400,
    BIT_RATE_460800  = 460800,
    BIT_RATE_921600  = 921600,
    BIT_RATE_1843200 = 1843200,
    BIT_RATE_3686400 = 3686400,
} UART_BautRate; //you can add any rate you need in this range

typedef enum {
    USART_HardwareFlowControl_None    = 0x0,
    USART_HardwareFlowControl_RTS     = 0x1,
    USART_HardwareFlowControl_CTS     = 0x2,
    USART_HardwareFlowControl_CTS_RTS = 0x3
} UART_HwFlowCtrl;

typedef enum {
    UART_None_Inverse = 0x0,
    UART_Rxd_Inverse  = UART_RXD_INV,
    UART_CTS_Inverse  = UART_CTS_INV,
    UART_Txd_Inverse  = UART_TXD_INV,
    UART_RTS_Inverse  = UART_RTS_INV,
} UART_LineLevelInverse;

typedef struct {
    UART_BautRate   baud_rate;
    UART_WordLength data_bits;
    UART_ParityMode parity;    // chip size in byte
    UART_StopBits   stop_bits;
    UART_HwFlowCtrl flow_ctrl;
    uint8           UART_RxFlowThresh ;
    uint32          UART_InverseMask;
} UART_ConfigTypeDef;

typedef struct {
    uint32 UART_IntrEnMask;
    uint8  UART_RX_TimeOutIntrThresh;
    uint8  UART_TX_FifoEmptyIntrThresh;
    uint8  UART_RX_FifoFullIntrThresh;
} UART_IntrConfTypeDef;


//=======================================
/**  
  * @brief   Wait uart tx fifo empty, do not use it if tx flow control enabled.
  * 
  * @param   UART_Port uart_no:UART0 or UART1
  *  
  * @return  null
  */
void UART_WaitTxFifoEmpty(UART_Port uart_no); //do not use if tx flow control enabled
/**  
  * @brief   Clear uart tx fifo and rx fifo.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  *  
  * @return  null
  */
void UART_ResetFifo(UART_Port uart_no);
/**  
  * @brief  Clear uart interrupt flags.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   uint32 clr_mask : To clear the interrupt bits
  *  
  * @return  null
  */
void UART_ClearIntrStatus(UART_Port uart_no, uint32 clr_mask);
/**  
  * @brief   Enable uart interrupts .
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   uint32 ena_mask : To enable the interrupt bits
  *  
  * @return  null
  */
void UART_SetIntrEna(UART_Port uart_no, uint32 ena_mask);
/**  
  * @brief   Register an application-specific interrupt handler for Uarts interrupts.
  * 
  * @param   void *fn : interrupt handler for Uart interrupts.
  * @param   void *arg : interrupt handler's arg.
  *  
  * @return  null
  */
void UART_intr_handler_register(void *fn, void *arg);
/**  
  * @brief   Config from which serial output printf function.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  *  
  * @return  null
  */
void UART_SetPrintPort(UART_Port uart_no);
/**  
  * @brief   Config Common parameters of serial ports.
  * 
  * @param   UART_Port uart_no : UART0 or UART1
  * @param   UART_ConfigTypeDef *pUARTConfig : parameters structure
  *  
  * @return  null
  */
void UART_ParamConfig(UART_Port uart_no,  UART_ConfigTypeDef *pUARTConfig);
void UART_IntrConfig(UART_Port uart_no,  UART_IntrConfTypeDef *pUARTIntrConf);
void UART_SetWordLength(UART_Port uart_no, UART_WordLength len);
void UART_SetStopBits(UART_Port uart_no, UART_StopBits bit_num);
void UART_SetParity(UART_Port uart_no, UART_ParityMode Parity_mode) ;
void UART_SetBaudrate(UART_Port uart_no, uint32 baud_rate);
void UART_SetFlowCtrl(UART_Port uart_no, UART_HwFlowCtrl flow_ctrl, uint8 rx_thresh);
void UART_SetLineInverse(UART_Port uart_no, UART_LineLevelInverse inverse_mask) ;
void uart_init_new(void);
STATUS uart_tx_string(uint8 uart, uint8 *TxString);  // charles222
#endif
