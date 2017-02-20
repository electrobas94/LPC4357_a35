
#include "lpc43xx_uart.h"
#include "lpc43xx_cgu.h"
#include "lpc43xx_scu.h"

#define UART_RING_BUFSIZE 256

/* Buf mask */
#define __BUF_MASK (UART_RING_BUFSIZE-1)
/* Check buf is full or not */
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))
/* Check buf will be full in next receiving or not */
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))
/* Check buf is empty */
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))
/* Reset buf */
#define __BUF_RESET(bufidx)	(bufidx=0)
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)


/************************** PRIVATE TYPES *************************/

/** @brief UART Ring buffer structure */
typedef struct
{
    __IO uint32_t tx_head;                /*!< UART Tx ring buffer head index */
    __IO uint32_t tx_tail;                /*!< UART Tx ring buffer tail index */
    __IO uint32_t rx_head;                /*!< UART Rx ring buffer head index */
    __IO uint32_t rx_tail;                /*!< UART Rx ring buffer tail index */
    __IO uint8_t  tx[UART_RING_BUFSIZE];  /*!< UART Tx data ring buffer */
    __IO uint8_t  rx[UART_RING_BUFSIZE];  /*!< UART Rx data ring buffer */
} UART_RING_BUFFER_T;


/************************** PRIVATE VARIABLES *************************/
uint8_t menu1[] = "Hello NXP Semiconductors \n\r";
uint8_t menu2[] =
"UART interrupt mode demo using ring buffer \n\r\t "
"MCU LPC43xx - ARM Cortex-M4+M0 \n\r\t "
"USART3 - 9600bps \n\r";
uint8_t menu3[] = "UART demo terminated!\n";

// UART Ring buffer
UART_RING_BUFFER_T rb;

// Current Tx Interrupt enable state
__IO FlagStatus TxIntStat;

void USART3_IRQHandler(void);
void UART_IntErr(uint8_t bLSErrType);
void UART_IntTransmit(void);
void UART_IntReceive(void);

uint32_t UARTReceive(LPC_USARTn_Type *UARTPort, uint8_t *rxbuf, uint8_t buflen);
uint32_t UARTSend(LPC_USARTn_Type *UARTPort, uint8_t *txbuf, uint8_t buflen);
void print_menu(void);

int c_entry(void)
{
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;

	uint32_t idx, len;
	__IO FlagStatus exitflag;
	uint8_t buffer[10];

	SystemInit();
	CGU_Init();

	//scu_pinmux(0xC ,13 , MD_PDN, FUNC2); 	// PC.13 : USART3_TXD
	//scu_pinmux(0xC ,14 , (MD_PLN|MD_EZI|MD_ZI), FUNC2); 	// PC.14 : USART3_RXD


	scu_pinmux(0x2 ,3 , MD_PDN, FUNC2); 	// PC.13 : UART1_TXD
	scu_pinmux(0x2 ,4 , MD_PLN|MD_EZI|MD_ZI, FUNC2); 	// PC.14 : UART1_RXD
	
	
	/* Initialize UART Configuration parameter structure to default state:
	 * Baudrate = 9600bps
	 * 8 data bit
	 * 1 Stop bit
	 * None parity
	 */
	UART_ConfigStructInit(&UARTConfigStruct);

	UART_Init((LPC_USARTn_Type *)LPC_USART3, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	UART_FIFOConfig((LPC_USARTn_Type *)LPC_USART3, &UARTFIFOConfigStruct);
	UART_TxCmd((LPC_USARTn_Type *)LPC_USART3, ENABLE);

	UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_RBR, ENABLE);
	UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_RLS, ENABLE);

	TxIntStat = RESET;

	// Reset ring buf head and tail idx
	__BUF_RESET(rb.rx_head);
	__BUF_RESET(rb.rx_tail);
	__BUF_RESET(rb.tx_head);
	__BUF_RESET(rb.tx_tail);

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(USART3_IRQn, ((0x01<<3)|0x01));
	/* Enable Interrupt for USART3 channel */
    NVIC_EnableIRQ(USART3_IRQn);


	// print welcome screen
	print_menu();

	// reset exit flag
	exitflag = RESET;

    /* Read some data from the buffer */
    while (exitflag == RESET)
    {
       len = 0;
        while (len == 0)
        {
            len = UARTReceive((LPC_USARTn_Type *)LPC_USART3, buffer, sizeof(buffer));
        }

        /* Got some data */
        idx = 0;
        while (idx < len)
        {
            if (buffer[idx] == 27)
            {
                /* ESC key, set exit flag */
            	UARTSend((LPC_USARTn_Type *)LPC_USART3, menu3, sizeof(menu3));
                exitflag = SET;
            }
            else if (buffer[idx] == 'r')
            {
                print_menu();
            }
            else
            {
                /* Echo it back */
            	UARTSend((LPC_USARTn_Type *)LPC_USART3, &buffer[idx], 1);
            }
            idx++;
        }
    }

    // wait for current transmission complete - THR must be empty
    while (UART_CheckBusy((LPC_USARTn_Type *)LPC_USART3));

    // DeInitialize USART3 peripheral
    UART_DeInit((LPC_USARTn_Type *)LPC_USART3);

    /* Loop forever */
    while(1);
}

int main(void)
{
    return c_entry();
}

void UART3_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = LPC_USART3->IIR;
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = LPC_USART3->LSR;
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}
	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			UART_IntReceive();
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit();
	}

}

void UART_IntReceive(void)
{
	uint8_t tmpc;
	uint32_t rLen;

	while(1){
		// Call UART read function in UART driver
		rLen = UART_Receive((LPC_USARTn_Type *)LPC_USART3, &tmpc, 1, NONE_BLOCKING);
		// If data received
		if (rLen)
			{
			if (!__BUF_IS_FULL(rb.rx_head,rb.rx_tail)){
				rb.rx[rb.rx_head] = tmpc;
				__BUF_INCR(rb.rx_head);
			}
		}
		// no more data
		else {
			break;
		}
	}
}

void UART_IntTransmit(void)
{
	
    UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_THRE, DISABLE);
	
    while (UART_CheckBusy((LPC_USARTn_Type *)LPC_USART3) == SET);

	while (!__BUF_IS_EMPTY(rb.tx_head,rb.tx_tail))
    {
        /* Move a piece of data into the transmit FIFO */
    	if (UART_Send((LPC_USARTn_Type *)LPC_USART3, (uint8_t *)&rb.tx[rb.tx_tail], 1, NONE_BLOCKING)){
        /* Update transmit ring FIFO tail pointer */
        __BUF_INCR(rb.tx_tail);
    	} else {
    		break;
    	}
    }

    /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
	if (__BUF_IS_EMPTY(rb.tx_head, rb.tx_tail)) {
    	UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_THRE, DISABLE);
    	// Reset Tx Interrupt state
    	TxIntStat = RESET;
    }
    else{
      	// Set Tx Interrupt state
		TxIntStat = SET;
    	UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_THRE, ENABLE);
    }
}

void UART_IntErr(uint8_t bLSErrType)
{
	uint8_t test;

	while (1)
	{
		test = bLSErrType;
		test = test;
	}
}

uint32_t UARTSend(LPC_USARTn_Type *UARTPort, uint8_t *txbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) txbuf;
    uint32_t bytes = 0;

    UART_IntConfig(UARTPort, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	while ((buflen > 0) && (!__BUF_IS_FULL(rb.tx_head, rb.tx_tail)))
	{
		/* Write data from buffer into ring buffer */
		rb.tx[rb.tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb.tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	if (TxIntStat == RESET) {
		UART_IntTransmit();
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		UART_IntConfig(UARTPort, UART_INTCFG_THRE, ENABLE);
	}

    return bytes;
}

uint32_t UARTReceive(LPC_USARTn_Type *UARTPort, uint8_t *rxbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) rxbuf;
    uint32_t bytes = 0;
	
	UART_IntConfig(UARTPort, UART_INTCFG_RBR, DISABLE);

	while ((buflen > 0) && (!(__BUF_IS_EMPTY(rb.rx_head, rb.rx_tail))))
	{
		/* Read data from ring buffer into user buffer */
		*data = rb.rx[rb.rx_tail];
		data++;

		/* Update tail pointer */
		__BUF_INCR(rb.rx_tail);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/* Re-enable UART interrupts */
	UART_IntConfig(UARTPort, UART_INTCFG_RBR, ENABLE);

    return bytes;
}

void print_menu(void)
{
	uint32_t tmp, tmp2;
	uint8_t *pDat;

	tmp = sizeof(menu1);
	tmp2 = 0;
	pDat = (uint8_t *)&menu1[0];
	while(tmp) {
		tmp2 = UARTSend((LPC_USARTn_Type *)LPC_USART3, pDat, tmp);
		pDat += tmp2;
		tmp -= tmp2;
	}

	tmp = sizeof(menu2);
	tmp2 = 0;
	pDat = (uint8_t *)&menu2[0];
	while(tmp) {
		tmp2 = UARTSend((LPC_USARTn_Type *)LPC_USART3, pDat, tmp);
		pDat += tmp2;
		tmp -= tmp2;
	}
}
