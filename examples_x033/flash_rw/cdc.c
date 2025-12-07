#include "cdc.h"
#include "fsusb.h"
#include "ch32fun.h"
#include <string.h>

CDC_config_t cdc;

void cdc_init(CDC_config_t * ctx) 
{
	cdc.tx_wrap_pos = UART_TX_BUF_SIZE;

	cdc.cdc_cfg[0] = (uint8_t)(UART_DEFALT_BAUD);
	cdc.cdc_cfg[1] = (uint8_t)(UART_DEFALT_BAUD >> 8);
	cdc.cdc_cfg[2] = (uint8_t)(UART_DEFALT_BAUD >> 16);
	cdc.cdc_cfg[3] = (uint8_t)(UART_DEFALT_BAUD >> 24);
	cdc.cdc_cfg[4] = UART_DEFAULT_STOPB;
	cdc.cdc_cfg[5] = UART_DEFAULT_PARITY;
	cdc.cdc_cfg[6] = 8;
	cdc.cdc_cfg[7] = UART_RX_TIMEOUT * 10;

	USBFS->UEP2_DMA = (uintptr_t)cdc_tx_buffer;
}

void cdc_process_rx(CDC_config_t * ctx) 
{
	uint32_t packlen = 0;
	int ret;

	//Są jakieś bajty do wysłania
	if (ctx->rx_remain) 
	{
		//to za pierwszym razem jest 0
		if (ctx->rxing == 0) 
		{
			//Jeśli liczba bajtów w buforze do wysłania jest większa niż rozmiar PAKIETU USB to przytnij
			if( ctx->rx_remain >= USBFS_PACKET_SIZE ) packlen = USBFS_PACKET_SIZE;
			//Minał timeout na czekanie na zapełnienie bufora to wysyłamy to co w buforze zostało
			// jeśli nie to packlen=0 i nic nie wyślemy
			else if (ctx->rx_timeout >= UART_RX_TIMEOUT) packlen = ctx->rx_remain;
			
			//jeśli packlen przekracza rozmiar bufora to przytnij aby nie wyszło poza bufor
			if (packlen > (UART_RX_BUF_SIZE - ctx->rx_pos)) packlen = (UART_RX_BUF_SIZE - ctx->rx_pos);

			if (packlen) 
			{
				NVIC_DisableIRQ(USB_IRQn);
				ctx->usb_timeout = 0;
				ret = USBFS_SendEndpointNEW(3, (uint8_t*)(uart_rx_buffer + ctx->rx_pos), packlen, 1);
				if (ret == 0) 
				{
					ctx->rxing = packlen;
					ctx->rx_remain -= packlen;
					ctx->rx_pos += packlen;
					if (ctx->rx_pos >= UART_RX_BUF_SIZE) ctx->rx_pos = 0;
					// Indicate we are done sending for now, so host can pass data further
					// https://electronics.stackexchange.com/questions/253669/usb-cdc-help-with-zero-packets
					if (packlen == USBFS_PACKET_SIZE) ctx->zero_packet_pending = 1;
				}
				NVIC_EnableIRQ(USB_IRQn);
			}
		} 
		else 
		{
			// Manually clear rxing flag if haven't receive IN request
			if (ctx->usb_timeout >= UART_USB_TIMEOUT) 
			{
				ctx->rxing = 0;
				USBFSCTX.USBFS_Endp_Busy[3] = 0;
			}
		}
	}

	if (ctx->zero_packet_pending && ctx->rxing == 0 && ctx->usb_timeout >= UART_ZERO_TIMEOUT) 
	{
		NVIC_DisableIRQ( USB_IRQn );
		ctx->usb_timeout = 0;
		ctx->rxing = 1;
		USBFS_SendACK(3, 1);
		// USBFS_SendEndpointNEW(3, (uint8_t*)(uart_rx_buffer + ctx->rx_pos), 0, 0);
		ctx->zero_packet_pending = 0;
		NVIC_EnableIRQ( USB_IRQn );
	}
}

/* System_Reset_Start_Mode */
#define Start_Mode_USER                  ((uint32_t)0x00000000)
#define Start_Mode_BOOT                  ((uint32_t)0x00004000)

void FLASH_Unlock(void)
{
    /* Authorize the FPEC of Bank1 Access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

void FLASH_Lock(void)
{
    FLASH->CTLR |= CR_LOCK_Set;
}

void SystemReset_StartMode(uint32_t Mode)
{
    FLASH_Unlock();

    FLASH->BOOT_MODEKEYR = FLASH_KEY1;
    FLASH->BOOT_MODEKEYR = FLASH_KEY2;

    FLASH->STATR &= ~(1<<14);
    if(Mode == Start_Mode_BOOT){
        FLASH->STATR |= (1<<14);
    }

    FLASH_Lock();
}


void cdc_process_tx(CDC_config_t * ctx) 
{
	//USB po DMA umieszcza dane w buforze cdc_tx_buffer
	//dane są te odbierane z edpointu 2
	//tx_remain zawiera ilośc odebranych danych i jest to ustawiane w przerwaniu
	//i jak są dane odebrane to są dopisywane do bufora i tx_remain aktualizowane
	//gdyby dane tu nie były odbierane to gdy bufor by się przepełnił
	//to po USB wysłane byłoby NAK
	
	//Są jakieś dane w buforze odebrane przez USB, i pasuje coś z nimi zrobić
	if (ctx->txing) 
	{
		//Nic nie robimy z tymi danymi tylk oodsyłamy ACK
		//i przesuwamy się po buforze
		NVIC_DisableIRQ(USB_IRQn);
		ctx->tx_remain -= ctx->txing;
		ctx->tx_pos += ctx->txing;
		if( ctx->tx_pos & 0x3 ) ctx->tx_pos = (ctx->tx_pos + 4) & ~0x3;
		ctx->txing = 0;
		if ((ctx->tx_stop) && (!ctx->tx_remain)) {
			USBFS_SendACK(2, 0);
			ctx->tx_stop = 0;
		}
		
		NVIC_EnableIRQ(USB_IRQn);
	}



	//Nie ma nowych danych odebranych przez USB, ale są jakieś wbuforze

	if (ctx->txing == 0 && ctx->tx_remain) 
	{
			NVIC_DisableIRQ(USB_IRQn);

			if (ctx->tx_wrap_pos <= ctx->tx_pos) 
			{
				ctx->tx_pos = 0; 
				ctx->tx_wrap_pos = UART_TX_BUF_SIZE;
			}

			if(*(cdc_tx_buffer + ctx->tx_pos) == 'r')
			{
				SystemReset_StartMode(Start_Mode_BOOT);
				NVIC_SystemReset();
			}

			// UART_TX_DMA->MADDR = (uint32_t)(cdc_tx_buffer + ctx->tx_pos);; // Set address for DMA to read from
			uint32_t to_send = (ctx->tx_pos + ctx->tx_remain < ctx->tx_wrap_pos)?ctx->tx_remain:(ctx->tx_wrap_pos - ctx->tx_pos);
			
			if (to_send == 0) return;
		
			ctx->txing = to_send;
			
			NVIC_EnableIRQ(USB_IRQn);
	}
}

int HandleInRequest( struct _USBState * ctx, int endp, uint8_t * data, int len )
{
	int ret = 0;  // Just NAK
	// if (usb_debug) printf("HandleInRequest - EP%d\n", endp);
	switch (endp)
	{
		case 1:
			// ret = -1; // Just ACK
			break;
		case 3:
			cdc.rxing = 0;
			// ret = -1; // ACK, without it RX was stuck in some cases, leaving for now as a reminder
			break;
	}
	return ret;
}

void HandleDataOut( struct _USBState * ctx, int endp, uint8_t * data, int len )
{
	// if (usb_debug) printf("HandleDataOut - EP%d\n", endp);
	if( endp == 0 )
	{
		ctx->USBFS_SetupReqLen = 0; // To ACK
		if( ctx->USBFS_SetupReqCode == CDC_SET_LINE_CODING )
		{
			// if (usb_debug) printf("CDC_SET_LINE_CODING\n");
			/* Save relevant parameters such as serial port baud rate */
			/* The downlinked data is processed in the endpoint 0 OUT packet, the 7 bytes of the downlink are, in order
				4 bytes: baud rate value: lowest baud rate byte, next lowest baud rate byte, next highest baud rate byte, highest baud rate byte.
				1 byte: number of stop bits (0: 1 stop bit; 1: 1.5 stop bit; 2: 2 stop bits).
				1 byte: number of parity bits (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space).
				1 byte: number of data bits (5,6,7,8,16); */

			// cdc.cdc_cfg[0] = CTRL0BUFF[0];
			// cdc.cdc_cfg[1] = CTRL0BUFF[1];
			// cdc.cdc_cfg[2] = CTRL0BUFF[2];
			// cdc.cdc_cfg[3] = CTRL0BUFF[3];
			// cdc.cdc_cfg[4] = CTRL0BUFF[4];
			// cdc.cdc_cfg[5] = CTRL0BUFF[5];
			// cdc.cdc_cfg[6] = CTRL0BUFF[6];
			// uint32_t baud = CTRL0BUFF[0];
			// baud += ((uint32_t)CTRL0BUFF[1] << 8);
			// baud += ((uint32_t)CTRL0BUFF[2] << 16);
			// baud += ((uint32_t)CTRL0BUFF[3] << 24);
			// cdc.uart->baud = baud;
			// cdc.uart->stop_bits = CTRL0BUFF[4];
			// cdc.uart->parity = CTRL0BUFF[5];
			// cdc.uart->word_length = CTRL0BUFF[6];
		}
	}
	if( endp == 2 )
	{
		if( cdc.tx_stop )
		{
			cdc.tx_stop = 0;
			return;
		}
		// printf("cdc_tx_buffer: 0=%c, 1=%c, 2=%c, 3=%c\n", cdc_tx_buffer[uart.tx_pos], cdc_tx_buffer[uart.tx_pos + 1], cdc_tx_buffer[uart.tx_pos + 2], cdc_tx_buffer[uart.tx_pos + 3]);
		int pad = 4 - USBFS->RX_LEN;
		if( pad < 0 ) pad = 0;
		uint32_t write_pos = cdc.tx_pos + cdc.tx_remain + USBFS->RX_LEN;
		if( write_pos > cdc.tx_wrap_pos ) write_pos -= cdc.tx_wrap_pos;
		if( write_pos & 0x3 )
		{
			write_pos = (write_pos + 4) & ~0x3;
			USBFS_SendNAK( 2, 0 );
			// USBFS->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
			// USBFS->UEP2_RX_CTRL |= USBFS_UEP_R_RES_NAK;
			cdc.tx_stop = 1;
		}
		// printf("[HandleDatOut] write_pos = %d, tx_pos = %d, tx_remain = %d, USBFS->RX_LEN = %d \n", write_pos, cdc.tx_pos, cdc.tx_remain, USBFS->RX_LEN);
		// if( write_pos >= uart.tx_wrap_pos) write_pos = uart.tx_remain - (uart.tx_wrap_pos - uart.tx_pos);
		// if( write_pos < uart.tx_pos && write_pos + 64 >= uart.tx_pos ) goto buffer_overflow;
		if( write_pos > ( UART_TX_BUF_SIZE - 64) )
		{
			// printf("WRAP -> write_pos = %d > 960, 961 = %c\n", write_pos, cdc_tx_buffer[961]);
			cdc.tx_wrap_pos = cdc.tx_pos + cdc.tx_remain + USBFS->RX_LEN;
			write_pos = 0;
		}
		
		// if( write_pos + len > UART_TX_BUF_SIZE - 1 )
		// {
		//   uart.tx_wrap_pos = write_pos;
		//   write_pos = 0;
		// }
		USBFS->UEP2_DMA = (uint32_t)(cdc_tx_buffer + write_pos);
		// printf("USBFS->UEP2_DMA = %08x\n", USBFS->UEP2_DMA);
		
		if( cdc.tx_remain >= ( UART_TX_BUF_SIZE ) )
		{
			// if( usb_debug ) printf("Buffer overflow, %c\n", cdc_tx_buffer[961]);
			USBFS_SendNAK( 2, 0 );
			// USBFS->UEP2_RX_CTRL &= ~USBFS_UEP_R_RES_MASK;
			// USBFS->UEP2_RX_CTRL |= USBFS_UEP_R_RES_NAK;
			cdc.tx_stop = 1;
		}
		cdc.tx_remain += USBFS->RX_LEN;
		// uart.tx_pos += USBFS->RX_LEN;
		// printf("write_pos = %d, tx_wrap_pos = %d, tx_remain = %d, txing = %d\n", write_pos, cdc.tx_wrap_pos, cdc.tx_remain, cdc.txing);
		// Prevent overwrite of the buffer
		
		// printf("\n");
		// printf("UART_TX_DMA->MADDR = %08x\n", UART_TX_DMA->MADDR);
	}
}

int HandleSetupCustom( struct _USBState * ctx, int setup_code)
{
	int ret = -1;
	if( ctx->USBFS_SetupReqType & USB_REQ_TYP_CLASS )
	{
		switch( setup_code )
		{
			case CDC_SET_LINE_CTLSTE:
				uint16_t wValue = ctx->USBFS_IndexValue;//pUSBFS_SetupReqPak->wValue;
				if (wValue&0x01)
				{
					cdc.DTR_state = 1;
				}
				else
				{
					cdc.DTR_state = 0;
				}

			case CDC_SET_LINE_CODING:
			case CDC_SEND_BREAK:
				ret = (ctx->USBFS_SetupReqLen)?ctx->USBFS_SetupReqLen:-1;

				break;
			case CDC_GET_LINE_CODING:
				ctx->pCtrlPayloadPtr = cdc.cdc_cfg;
				ret = ctx->USBFS_SetupReqLen;
				break;

			default:
				ret = 0;
				break;
		}
	}
	else if( ctx->USBFS_SetupReqType & USB_REQ_TYP_VENDOR )
	{
		/* Manufacturer request */
	}
	else
	{
		ret = 0; // Go to STALL
	}
	return ret;
}


void write_cdc(const uint8_t* data, size_t len)
{
	//Ta implementacja bazuje na buforze kołowym takim skopanym troche
	// dokładamy tylko do maks napełnienia bufora,
	//
	//potem danych nie dodajemy. Jak całośc zostanie wysłana to bufor leci od początku
	NVIC_DisableIRQ(USB_IRQn);
	if((len+cdc.rx_remain) < UART_RX_BUF_SIZE)
	{
		
		uint8_t *buf_empty_part = (uint8_t*)(uart_rx_buffer + cdc.rx_pos+cdc.rx_remain);
		cdc.rx_remain += len;
		cdc.rx_timeout = 0;
		memcpy(buf_empty_part, data, len);
		
	}
	NVIC_EnableIRQ(USB_IRQn);
}

static int __puts_cdc( char *s, int len, void *buf )
{
	(void)buf;//To zawiera wskaźnika na oryginalny string z printf

	write_cdc((uint8_t*)s, len);
	return len;
}

int printf( const char* format, ... )
{
	va_list args;
	va_start( args, format );
	int ret_status = mini_vpprintf(__puts_cdc, 0, format, args);
	va_end( args );
	return ret_status;
}

int _write(int fd, const char *buf, int size)
{
	return size;
}