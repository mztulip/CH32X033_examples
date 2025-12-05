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

	USBFS->UEP2_DMA = (uintptr_t)uart_tx_buffer;
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

void cdc_process_tx(CDC_config_t * ctx) 
{
	//USB po DMA umieszcza dane w buforze uart_tx_buffer
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

			// UART_TX_DMA->MADDR = (uint32_t)(uart_tx_buffer + ctx->tx_pos);; // Set address for DMA to read from
			uint32_t to_send = (ctx->tx_pos + ctx->tx_remain < ctx->tx_wrap_pos)?ctx->tx_remain:(ctx->tx_wrap_pos - ctx->tx_pos);
			
			if (to_send == 0) return;
		
			ctx->txing = to_send;
			
			NVIC_EnableIRQ(USB_IRQn);
	}
}
