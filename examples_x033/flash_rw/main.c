#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"
#include "cdc.h"
#include "systick.h"

// sudo python chprog.py usbfs_cdc_uart.bin

//SPI pins
#define PIN_CS 7  // PB7
#define PIN_MISO   6  // PA6
#define SPI_SCLK  5  // PA5
#define SPI_MOSI  7  // PA7


static inline void flash_cs_low(void)
{
	GPIOB->BCR |= 1 << PIN_CS;
}

static inline void flash_cs_high(void)
{
	GPIOB->BSHR |= 1 << PIN_CS;
}

void init_spi(void)
{
	//Flash SPI init
	
	// Enable GPIO Port C and SPI peripheral
    //Peripheral clock enable register
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1;
    // PC3 - CS
    //CFGLR - Port Configuration register
    GPIOB->CFGLR &= ~(0xf << (PIN_CS << 2));
    GPIOB->CFGLR |= (GPIO_CNF_OUT_PP | GPIO_Speed_50MHz) << (PIN_CS << 2);

    // PA6 - MISO
    GPIOA->CFGLR &= ~(0xf << (PIN_MISO << 2));
    GPIOA->CFGLR |= (GPIO_CNF_IN_FLOATING) << (PIN_MISO << 2);

    // PA5 - SCLK
    GPIOA->CFGLR &= ~(0xf << (SPI_SCLK << 2));
    GPIOA->CFGLR |= (GPIO_CNF_OUT_PP_AF | GPIO_Speed_50MHz) << (SPI_SCLK << 2);

    // PA7 - MOSI
    GPIOA->CFGLR &= ~(0xf << (SPI_MOSI << 2));
    GPIOA->CFGLR |= (GPIO_CNF_OUT_PP_AF | GPIO_Speed_50MHz) << (SPI_MOSI << 2);

	// Configure SPI
    SPI1->CTLR1 = SPI_CPHA_1Edge             // Bit 0     - Clock PHAse, data sampling starts from first clock edge
                  | SPI_CPOL_Low            // Bit 1     - Clock POLarity - idles at the logical low voltage
                  | SPI_Mode_Master          // Bit 2     - Master device
                  | SPI_BaudRatePrescaler_2  // Bit 3-5   - F_HCLK / 2 = 48MHz/2=24MHz, I do not know why, but it only works with this prescaler
                  | SPI_FirstBit_MSB         // Bit 7     - MSB transmitted first
                  | SPI_NSS_Soft             // Bit 9     - Software slave management
                  | SPI_DataSize_8b          // Bit 11    - 8-bit data
                  | SPI_Direction_2Lines_FullDuplex;  // Bit 14-15 - 1-line SPI, transmission only
    SPI1->CRCR = 7;                          // CRC
    SPI1->CTLR1 |= CTLR1_SPE_Set;            // Bit 6     - Enable SPI

}

uint8_t spi_write_read(uint8_t tx_data)
{
	//TX empty status data can be send
	uint8_t counter = 0;
	while((SPI1->STATR & SPI_I2S_FLAG_TXE) == 0 && counter < 100) {Delay_Ms( 1 );counter++;}
	if(counter >=100)
	{
		printf("TX empty not set, timeout reached\n\r");
		return 0;
	}
	SPI1->DATAR = tx_data;

	counter =0;
	while((SPI1->STATR & SPI_I2S_FLAG_RXNE) != 0) {Delay_Ms( 1 );counter++;}
	if(counter >=100)
	{
		printf("RX not empty not set, timeout reached\n\r");
		return 0;
	}
	uint16_t rx_data = SPI1->DATAR;
	return (uint8_t)rx_data;
}

void flash_read_info()
{
	// spi_write_read(0);

	flash_cs_low();
	spi_write_read(0x90);
	spi_write_read(0);
	spi_write_read(0);
	spi_write_read(0);
	uint8_t manuf_id = spi_write_read(0);
	uint8_t device_id = spi_write_read(0);
	flash_cs_high();
	printf("Manuf ID: 0x%x", manuf_id);
	printf(" Device ID: 0x%x\n\r", device_id);
	if(manuf_id == 0xef && device_id == 0x16)
	{
		printf("W25Q64FV 64Mbit(8MB)\n\r");
	}
}

char print_buffer[255];

int main()
{
	SystemInit();
	systick_init();

	funGpioInitAll();
	RCC->AHBPCENR = RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;
	cdc_init(&cdc);
	USBFSSetup();
	init_spi();

	int last_delay = millis_cnt;
	funPinMode( PA9,     GPIO_CFGLR_OUT_10Mhz_PP ); // Set PIN_1 to output
	int state_led = 0;
	while(cdc.DTR_state == 0)
	{
		if((millis_cnt - last_delay) > 100)
		{
			last_delay = millis_cnt;
			if (state_led == 0)
			{
				state_led = 1;
				funDigitalWrite( PA9,     FUN_HIGH );
			}
			else
			{
				state_led = 0;
				funDigitalWrite( PA9,     FUN_LOW );
			}
		}
	}

	while(1)
	{
		if((millis_cnt - last_delay) > 1000)
		{
			last_delay = millis_cnt;
			if (state_led == 0)
			{
				state_led = 1;
				funDigitalWrite( PA9,     FUN_HIGH );
			}
			else
			{
				state_led = 0;
				funDigitalWrite( PA9,     FUN_LOW );
			}

			// int len = sprintf(print_buffer, "hello\n\r");
			// int len = sprintf(print_buffer, "Before flash read\n\r");
			// write_cdc((uint8_t*)print_buffer, len);
			printf("Before flash read\n\r");
			flash_read_info();
			printf("Hello after read\n\r");
		}

		
		cdc_process_tx(&cdc);
		cdc_process_rx(&cdc);
	}
}

