
volatile uint64_t millis_cnt = 0;

void systick_init(void)
{

	SysTick->CTLR = 0;
	SysTick->CMP = DELAY_MS_TIME - 1;
	SysTick->CNT = 0;
	SysTick->CTLR |= SYSTICK_CTLR_STE |  // Enable Counter
                  SYSTICK_CTLR_STIE |  // Enable Interrupts
                  SYSTICK_CTLR_STCLK;  // Set Clock Source to HCLK/1
	
	cdc.rx_timeout = 0;
	cdc.usb_timeout = 0;
	millis_cnt = 0;

	NVIC_EnableIRQ(SysTick_IRQn);
}

void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	SysTick->CMP += DELAY_MS_TIME;
	SysTick->SR = 0;

	cdc.rx_timeout++;
	cdc.usb_timeout++;
	millis_cnt++;
}