/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"



#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

//Define os leds do OLED DYSPLAY

#define LED3_PIO_ID		ID_PIOB
#define LED3_PIO        PIOB
#define LED3_PIN		2
#define LED3_PIN_MASK   (1<<LED3_PIN)

#define LED1_PIO_ID		ID_PIOA
#define LED1_PIO        PIOA
#define LED1_PIN		0
#define LED1_PIN_MASK   (1<<LED1_PIN)

#define LED2_PIO_ID		ID_PIOC
#define LED2_PIO        PIOC
#define LED2_PIN		30
#define LED2_PIN_MASK   (1<<LED2_PIN)

/**
* Bot?o
*/
//Botao 1 - Pulso Mag
#define BUT1_PIO_ID				ID_PIOD
#define BUT1_PIO				PIOD
#define BUT1_PIN				28
#define BUT1_PIN_MASK			(1 << BUT1_PIN)

#define BUT2_PIO_ID				ID_PIOC
#define BUT2_PIO				PIOC
#define BUT2_PIN				31
#define BUT2_PIN_MASK			(1 << BUT2_PIN)

#define BUT3_PIO_ID				ID_PIOA
#define BUT3_PIO				PIOA
#define BUT3_PIN				19
#define BUT3_PIN_MASK			(1 << BUT3_PIN)


volatile int mag_pulses = 0;
volatile double vel = 0;
volatile double distance = 0;
volatile double raio = 1;
volatile bool flag_draw = 0;
volatile char what = 0;


static void Button1_Handler(uint32_t id, uint32_t mask)
{
	mag_pulses = mag_pulses+1;
}
static void Button2_Handler(uint32_t id, uint32_t mask)
{
	
}
static void Button3_Handler(uint32_t id, uint32_t mask)
{
	
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			
			vel = 2*3.1415*raio*mag_pulses/(4);
			distance += 2*3.1415*raio;  
			mag_pulses = 0;
			flag_draw = true;
			
			uint32_t YEAR2,MONTH2,DAY2,WEEK2,HOUR2,MINUTE2,SECOND2;
			
			rtc_get_date(RTC,&YEAR2,&MONTH2,&DAY2,&WEEK2);
			rtc_get_time(RTC,&HOUR2,&MINUTE2,&SECOND2);
			rtc_set_date_alarm(RTC, 1, MONTH2, 1, DAY2);
			rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE2, 1, SECOND2+4);	
	}
	
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
	
	
}
void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup??o */
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, Button2_Handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIN_MASK, PIO_IT_FALL_EDGE, Button3_Handler);
	
	/* habilita interrup?c?o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 2);
	NVIC_SetPriority(BUT2_PIO_ID, 3);
	NVIC_SetPriority(BUT3_PIO_ID, 4);
	
};
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	WDT->WDT_MR = WDT_MR_WDDIS;
	board_init();
	sysclk_init();
	RTC_init();

	delay_init();
	BUT_init();
	gfx_mono_ssd1306_init();
	char buffer[32];
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+4);
	
	
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		//gfx_mono_draw_string("0", 50,16, &sysfont);
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if(flag_draw){
			if(what==0){
				sprintf(&buffer,"%d",vel);
				gfx_mono_draw_string("Vel: ",5,16,&sysfont);
				gfx_mono_draw_string(buffer,50,16,&sysfont);
			}
			if(what==1){
				sprintf(&buffer,"%d",distance);
				gfx_mono_draw_string("Dist: ",16,16,&sysfont);
				gfx_mono_draw_string(buffer,50,16,&sysfont);
			}
			flag_draw = false;
			
		}
	}
}
