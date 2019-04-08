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
volatile int vel = 0;
volatile int distance = 0;
volatile double raio = 1;
volatile bool flag_draw = 0;
volatile char what = 0;
volatile Bool f_rtt_alarme = false;
volatile int temps = 0;



static void Button3_Handler(uint32_t id, uint32_t mask)
{
	mag_pulses = mag_pulses+1;
}
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	//play pause
}
static void Button2_Handler(uint32_t id, uint32_t mask)
{
	what =what+1;
	if (what > 2){
		what = 0;
	}
	flag_draw = true;
}
static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}
void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		
	vel = 3.6*2*3.1415*raio*mag_pulses/(4);
	distance = 2*3.1415*raio*mag_pulses + distance;
	mag_pulses = 0;
	flag_draw = true;
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		   
		flag_draw = true;
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}


void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		temps = temps+1;
		flag_draw = true;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	
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

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

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
	rtc_enable_interrupt(RTC, RTC_IER_SECEN);

}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	WDT->WDT_MR = WDT_MR_WDDIS;
	board_init();
	sysclk_init();
	f_rtt_alarme = true;
	RTC_init();

	delay_init();
	BUT_init();
	gfx_mono_ssd1306_init();
	char buffer[64];
		
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		//gfx_mono_draw_string("0", 50,16, &sysfont);
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if(flag_draw){
			//gfx_mono_draw_string("                   ",0,16,&sysfont);
			if(what==0){
				sprintf(&buffer,"%d km/h  ",vel);
				gfx_mono_draw_string("Vel  ",1,16,&sysfont);
				gfx_mono_draw_string(buffer,40,16,&sysfont);
			
			}
			else if(what==1){
				sprintf(&buffer,"%d m",distance);
				gfx_mono_draw_string("Mts ",1,16,&sysfont);
				gfx_mono_draw_string(buffer,40,16,&sysfont);
			}
			else if(what==2){
				int hour = temps/360;
				int minut = (temps%360)/60;
				int seconds = (temps%60);		
				sprintf(buffer,"%02d:%02d:%02d",hour, minut, seconds);
				gfx_mono_draw_string(buffer,0,16,&sysfont);
			}
			flag_draw = false;
		}
		if (f_rtt_alarme){
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 8;
			RTT_init(pllPreScale, irqRTTvalue);
			f_rtt_alarme = false;
		}
		
	}
	
	
	
	
}
