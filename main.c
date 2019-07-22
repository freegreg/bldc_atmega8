/*
 * test_motor.c
 *
 * Created: 7/8/2019 23:21:58
 * Author : gutu
 */ 
#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart/uart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define PIN_U_L			2
#define PORT_U_L		PORTD
#define DDR_U_L			DDRD

#define PIN_W_L			3
#define PORT_W_L		PORTD
#define DDR_W_L			DDRD

#define PIN_V_L			7
#define PORT_V_L		PORTD
#define DDR_V_L			DDRD

#define U_H_EN TCCR2 |= (1 << COM21)
#define U_H_DIS TCCR2 &= ~(1 << COM21)
#define U_L_EN PORT_U_L |= (1 << PIN_U_L)
#define U_L_DIS PORT_U_L &= ~(1 << PIN_U_L)

#define W_H_EN TCCR1A |= (1 << COM1B1)
#define W_H_DIS TCCR1A &= ~(1 << COM1B1)
#define W_L_EN PORT_W_L |= (1 << PIN_W_L)
#define W_L_DIS PORT_W_L &= ~(1 << PIN_W_L)

#define V_H_EN TCCR1A |= (1 << COM1A1)
#define V_H_DIS TCCR1A &= ~(1 << COM1A1)
#define V_L_EN PORT_V_L |= (1 << PIN_V_L)
#define V_L_DIS PORT_V_L &= ~(1 << PIN_V_L)

#define DIS_ALL U_H_DIS;U_L_DIS;W_H_DIS;W_L_DIS;V_H_DIS;V_L_DIS;
#define INIT_ALL DDR_U_L |= (1 << PIN_U_L);\
DDR_W_L |= (1 << PIN_W_L);\
DDR_V_L |= (1 << PIN_V_L);

/* Define UART buad rate here */
#define UART_BAUD_RATE      38400

bool bldcRunning(void);
void bldcStartUp(void);
void setPwm(uint8_t pwm);
void timersPwmInit(void);
void acInit(void);

void uartTerminator(char uart_ch);

typedef enum _bldcPhase{
	PHASE_U = 0,
	PHASE_W,
	PHASE_V
} BldcPhase;


volatile bool bldcDirection = false;
volatile bool changeBldcDirection = false;

volatile int bldcStep = 0;


volatile long long int bldcStartUpDelay = 0;
volatile long long int bldcZcLag = 0;
volatile bool zcCapturred = false;

volatile unsigned long long int stepCounter = 0;
volatile unsigned long long int speedTimerCounter = 0;
volatile unsigned int bldcSpeedRpm = 0;
enum {
	BLDC_WAIT,
	BLDC_STARTUP,
	BLDC_RUNNING,
	BLDC_STOP
} bldcState = BLDC_STARTUP;

volatile BldcPhase bldcPhase = 0;

void delayus(uint64_t delay){
	while(--delay != 0){
		asm("nop");
		asm("nop");
		asm("nop");

	}
}
char str_help[100];
int main(void)
{
	DIS_ALL;
	INIT_ALL;
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	timersPwmInit();
	acInit();
	sei();
	uart_puts("MAIN_LOOP START\n");
    /* Replace with your application code */
	uint64_t stepDelay = 2000;
    while (1) 
    {
		int uart = uart_getc();
		if (uart != UART_NO_DATA){
			char uar = uart & 0x00FF;
			uartTerminator(uar);
		}
		switch(bldcState){
				case BLDC_WAIT:
					_delay_ms(10);
				break;
				case BLDC_STARTUP:
					//if (bldcStartUpDelay++ < 5000){
					if (stepDelay > 100){
							stepDelay -= 5;
						//}
						setPwm(150);
						bldcStartUp();
						delayus(stepDelay);
						
					}
					else{
						bldcState = BLDC_RUNNING;
						sprintf(str_help, "BLDC_RUNNING\n");
						uart_puts(str_help);
						bldcStartUpDelay = 0;
						stepDelay = 2000;
						ACSR  |=  (1 << ACIE); //Enalbe AC interrupt
					}
				break;
				case BLDC_RUNNING:
					if (!bldcRunning())
						bldcState = BLDC_STOP;
					if (changeBldcDirection){
						bldcState = BLDC_STOP;
					}
						_delay_ms(10);
				break;
				case BLDC_STOP:
					uart_puts("BLDC_STOP\n");
					ACSR  &=  ~(1 << ACIE); //Enalbe AC interrupt
					DIS_ALL;
					_delay_ms(10);
					DIS_ALL;
					bldcStep = 0;
					bldcStartUpDelay = 0;
					bldcZcLag = 0;
					zcCapturred = false;
					if (changeBldcDirection){
						changeBldcDirection = false;
						bldcDirection = !bldcDirection;
						_delay_ms(100);
						bldcState = BLDC_STARTUP;
					}else {
						bldcState = BLDC_WAIT;
					}
					
				break;
				default:
					bldcState = BLDC_STOP;
				break;
				}
				
    }
}

#define BUFF_LENGTH 3
char buff_uart[BUFF_LENGTH];
void uartTerminator(char uart_ch){
	static int curr_ur = 0;
	if (uart_ch == 's'){
		bldcState = BLDC_STARTUP;
	}else if (uart_ch == 'r'){
		bldcState = BLDC_STARTUP;
	}else if (uart_ch == 'x'){
		bldcState = BLDC_STOP;
	}else if (uart_ch == 'w'){
		changeBldcDirection = true;
	}
	 if (uart_ch == '\n' && curr_ur > 0){
		 uint8_t pwm =  ((curr_ur > 2) ? (100 * (buff_uart[curr_ur-3]-'0')) : 0) +
		 ((curr_ur > 1) ?  (10 * (buff_uart[curr_ur-2]-'0')) : 0) +
		 (buff_uart[curr_ur - 1]-'0');
		 setPwm(pwm);
		 sprintf(str_help, "PWM: %d\n", pwm);
		 uart_puts(str_help);
		 curr_ur = 0;
	 }
	if (curr_ur > BUFF_LENGTH)
		curr_ur = 0;
	if (uart_ch >= '0' && uart_ch <= '9'){
		buff_uart[curr_ur++] = uart_ch;
	}
}

void timersPwmInit(void){
	/* Init Timers Outputs */
	PORTB &= ~0x0E;
	DDRB |= 0x0E;
	/* Init timer 1 */
	// Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
	OCR1A = 0x00;
	OCR1B = 0x00;
	TCCR1A = (1 << WGM10) | (0 << COM1A1) | (0 << COM1B1);
	TCCR1B = (1 << CS10);
	// Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
	OCR2 = 0x00;
	TCCR2 = (1 << WGM20) | (1 << CS20) | (0 << COM21);
	
	//Timer1 Enable Overflow Interrupt for Speed Measure
	TIMSK != (1 << TOIE1);

}

void setPwm(uint8_t pwm){
	OCR1A = pwm;
	OCR1B = pwm;
	OCR2 = pwm;
}

void acInit(void){
	//ACD ACBG ACO ACI ACIE ACIC ACISn[1:0] 
	  // Analog comparator setting
	  ACSR   = (1 << ACI);           // Disable and clear (flag bit) analog comparator interrupt

      ADCSRA = (0 << ADEN);   // Disable the ADC module
	  SFIOR = (1 << ACME);			//?Analog Comparator Multiplexer Enable
}



void checkZCPhase(BldcPhase phase){
	switch(phase){
		case PHASE_U:
			//ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
			ADMUX = (5<<MUX0);
		break;
		case PHASE_W:
			//ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
			ADMUX = (4<<MUX0);
		break;
		case PHASE_V:
			//ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
			ADMUX = (3<<MUX0);
		break;
		default:
			//ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0));
			ADMUX = (5<<MUX0);
			bldcPhase = PHASE_U;
		break;
	}
}

void bldcNextStepZC(void){
	stepCounter++;
	switch(bldcStep){
		case 0:
			//uv
			U_H_EN;
			V_L_EN;
			checkZCPhase(PHASE_W);
			if (bldcDirection){
				ACSR  |= (1 << ACIS1);
				ACSR   &= ~(1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
			else{
				ACSR  |= (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Rising Output Edge
			}
		break;
		case 1:
			//wv
			W_H_EN;
			V_L_EN;
			checkZCPhase(PHASE_U);
			if (bldcDirection){
				ACSR  |= (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
			else{
				ACSR  |= (1 << ACIS1);
				ACSR   &= ~(1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
		break;
		case 2:
			//wu
			W_H_EN;
			U_L_EN;
			checkZCPhase(PHASE_V);
			if (bldcDirection){
				ACSR  |= (1 << ACIS1);
				ACSR   &= ~(1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
			else{
				ACSR  |= (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Rising Output Edge
			}
		break;
		case 3:
			//vu
			V_H_EN;
			U_L_EN;
			checkZCPhase(PHASE_W);
			if (bldcDirection){
				ACSR  |= (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
			else{
				ACSR  |= (1 << ACIS1);
				ACSR   &= ~(1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
		break;
		case 4:
			//vw
			V_H_EN;
			W_L_EN;
			checkZCPhase(PHASE_U);
			if (bldcDirection){
				ACSR  |= (1 << ACIS1);
				ACSR   &= ~(1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
			else{
				ACSR  |= (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Rising Output Edge
			}
		break;
		case 5:
			//uw
			U_H_EN;
			W_L_EN;
			checkZCPhase(PHASE_V);
			if (bldcDirection){
				ACSR  |= (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
			else{
				ACSR  |= (1 << ACIS1);
				ACSR   &= ~(1 << ACIS0);//Comparator Interrupt on Falling Output Edge
			}
		break;
		default:
			bldcStep = 0;
		break;
	}
}


void bldcStartUp(void){
	if (bldcDirection){
		if (--bldcStep < 0) bldcStep = 5;
	}
	else{
		if (++bldcStep >= 6) bldcStep = 0;
	}
	
	DIS_ALL;
	bldcNextStepZC();
}




bool bldcRunning(void){
	if (stepCounter != 0)
		bldcSpeedRpm = speedTimerCounter/stepCounter;
	if (zcCapturred){
		bldcZcLag = 0;
		zcCapturred = false;
	}
	else if (++bldcZcLag > 200){
		return false;
	}
	return true;
}


// Analog comparator ISR
ISR (ANA_COMP_vect) {
	// BEMF debounce
	for(int i = 0; i < 10; i++) {
if (bldcDirection){
		if(bldcStep & 1){
			if((ACSR & (1 << ACO))) i -= 1;
		}
		else {
			if(!(ACSR & (1 << ACO)))  i -= 1;
		}
}
else{
		if(bldcStep & 1){
			if(!(ACSR & (1 << ACO))) i -= 1;
		}
		else {
			if((ACSR & (1 << ACO)))  i -= 1;
		}
}
	}
if (bldcDirection){
	if (--bldcStep < 0) bldcStep = 5;
}
else{
	if (++bldcStep >= 6) bldcStep = 0;
}

	DIS_ALL;
	bldcNextStepZC();
	zcCapturred = true;
}

ISR (TIMER1_OVF_vect) {
	speedTimerCounter++;
}