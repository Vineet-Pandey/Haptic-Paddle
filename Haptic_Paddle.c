/*
 * Closed Loop Position and Velocity Control for Haptic effect
 *
 * Created: 4/20/2018 6:34:15 PM
 * Author : vineet
 */ 

#define F_CPU 16000000L
#define BAUD 9600 // Define baud rate
#define VREF 5 // Reference voltage
#include <stdbool.h>
#include <util/setbaud.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define ADC_PIN 1 // Defining which analog pin to read
//Declaring variables
float curr_Deg;
float curr_Vel;
float curr_Vol;
float curr_Error;
float prev_Vol;
float prev_Deg;
float prev_Vel;
float prev_Error;
float d_error;
float i_error;
float vel_Out;
float vel_1;
float vel_2;
float fvel_1;
float fvel_2;
float fvel;
int dutyCycle;

float control = 0;
float u;
float Kp = 0.5; // Position Control Values
float Kd = 0.005;
int AREF = 45;
//float Kp = 0.001; // Velocity Control Values
//float Kd = 0.000001;
float Ki = 0.0000000001;
 int vel_ref = 2000;
 float Kspring = 1.1; // Virtual Spring Values
// int spring_ref = 0;
float Bdamp = 0.15; // Viscous Damping Values
float Kwall = 8; // Virtual Wall Values
float Bwall = 20;
//Tested saturation values
float sat_High = 180;
float sat_Low = 75;
// Initializing UART
void uart_init (void)
{
	UBRR0H = UBRRH_VALUE; // enabled by setbaud.h
	UBRR0L = UBRRL_VALUE;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0); // enable receiver and transmitter
	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01); // set frame format: 1 stop bit, 8bit data
}
// Data transmit function
void uart_transmit (unsigned char data) //(unsigned char data)
{
	while (!(UCSR0A & (1<<UDRE0))); // wait for empty transmit buffer
	UDR0 = data; // load data in the register, which sends it
}
// Data receive function
unsigned char uart_receive (void)
{
	while ( !(UCSR0A & (1<<RXC0)) ); // wait for data to be received
	return UDR0; // get and return 8-bit data from buffer
}
// Float transmit function
void printfloat(float number)
{
	uint8_t *p = (uint8_t *)&number;
	uart_transmit(p[0]);
	uart_transmit(p[1]);
	uart_transmit(p[2]);
	uart_transmit(p[3]);
}
// Initializing ADC
void InitADC()
{
	ADMUX|=(1<<REFS0); // Sets 5v internal source as reference voltage
	ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADEN enables the ADC, ADPS0~2 sets pre-scaler
}
// Function that reads the ADC value from the designated channel
float ReadADC(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	return ADC;
}
// Function that converts the voltage reading into degrees
float Vol2D(float v)
{
	float angular_pos;
	if (v>=1.59){ // According to the straight line fit taking 10 data points.
	angular_pos = 87.4*v - 317;}
	else {
	angular_pos = 95.4*v + 29.69;}
	if(angular_pos > 180)
	{
		return angular_pos = angular_pos - 360;
	}
	else
	{
		return angular_pos = angular_pos;
	}
}
// Function that calculates the angular velocity
float D2Vel(float curr_Deg, float prev_Deg)
{
	return (curr_Deg - prev_Deg)/0.01;
}
// Apply filter (Butterworth filter)
float butter(float t, float t_step1, float t_step2,float tf_step1, float tf_step2)
{
	// Cutoff Frequency = 25 Hz.
	float b1 = 0.000944691844;
	float b2 = 0.001889383688;
	float b3 = b1;
	float a1 = 1;
	float a2 = -1.911197067426;
	float a3 = 0.914975834801;
	float Out = (b1*t + b2*t_step1 + b3*t_step2 - a2*tf_step1 - a3*tf_step2);
	return Out;
}
// Saturation function
float vel_after_sat(float curr_Deg,float prev_Deg, float sat_Low,float sat_High,float prev_Vel)
{
	if ((prev_Deg <= sat_Low && curr_Deg >= sat_Low) || (prev_Deg >= sat_High && curr_Deg <= sat_High))
	{
		return prev_Vel;
	}
	return D2Vel(curr_Deg,prev_Deg);
}
// Calculations.
void timer0(void)
{
	OCR0A = 155;
	TCNT0 = 0;
	TIMSK0 = TIMSK0 | (1<<OCIE0A);
	TCCR0B |= (1<< CS02) | (1 << CS00);
	TCCR0A =(1<<WGM01); //CTC
	sei();
}

// Timer 0 Interrupt Service Routine
ISR(TIMER0_COMPA_vect)
{
	curr_Vol = (float)VREF/1024 * ReadADC(ADC_PIN);
	curr_Deg = Vol2D(curr_Vol);
	//printfloat(curr_Deg);
	curr_Vel = vel_after_sat(curr_Deg,prev_Deg,sat_High,sat_Low,prev_Vel); //calculate the current velocity
	prev_Deg = curr_Deg;
	prev_Vel = curr_Vel;
	fvel=butter(curr_Vel,vel_1,vel_2,fvel_1,fvel_2);
	vel_2=vel_1;
	vel_1=curr_Vel;
	fvel_2=fvel_1;
	fvel_1=fvel;
	TCNT1 = 0;
}
void timer1_init(void)
{
	// set up timer with prescaler = 256
	TCCR1B |= (1 << WGM12)|(1 << CS12);
	// initialize counter
	TCNT1=0;
	// initialize compare value
	OCR1A = 62499;
	// enable compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	//ENABLE GLOBAL INTERUPTS
	sei();
}

// Using Timer 2 to set up PWM.
void pwm_init(void)
{
	DDRB |= (1 << DDB0); //Set pin 8 as output
	DDRB |= (1 << DDB3); //Set pin 11 (correspond to motor output B on the Ardumoto) as output
	TCCR2A |= (1 << WGM21) | (1 << WGM20); //Set fast PWM Mode
	TCCR2B |= (1 << CS21)|(1 << CS22); //Set pre-scaler clock to 256 and starts PWM
	OCR2A = dutyCycle; //Set duty cycle, the outputs will be switched on/off if counter hits this value
}
// Set Motor ON bits
void switch_ON(void)
{
	PORTB |= (1 << DDB0); //Enable 12V into the board
	PORTB |= (1 << DDB3); //Set pin 11 as HIGH
}

//Clear All
void switch_OFF(void)
{
	PORTB &= ~(1 << DDB0);
	DDRB &= ~(1 << DDB5);
	PORTB &= ~(1 << DDB5);
	PORTB &= ~(1 << DDB3);
	TCCR2A &= ~(1 << COM2A0)|(1 << COM2A1);
}
// Rotate Counter Clockwise
void CCW(void)
{
	switch_ON();
	DDRB |= (1 << DDB5); //Set DIR B as output
	PORTB |= (1 << PORTB5); //Activate DIRB
	TCCR2A |= (1 << COM2A0)|(1 << COM2A1); //Inverting mode (LOW at bottom, HIGH on Match)
}
// Rotate Clockwise
void CW(void)
{
	
	TCCR2A &= ~(1 << COM2A0);
	TCCR2A |= (1 << COM2A1); //Non-inverting mode (HIGH at bottom, LOW on Match)
	DDRB |= (1 << DDB5);
	switch_ON();
}

int main(void)
{
	InitADC();
	DDRC = (1 << PINC0); // Hall Effect sensor
	PORTC = (1 << PINC0);
	timer0();
	uart_init();
	timer1_init();
	prev_Error = 0;
	i_error = 0;
	/***************************************************************************Position Control************************************************************************/
	while (1)
	{
	curr_Error = AREF - curr_Deg;
	//printfloat(curr_Error);
	d_error = (curr_Error - prev_Error)/0.01;
	float u = (Kp*curr_Error) + (Kd*d_error);
	//printfloat(u);
	dutyCycle = 9 + abs((u*255)/90);
	if(dutyCycle > 255){
	dutyCycle = 255;
	}
	//printfloat(dutyCycle);
	pwm_init();
	if(curr_Error > 3){
	CCW();
	}
	else if(curr_Error < -3){
	CW();
	} 
	else switch_OFF();
	printfloat(curr_Error);
	prev_Error = curr_Error;
	}  
	
	
/***************************************************************************Velocity Control************************************************************************/
	/*while(1)
	{
	curr_Error = abs(vel_ref) - abs(fvel);
	i_error = i_error + (curr_Error*0.01);
	d_error = (curr_Error - prev_Error)/0.01;
	u = (Kp*curr_Error) + (Kd*d_error) + (Ki*i_error);
	if (u > 255){
	u = 255;
	}
	control = control + u;
	if (vel_ref >0){
	CCW();
	}
	else{
	CW();
	}
	dutyCycle = control;
	printfloat(fvel);
	pwm_init();
	prev_Error = curr_Error;
	} 
*/
	/**************************************************************************************Virtual Spring********************************************************************/
/*	while(1)
	{
	float springForce = Kspring*abs(curr_Deg);
	if(curr_Deg >= 4){
	CW();
	}
	else if(curr_Deg <= -4){
	CCW();
	}
	else switch_OFF();
	
	dutyCycle = 15 + springForce;
	//printfloat(dutyCycle);
	pwm_init();
	} */

	/***************************************************************************Viscous Force************************************************************************/
/*	while(1)
	{
	switch_OFF();
	float viscForce = Bdamp*abs(fvel);
	//printfloat(viscForce);
	if(fvel > 5){
	CW();
	}
	else if(fvel < -5){
	CCW();
	}
	else switch_OFF();
	dutyCycle = 10 + viscForce;
	if (dutyCycle > 255){
	dutyCycle = 255;
	}
	printfloat(dutyCycle);
	pwm_init();
	} */

	/***************************************************************************Virtual Wall************************************************************************/
/*	while(1)
	{
		if(curr_Deg < 0 && curr_Deg > -90){
			u = (Kwall*abs(curr_Deg)) + (Bwall*(abs(curr_Deg - prev_Deg))/0.01);
			dutyCycle = 15 + u;
			pwm_init();
			CCW();
		} 
		else if (curr_Deg >= -180 && curr_Deg <= -90){
			u = (Kwall*abs(curr_Deg+180)) + (Bwall*(abs(curr_Deg - prev_Deg))/0.01);
			dutyCycle = 15 + u;
			pwm_init();
			CW();
		}
		else
		
		{u = 0;
			dutyCycle = 0;
			pwm_init();
		}
		printfloat(curr_Deg);
	} */
	return 0;
}


