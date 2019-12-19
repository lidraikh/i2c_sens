/*
* i2c_se ns.c
*
* Created: 04/28/2018 15:24:48
* Author : Leo
* Version: 1.0
*/

#ifndef F_CPU
#define F_CPU 2000000UL // 2 MHz clock speed
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <math.h>

const int I2CAddress = 0x01;
int I2CReg=0x04;
volatile int Buffer[] = {0,0,0,0,0,0,0,0,0xCC,0x10};  //(M1H,M1L,M2H,M2L,T1H,T1L,C1H,C1L) last two bytes for version and function
int ADCreadings[] = {0,0,0,0,0,0,0,0}; //(M1H,M1L,M2H,M2L,T1H,T1L,C1H,C1L)
uint16_t result =0;
uint16_t kTemp = 0;
uint16_t rResult = 0;
volatile int Write = 0;
volatile int Read = 0;




/*ADC Conversion Complete Interrupt Service Routine (ISR)*/
ISR(ADC_vect)
{
	//Buffer[0] = ADCH;			// Output ADCH to PortD
	//Buffer[1] = ADCL;			// Output ADCH to PortD
}
// Setup **********************************************

void setup() {
	/*
	* Setup All Needed I/O
	*/
	//DDRB = 0x00; //Makes PORTB as Output
	DDRB = 0x0f; //Makes PORTB as Output
	DIDR0 =0x07; // ADC0/1/2 digital input disabled
	/* Enable Interrupt */
	sei();				// Enable Global Interrupts
	
	/*
	* Setup I2C slave
	*/
	TWSCRA = 1<<TWDIE | 1<<TWASIE | 1<<TWEN;
	TWSA = I2CAddress<<1;
}

/*
* Input of this Function is one of these options
* 0 ; for temperature sensor - channel zero (A0)
* 1 ; for first moisture sensor (A1)
* 2 ; for second moisture sensor (A2)
* 7 ; for conductivity sensor. (A7)
*/
uint16_t analogRead(uint16_t AN){
	/*
	* ADC Setup
	*/
	uint16_t adcResult = 0;
	// PA0 is ADC0 input
	ADCSRA = 0x83;			// Enable the ADC and its interrupt feature,
	// and set the ACD clock pre-scalar to sysclk/16, not interrupt
	ADCSRB = 0x00;
	ADMUXA = AN;			// Select single ended line
	ADMUXB = 0x60; // 4.096V as Vref,
	// data registers and select ADC0 as input channel
	for(int i=0; i<1; i++){
		ADCSRA |= 1<<ADSC;		// Start Conversion
		while ( ADCSRA & ( 1 << ADSC ) );//wait for conversion to end
	}
	switch (AN)
	{
		case 0:
		Buffer[4] = ADCH;			// Output ADCH
		Buffer[5] = ADCL;			// Output ADCH
		break;
		case 1:
		Buffer[0] = ADCH;			// Output ADCH
		Buffer[1] = ADCL;			// Output ADCH
		break;
		case 2:
		Buffer[2] = ADCH;			// Output ADCH
		Buffer[3] = ADCL;			// Output ADCH
		break;
		case 7:
		Buffer[6] = ADCH;			// Output ADCH
		Buffer[7] = ADCL;			// Output ADCH
		break;
		
	}
	
	adcResult = ADCL;
	adcResult |= ADCH << 8;

	return adcResult;
	
}

uint16_t readT1()
{
	uint16_t T1 = 0;
	T1 = analogRead(0);
	return T1;
}

uint16_t readC1()
{
	uint16_t C1 = 0;
	C1 = analogRead(7);
	return C1;
}

uint16_t readM1()
{
	uint16_t M1 = 0;
	M1 = analogRead(1);
	return M1;
}

uint16_t readM2()
{
	uint16_t M2 = 0;
	M2 = analogRead(2);
	return M2;
}

// I2C Interface **********************************************
/*
The structure of this interrupt service routine is as follows:
-If the interrupt is caused by an address match:
-- If it is a Master Read then zero the read pointer, and send an ACK.
-- If it is a Master Write then zero the write pointer, and send an ACK.
-Otherwise the interrupt is a data interrupt:
-- If the address indicated a Master Read and there are more bytes to send, put the next byte in the data register, otherwise end the transaction.
-- If the address indicated a Master Write, read the next byte from the data buffer and display it, and send an ACK if we want to receive more bytes, otherwise a NACK.
*/
// TWI interrupt
ISR(TWI_SLAVE_vect) {

	if (TWSSRA & 1<<TWASIF) {              // Address received
		if (TWSSRA & 1<<TWDIR) {             // Master read
			Read = 0;
			TWSCRB = 3<<TWCMD0;                // ACK
			} else {                             // Master write
			Write = 0;                         // Reset pointer
			TWSCRB = 3<<TWCMD0;                // ACK
		}
		} else if (TWSSRA & 1<<TWDIF) {        // Data interrupt
		if (TWSSRA & 1<<TWDIR) {             // Master read
			if (Read < 2) {
				switch (I2CReg)
				{
					case 1:
					TWSD = Buffer[Read++];
					TWSCRB = 3<<TWCMD0;
					break;
					case 2:
					TWSD = Buffer[2+Read++];
					TWSCRB = 3<<TWCMD0;
					break;
					case 3:
					TWSD = Buffer[4+Read++];
					TWSCRB = 3<<TWCMD0;
					break;
					case 4:
					TWSD = Buffer[6+Read++];
					TWSCRB = 3<<TWCMD0;
					break;
					case 5:
					TWSD = Buffer[8+Read++];
					TWSCRB = 3<<TWCMD0;
					break;
					default:
					TWSD = Buffer[8+Read++];
					TWSCRB = 3<<TWCMD0;
				}
					
			} else TWSCRB = 2<<TWCMD0;         // Complete transaction
			} else {                             // Master write
			//if (Write < 1) Buffer[Write++] = TWSD;
			if (Write < 1)
			{
				I2CReg = TWSD;
				++Write;
			}
			if (Write < 1) TWSCRB = 3<<TWCMD0; // ACK
			else TWSCRB = 1<<TWAA | 2<<TWCMD0; // NACK and complete
		}
	}
}

int main(void)
{
	CLKCR  |= 0x02;
	CCP = 0xD8;
	CLKPR = 0x02;
	setup();
	while(1) //infinite loop
	{
		PORTB |= ( 1 << PINB0); //Turns ON All LEDs
		//PORTB &= ~(1 << PINB0);           // switch PD0 LED off
		_delay_ms(500); //1 second delay
	// Make readings
		readC1();
		readM1();
		readM2();
		readT1();
		PORTB |=  (1 << PINB0);           // switch PD0 LED on
		PORTB &= ~(1 << PINB0);           // switch PD0 LED off
		_delay_ms(500); //1 second delay
	}
}
