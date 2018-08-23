/*
 * GccApplication2.c
 *
 * Created: 04/28/2018 15:24:48
 * Author : Leo
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
volatile int Buffer[] = {0, 0};
uint16_t result =0;
uint16_t adcResult = 0;
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
	DDRB = 0x00; //Makes PORTB as Output
	DDRA = 0x88;			// Configure PortA 1000 1000
	
	DIDR0 =0x07; // ADC0/1/2 digital input disabled
	/* Enable Interrupt */
	sei();				// Enable Global Interrupts
	
	/*
	* Setup I2C slave
	*/
	TWSCRA = 1<<TWDIE | 1<<TWASIE | 1<<TWEN;
	TWSA = I2CAddress<<1;
}

uint16_t readTempreture()
{
	
	/*
	* ADC Setup
	*/
	// PA0 is ADC0 input
	ADCSRA = 0x83;			// Enable the ADC and its interrupt feature, 
	// and set the ACD clock pre-scalar to sysclk/16, not interrupt
	ADCSRB = 0x00;			
	ADMUXA = 0x01;			// Select single ended line
	ADMUXB = 0x00; // VCC as Vref,
	// data registers and select ADC0 as input channel
	for(int i=0; i<1; i++){
		ADCSRA |= 1<<ADSC;		// Start Conversion
		while ( ADCSRA & ( 1 << ADSC ) );//wait for conversion to end
	}
	Buffer[0] = ADCH;			// Output ADCH
	Buffer[1] = ADCL;			// Output ADCH
	
	adcResult = ADCL;
	adcResult |= ADCH << 8;
	
	// R = 10K ((1023/ADC)-1)
	
	rResult =10000*((1023/adcResult)-1);
	// 1/T = 1/To + (1/B)(ln(R/Ro))
	// 1/T = 1/273.15 + (1/3428)(ln(rResult/10000))
	//double z = log(rResult/10000);
	//kTemp = 1/(1/273.15 + (1/3428)(z));
	return 1023 - adcResult;
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
	//result = readTempreture();
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
				TWSD = Buffer[Read++];				
				TWSCRB = 3<<TWCMD0;				
			} else TWSCRB = 2<<TWCMD0;         // Complete transaction
		} else {                             // Master write
			if (Write < 2) Buffer[Write++] = TWSD;
			if (Write < 2) TWSCRB = 3<<TWCMD0; // ACK
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
		//PORTB = (1<<PINB0); //Turns ON All LEDs
		PORTB &= ~(1 << PINB0);           // switch PD0 LED off		
		PORTA &= ~(1 << PINA7);           // switch PD0 LED off	
		_delay_ms(500); //1 second delay
		//while ( ADCSRA & ( 1 << ADSC ) );//wait for conversion to end
		result = readTempreture();
		PORTB |=  (1 << PINB0);           // switch PD0 LED on
		PORTA |= (1 << PINA7);           // switch PD0 LED off	
		_delay_ms(500); //1 second delay
	}
}
