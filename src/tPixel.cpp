/*	tPixel is a flappy bird rendition fin a 1kB program space.
 *  Uses mixed ASM and C programming to trim space where needed by making straight forward bsic functions
 *  
 * Created: 11/23/2016 12:40:08 PM
 *  Author: ReeceM
 */ 

//#define F_CPU 8000000
//Pin names 
#define CLK PB0		//register clock
#define Btt PB2		//Button
#define SRL PB1		//Data out
#define LCH PB3		//Latch pin
#define SPK PB4		//Speaker 

//MACRO
#define On(p)		(PORTB |= (1<<p))
#define Off(p)		(PORTB &= ~(1<<p))
#define Map(n) ((n>>4)&7)	///Saves space to MAP like this, saved 86bytes

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

uint8_t lfsr = 67;
uint8_t temp;

uint8_t pixelMap[8] = {
  0b00000001,
  0b00000001,
  0b00000001,
  0b00000001,
  0b00000001,
  0b00000001,
  0b00000001,
  0b00000001
};  //8 by 8 pixel map
	
uint8_t yPixel = 4;
uint8_t yPixelHole = 4;
uint8_t currentPoint = 0;

//function to shift to 74hc595 ALWAYS MSB first
void shiftOut(uint8_t toShift){
	uint8_t mask = 0;
	
	while(mask < 8){
		
		if(toShift&(1<<(7-mask))) On(SRL);
		else Off(SRL);
		
		On(CLK);
		Off(CLK);
		mask++;
	}
	
	On(LCH);
	Off(LCH);
}

void checkCollision(void){
	
}
/*
	name: scrollScreen
	input:none
	return:none
	Definition:
	This function moves the pixel map across the 8x8 matrix
*/
void scrollScreen(void){
  //shift stuff ~ works
  for (int8_t p = 8; p >= 0; --p) {
    pixelMap[p + 1] = pixelMap[p];
  }
  //add a new columns
  if (pixelMap[1] & 0x03) {
    pixelMap[0] = 0;
  }
  else {
    if(Map(temp) != 7) {
        pixelMap[0] = 0xFF;
		pixelMap[0] &= ~(1 << Map(temp));
    }
  }
 //add pexil
  pixelMap[yPixel] = (pixelMap[yPixel] & 0xFF) | 0x40;
  //Add check collision algorithm here
}

volatile bool inFlag = false;	//flag to signal button press

/****************MAIN*******************
 * 
 * 
 ****************************************/

int main(void)
{
/***
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) ; // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz
   ADMUX |= (1 << ADLAR) | (1 << MUX0);	
   ADCSRA |= (1 << ADEN);  // Enable ADC   
   ADCSRA |= (1 << ADSC);  // Start A2D Conversions
 //Sets outputs
 	DDRB |= ((1 << CLK) | (1 << SRL) | (1 << LCH) | (1 << SPK));
// Interupt stuff
 	DDRB &= ~(1 << Btt);	//INT0 pin
	 MCUCR |= (1 << ISC01); //want a falling edge interrupt to trigger
*/

	//Set the input first of the AD line and then read and then
	//change it to the needed state.

   	asm volatile(
	"ldi r16, 0x1B"";Set the output pins\n\t"
	"out 0x17, r16 ; load them to the register\n\t"
	"cbi 0x17, 2 ;set input on B\tt""\n\t "
	"out 0x06, 0x06 ;ADC prescaler""\n\t"
	"sbi 0x07, 1 ;ADMUX PB2""\n\t"
	"nop ;Give it a short break\n\t"
	"sbi 0x06, 7 ;ADEN""\n\t"
	"sbi 0x06, 6 ;ADSC""\n\t"
	"sei ;set interrupts""\n\t"
	::);	//assembly equivelent of made as one liners

	GIMSK |= (1 << INT0);	//Set the INT0 mask

   while( ADCSRA & (1 << ADSC))
   lfsr = ADC;	//give a new seed to the random number gen.
	shiftOut(0);
   uint8_t inc = 2;
    while(1){
		//keep this here and not in a function as it reduces the size
		lfsr = (lfsr >> 1) ^ (-(lfsr & 1u) & 0xA4); 
		temp = lfsr;
   //  scrollScreen();

     if(inFlag)
     {
		inc++;
		shiftOut((1<<inc));
		inFlag = false;
	    _delay_ms(100);
	   }
	  else{
	  	inc--;
		   shiftOut((1<<inc));
		   _delay_ms(100);
	   }

	   if(inc >= 8) inc = 8;
	   //Keep this at the end of the while(1)
	   temp = (lfsr >> 5);
    }//end of loop
}//main

ISR(INT0_vect){
	inFlag = true;
}

ISR(TIM0_COMPA_vect){
	 scrollScreen();
 }
