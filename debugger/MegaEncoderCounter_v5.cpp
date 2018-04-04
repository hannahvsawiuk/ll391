/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Author: Andrew Jalics */
/* Modified by: Eric Chen*/

/* Please see header file (MegaEncoderCounter_v4.h) and Avago HCTL-2022 Datasheet for more technical details */

#include "MegaEncoderCounter_v5.h"

void PitchReset() {
	// Set modes for data pins, SEL1, SEL2, OE, RST
	DDRC = B00000000;
	pinMode(PITCH_CHIP_SEL1, OUTPUT);
	pinMode(PITCH_CHIP_SEL2, OUTPUT);
	pinMode(PITCH_CHIP_OE, OUTPUT);
	pinMode(PITCH_CHIP_RESET, OUTPUT);

	digitalWrite(PITCH_CHIP_OE, LOW);
	digitalWrite(PITCH_CHIP_RESET, LOW);
	delay(50);
	digitalWrite(PITCH_CHIP_RESET, HIGH);
}



void YawReset() {
	// set modes for data pins, SEL1, SEL2, OE, RST
	DDRA = B00000000;
	pinMode(YAW_CHIP_SEL1, OUTPUT);
	pinMode(YAW_CHIP_SEL2, OUTPUT);
	pinMode(YAW_CHIP_OE, OUTPUT);
	pinMode(YAW_CHIP_RESET, OUTPUT);

	digitalWrite(YAW_CHIP_OE, LOW);
	digitalWrite(YAW_CHIP_RESET, LOW);
	delay(50);
	digitalWrite(YAW_CHIP_RESET, HIGH);

}



int PitchGetCount() {
	int decoderCount = 0; // reset decoderCount

	PORTL &= B01111111; // Sel 1 Low
	PORTL &= B10111111; // Sel 2 Low
	decoderCount += PINC;
	decoderCount <<= 8;

	PORTL |= B10000000; // Sel 1 High
	PORTL &= B10111111; // Sel 2 Low
	decoderCount += PINC;

	return decoderCount;
}


int YawGetCount() {
	int decoderCount = 0; // reset decoderCount

	PORTD &= B01111111; // Sel 1 Low
	PORTG &= B11111011; // Sel 2 Low
	decoderCount += PINA;
	decoderCount <<= 8;

	PORTD |= B10000000; // Sel 1 High
	PORTG &= B11111011; // Sel 2 Low
	decoderCount += PINA;
	
	return decoderCount;
}
