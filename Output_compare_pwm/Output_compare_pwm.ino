//***************************//
//         Constants         //
//***************************//

#define sysCLK  16000000 // 16MHz external clock

#define pwmOUT1 8       // port H5 (OC4C, )// with timer 4
#define pwmOUT2 13      // port B7 (OC1C, OC0A)//will be using timer 0

#define duty_cyle1 128
#define duty_cyle2 128

//***************************//
//  Header and include files //
//***************************//
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

void setup(void) { 
    noInterrupts(); // disables interrupts
    // configure ports
    pinMode(pwmOUT1, OUTPUT); // enables port H5 (0C4C) as an output pin
    pinMode(pwmOUT2, OUTPUT); // enables port B7 (OC0A if timer 0) as an output pint

    /* Sets Timer0 in Fast PWM mode. Clears OC0A on Compare Match, set OC0A at
   BOTTOM (non-inverting mode). Then, waveform generation is set to mode 3: Fast PWM with TOP of 0xFF*/
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); 
    TCCR0B = (1 << CS01);  /* Pre-scaling of 8 */

    /* Sets Timer4 in Fast PWM mode. Clears OC4C on Compare Match, sets OC4C at BOTTOM (non-inverting mode).
    Then, waveform generation is set 8-bit Fast PWM with TOP of 0xFF*/
    TCCR4A = (1 << COM4C1) | (1 << WGM40);
    TCCR4B = (1 << CS41) | (1 << WGM42); /* Pre-scaling of 8 */

    TIMSK0 = (1 << TOIE0);  /*Enable Timer0*/
    TIMSK4 = (1 << TOIE4);  /*Enable Timer4*/

    OCR4C = duty_cyle1;
    OCR0A = duty_cyle2;

    interrupts(); /*enable interrupts*/
}

/******************************/
/*      Main System Run       */
/******************************/
void loop (void) 
{
}