
// #define CLK 16000000
// int controlPS;
// int controlTimerSpeed = CLK/controlPS;
// int controlInterruptFreq = CLK/(controlPS*(controlCMR + 1));
// int controlFreq;
// int controlCMR = (CLK/(controlPS*controlFreq)) - 1;


bool change = false;






void setup()
{
    Serial.println("SETUP");

    noInterrupts();

    //set timer1 interrupt at 1Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    
    // //set timer1 interrupt at 1Hz
    // TCCR1A = 0;// set entire TCCR1A register to 0
    // TCCR1B = 0;// same for TCCR1B
    // TCNT1  = 0;//initialize counter value to 0
    // // set compare match register for 1hz increments
    // OCR1A = 7999;// = 6*10^6/(2000*1)) -1 (must be <65536)
    // // turn on CTC mode
    // TCCR1B |= (1 << WGM12);
    // // Set CS10 and CS12 bits for 1024 prescaler
    // TCCR1B |= (1 << CS10);  
    // // enable timer compare interrupt
    // TIMSK1 |= (1 << OCIE1A);
    interrupts();
    Serial.println("Done setting up");

}

void loop()
{
    if (change)
    {
        Serial.println("i");
        change = false;
    }
}

ISR(TIMER1_COMPA_vect)
{
    change = true;
}