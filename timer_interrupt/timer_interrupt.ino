
// #define CLK 16000000
// int controlPS;
// int controlTimerSpeed = CLK/controlPS;
// int controlInterruptFreq = CLK/(controlPS*(controlCMR + 1));
// int controlFreq;
// int controlCMR = (CLK/(controlPS*controlFreq)) - 1;


bool timer1 = false;
bool timer3 = false;





void setup()
{
    Serial.begin(115200);
    Serial.println("SETUP");

    noInterrupts();

    // //set timer1 interrupt at 1Hz
    // TCCR1A = 0;// set entire TCCR1A register to 0
    // TCCR1B = 0;// same for TCCR1B
    // TCNT1  = 0;//initialize counter value to 0
    // // set compare match register for 1hz increments
    // OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // // turn on CTC mode
    // TCCR1B |= (1 << WGM12);
    // // Set CS10 and CS12 bits for 1024 prescaler
    // TCCR1B |= (1 << CS12) | (1 << CS10);  
    // // enable timer compare interrupt
    // TIMSK1 |= (1 << OCIE1A);

    
    //set timer1 interrupt at 2000Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 7999;// = 16*10^6/(2000*1)) -1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    //set timer1 interrupt at 5Hz(0.2s)
    TCCR3A = 0;// set entire TCCR3A register to 0
    TCCR3B = 0;// same for TCCR3B
    TCNT3  = 0;//initialize counter value to 0
    // set compare match register for 3hz increments
    OCR3A = 49999;// = 16*10^6/(5*64) - 1 (must be < 65536)
    // turn on CTC mode
    TCCR3B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR3B |= (1 << CS11) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK3 |= (1 << OCIE3A);

    interrupts();
    // Serial.println("Done setting up");

}

void loop()
{
    // if (timer1)
    // {
            // noInterrupts();
    //     Serial.println("t1");
    //     timer1 = false;
        // interrupts();
    // }

    if (timer3)
    {
        noInterrupts();
        Serial.println("t2");
        timer3 = false;
        interrupts();
    }
}

ISR(TIMER1_COMPA_vect)
{
    timer1 = true;
}

ISR(TIMER3_COMPA_vect)
{
    timer3 = true;
}

