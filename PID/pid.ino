#include <TimerOne.h>   

#define oePin  32 //active low
#define reset 35

#define enA 8
#define in1 9
#define in2 10

const float sampleTime = 200*10^(-6); // us

const float scaleGains = 2*3.14159/400;
const float scaleAngle = 400/(2*3.14159);

// const float K_U =281*scaleGains;
// const float KP_X = K_U*8.417;
// const float KD_X = K_U;

// const float K_U =10*scaleGains;
// const float KP_X = K_U*8.417;
// const float KD_X = K_U;

// const float K_U =20*scaleGains;
// const float KP_X = K_U*20;
// const float KD_X = K_U;

// const float KP_X = 2365.177*scaleGains;
// const float KD_X = 281*scaleGains;

// const float KP_X = 80*scaleGains;
// const float KD_X = 20*scaleGains/sampleTime;

const float KP_X = 105*scaleGains;
const float KD_X = 50*scaleGains/sampleTime;
// const float KD_X = 40*scaleGains;
//const float KD_X = 0;


//const float N_X = 100;
const float N_X = 100/scaleGains;
const float LASTD_MULT = 1/(1 + N_X*sampleTime);
const float D_MULT = N_X/(1 + N_X*sampleTime);

int pos_X = 0;
int desired_X  = 45*3.1415/180*scaleAngle;
int error_X = 0;
int last_error_X = desired_X;
int last_pos_X = 0;

float p_X = 0;
float d_X = 0;
float last_d_X = 0;
float pid_X = 0;
int bias_X = 84;

bool change = false;

int maxRPM = 2000;
// int read_thresh = maxRPM*400/60*sampleTime;
int read_thresh = 7;

void setup() {

    noInterrupts();

    Serial.begin(115200);

    pinMode(oePin, OUTPUT);
    pinMode(reset, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enA, OUTPUT);

    digitalWrite(oePin, LOW);
    digitalWrite(reset, LOW);
    delay(100);
    digitalWrite(reset, HIGH);
    delay(200);

    // Add homing function

    // Port registers
    DDRC = B01100100;
    DDRL = B00011001;
    DDRG = B00000100;
    DDRD = B00000000;

    DDRB = B00010000;
    DDRH = B01000000;

    // Timer1.initialize(1000);  // 3ms  // initialize timer1, and set a 1/2 second period
    // Timer1.attachInterrupt(control);  // attaches callback() as a timer overflow interrupt

    // //  set timer1 interrupt at 2000Hz (500us)
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

        //  set timer1 interrupt at 2000Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 1199;// = 6*10^6/(2000*1)) -1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    //Serial.println(desired_X*0.9);


    analogWrite(enA, 0);              // Send PWM signal to L298N Enable pin
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 

        
    // qd1
    DDRC = B01100100;
    DDRL = B00011001;
    DDRG = B00000100;
    DDRD = B00000000;
    
    //qd2
    DDRA = B00001000;
    //Serial.println(read_thresh);

    interrupts();

}

void loop () {

    if (change)
    {
        noInterrupts();

        // Get position of _X motor
        // long unsigned start = micros();
        // pos_X = 0;
        // // set SEL1 HIGH and SEL2 LOW
        // // RESET is always high
        // PORTL = B00010001;
        // PORTG = B00000000;

        // // write lower bits to pos_X

        // pos_X = (PINL & B00100000) >> 5 |
        //         (PINC & B00000010) |
        //         (PINC & B00000001) << 2 |
        //         (PIND & B10000000) >> 4 |
        //         (PING & B00000010) << 3 |
        //         (PING & B00000001) << 5 |
        //         (PINL & B10000000) >> 1 |
        //         (PINL & B00000010) << 6; 

        // // long unsigned end = micros();
        // // Serial.print(end - start);

        //     // set SEL1 LOW and SEL2 LOW
        //     // RESET is always high
        // PORTL = B00000001;
        // PORTG = B00000000;

        // // write higher bits to pos_X

        // pos_X = pos_X |
        //         ((PINL & B00100000) >> 5) << 8 |
        //         ((PINC & B00000010)) << 8 |
        //         ((PINC & B00000001) << 2) << 8 |
        //         ((PIND & B10000000) >> 4) << 8 |
        //         ((PING & B00000010) << 3) << 8 |
        //         ((PING & B00000001) << 5) << 8 |
        //         ((PINL & B10000000) >> 1) << 8 |
        //         ((PINL & B00000010) << 6) << 8; 
    pos_X = 0;
    bool read = true;
    // set SEL1 HIGH and SEL2 LOW
    // RESET is always high
    while (read)
    {
        PORTC = B01000100;
        PORTG = B00000000;

        // write lower bits to pos_X

        pos_X = (PINC & B10000000) >> 7 |
            (PINA & B00000001) << 1 |
            (PINA & B00000010) << 1 |
            (PINA & B00000100) << 1 |
            (PINA & B00010000)      |
            (PINA & B00100000)      |
            (PINA & B01000000)      |
            (PINC & B00001000) << 4; 
        
        // long unsigned end = micros();
        // Serial.print(end - start);

            // set SEL1 LOW and SEL2 LOW
            // RESET is always high
        PORTC = B00000100;
        PORTG = B00000000;

        // write higher bits to pos_X

        pos_X = pos_X |
            ((PINC & B10000000) >> 7) << 8 |
            ((PINA & B00000001) << 1) << 8 |
            ((PINA & B00000010) << 1) << 8 |
            ((PINA & B00000100) << 1) << 8 |
            ((PINA & B00010000)     ) << 8 |
            ((PINA & B00100000)     ) << 8 |
            ((PINA & B01000000)     ) << 8 |
            ((PINC & B00001000) << 4) << 8; 

        if (pos_X - last_pos_X < read_thresh || last_pos_X - pos_X < read_thresh)
        {
            read = false;
            // Serial.print("Stop read");
            // Serial.print("\t");
        }
        // Serial.println("Reading...");
    }

        // Serial.print(pos_X);
        // Perform PID calculation
        error_X = desired_X - pos_X;
       



        // Serial.print("\t");
        // Serial.print(pos_X);
        // Serial.print("\t");
        // Serial.print(pid_X);
        // Serial.print("\t");
        // Serial.print(error_X);
        // Serial.print("\t");
        // Serial.println(desired_X);

        // Write values to motor
       
        // d_X = LASTD_MULT*last_d_X + KD_X*D_MULT*(error_X - last_error_X);
        d_X = KD_X*(error_X - last_error_X);

        if ((error_X <= 1) && (error_X >= -1))
        {
            PORTB = B00000000;
            PORTH = B00000000; // in2 low

            // Serial.print("\t");
            //Serial.println("LL");
        }
        else if (pos_X < desired_X)
        {

            PORTB = B00010000;
            PORTH = B00000000; // in2 low
            // digitalWrite(in1, HIGH);
            // digitalWrite(in2, LOW); 
            // Serial.print("\t");
            // Serial.println("HL");
            p_X = KP_X*error_X;
            pid_X = p_X + bias_X + d_X;
        }
        else
        {
            PORTB = B00000000;
            PORTH = B01000000; // in2 HIGH
            // digitalWrite(in1, LOW);
            // digitalWrite(in2, HIGH); 
            // Serial.print("\t");
            // Serial.println("LH");
            p_X = KP_X*error_X;
            pid_X = p_X - bias_X + d_X;
        }

        
        if (pid_X < 0)
        {
            pid_X = -1*pid_X;
        }

        if (pid_X > 255)
        {
            pid_X = 255;
        }

        // p_X = KP_X*error_X;
        // //d_X = LASTD_MULT*last_d_X + KD_X*D_MULT*(error_X - last_error_X);
        // // d_X = KD_X*(error_X - last_error_X);
        
        // // pid_X = p_X + d_X;

        last_d_X = d_X;
        // //last_pos_X=pos_X;
        last_error_X = error_X;
        // // Take absolute value of pidterm and constrain
        // pid_X = p_X + bias_X;
        last_pos_X = pos_X;
        analogWrite(enA, pid_X); // TODO: optimize

        change = false;

        // long unsigned finish = micros();

        //Serial.println(finish - start);

        interrupts();
       
    }

}

void control()
{
    change = true;
}


ISR(TIMER1_COMPA_vect)
{
    change = true;
}
