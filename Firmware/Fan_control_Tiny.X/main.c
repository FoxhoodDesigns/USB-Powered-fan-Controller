/*
 * File:   main.c
 * Author: Foxhood
 *
 * Created on August 4, 2024, 11:35 PM
 */

//Reference cheat-sheet Read the IOTN1614.h file!

#define F_CPU   2500000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/* Pinout Cheat sheet:
 * B0: Green led
 * B1: Red led
 * B2: PWM
 * B3: TAC
 * A0: UPDI
 * A1: INT (Encoder)
 * A2: ENC (U/L)
 * A3: SW1
 * A4: FAN_E
 * A5: CP1
 * A6: CP2
 * A7: CP3 
 */

//Pin bitmasks
#define G_LED 0x01
#define R_LED 0x02
#define TAC 0x08
#define ENC 0x04
#define ENC_SW 0x08
#define FAN_E 0x10
#define PWM 0x04
//Macros
#define ERROR 0
#define DC_MODE 1
#define PWM_MODE 2
#define OFF_MODE 3


const uint8_t C_LED_OUT[6] = {0x04,0x02,0x02,0x01,0x04,0x01};
const uint8_t C_LED_DIR[6] = {0x06,0x06,0x03,0x03,0x05,0x05};
const uint8_t TMR_MAX_LIMIT = 100;
const uint8_t TMR_MIN_LIMIT = 40; //Set minimum speed to ~40%, 192 steps
const uint16_t TIMEOUT_limit = 1000;
const uint8_t Encoder_step = 5; //12 encoder steps to go from minimum to top speed
const uint8_t Led_step = 10;
volatile uint8_t set_speed = 50;
volatile uint16_t fan_rpm = 0;
volatile uint8_t time_out = 0;
volatile uint8_t state = 0;
volatile uint16_t pulse_width_counter = 0;
uint8_t fan_detect = PWM_MODE;
volatile uint8_t fan_mode = PWM_MODE;
uint8_t fan_stat;

uint8_t locked = 0;




//AVR Interrupt Service Routine
int led_out(uint8_t stat){
    if (stat == 0xff){
        PORTA.DIR = (PORTA.DIR & 0x1f);
        PORTA.OUT = (PORTA.OUT & 0x1f);
        return 1;
    }
    PORTA.OUT = (PORTA.OUT & 0x1f);
    PORTA.DIR = (PORTA.DIR & 0x1f) + (C_LED_DIR[stat]<<5);
    PORTA.OUT = (PORTA.OUT & 0x1f) + (C_LED_OUT[stat]<<5);
    return 0;
}
void led_bar_tick(uint8_t leds){
    static uint8_t ticker = 0;
    if (ticker < leds){
        led_out(ticker);
    } else {
        PORTA.DIR = (PORTA.DIR & 0x1f);
        //led_out(0xff);
    }
    ticker++;
    if (ticker == 6) ticker = 0;
}
ISR(PORTA_PORT_vect){
    if (PORTA.IN & ENC) {
        set_speed -= Encoder_step;
        if (set_speed < TMR_MIN_LIMIT) {
            set_speed = TMR_MIN_LIMIT;
        }
    } else {
        set_speed += Encoder_step;
        if (set_speed > TMR_MAX_LIMIT) {
            set_speed = TMR_MAX_LIMIT+1;
        }
    }
    if (fan_mode == PWM_MODE) {
        TCA0.SINGLE.CMP2 = set_speed;
    } else if (fan_mode == DC_MODE){
        TCA0.SPLIT.HCMP1 = set_speed << 1;
    }
    //led_out((set_speed-TMR_MIN_LIMIT)/Led_step);
    PORTA.INTFLAGS = 0xff;
}
ISR(TCB0_INT_vect){
    uint8_t input = (PORTB.IN & PIN3_bm);
    if (input == state){
        pulse_width_counter++;
    } else {
        fan_rpm = 1000 / pulse_width_counter * 15;
        pulse_width_counter = 0;
        state = input;
    }
    if (pulse_width_counter > TIMEOUT_limit && fan_mode != OFF_MODE) {
        time_out = 1;
    }
    TCB0.INTFLAGS = 1;
}

//Initiates timerA for output PWM generation.
//Target frequency is 25Khz. By dividing main 20Mhz clock by 8 (2.5Mhz) and using a period of 100. No division is needed.
void TCA_PWM_init(void) {
    TCA0.SINGLE.CTRLB = 0 << TCA_SINGLE_ALUPD_bp
	                    | 0 << TCA_SINGLE_CMP0EN_bp
	                    | 0 << TCA_SINGLE_CMP1EN_bp
	                    | 1 << TCA_SINGLE_CMP2EN_bp
	                    | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = TMR_MAX_LIMIT;
    TCA0.SINGLE.CMP2 = TMR_MAX_LIMIT/2;
    TCA0.SINGLE.DBGCTRL = 1;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc //set for 25khz
                        | 1 << TCA_SINGLE_ENABLE_bp;
}
//Reconfigures timerA for output DC generation.
void TCA_DC_init(void) {
    TCA0.SINGLE.CTRLA = 0;  //Disable from PWM mode
    TCA0.SINGLE.CMP2 = 0;
    TCA0.SPLIT.CTRLD = 1 << TCA_SPLIT_SPLITM_bp;
    TCA0.SPLIT.CTRLB =  1 << TCA_SPLIT_HCMP1EN_bp;
    TCA0.SPLIT.HPER = TMR_MAX_LIMIT*2;
    TCA0.SPLIT.HCMP1 = TMR_MAX_LIMIT;
    TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV1024_gc //set for lowest possible speed. (12.5hz)
                        | 1 << TCA_SPLIT_ENABLE_bp;
    PORTB.DIRSET = PWM;
    PORTB.OUTSET = PWM;    
}
//Initiates TimerB for tachometer monitoring. Sampling rate of 1Khz
void TCB_INT_int(void) {
    TCB0.INTCTRL = 1;
    TCB0.CCMP = 2500;
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //Enable with clkdiv1
}

//Initiates EVSYS and TimerB for Frequency measurement (FAILED TO GET WORKING...)
//https://github.com/microchip-pic-avr-examples/avr128da48-adc-evsys-example
/*
void evsys_timer_b_init(void) {
    //Activate EVSYS Event puss
    EVSYS.SYNCCH1 = EVSYS_SYNCCH1_PORTB_PIN2_gc; //Set Synchronous channel 1 to PORTB PIN3
    EVSYS.SYNCUSER0 = EVSYS_ASYNCUSER0_SYNCCH1_gc; //Set TCB User channel to Synchronous channel 1
    //EVSYS.SYNCCH1 = 0xb; //Set Synchronous channel 1 to PORTB PIN3
    //EVSYS.SYNCUSER0 = 0x2; //Set TCB User channel to Synchronous channel 1
    PORTB.DIRCLR = PIN3_bm;
    PORTB.PIN3CTRL = 0x00;
    TCB0.CTRLB = TCB_CNTMODE_CAPT_gc; //Set to Capture?
    //TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm;
    TCB0.DBGCTRL = 1;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA = 0x02 | TCB_ENABLE_bm; //Enable with clkdiv2
}*/

int main(void) {
    //set clock to 2.5Mhz
    CCP = 0xD8;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PEN_bm | CLKCTRL_PDIV_8X_gc;    
    //Set IO
    PORTB.DIR = 0x07;    //Set B0-B2 as Output
    PORTA.DIR = 0xf0;    //set A4-A7 as Output
    //Start initial configuration
    PORTA.OUTSET = FAN_E;
    PORTA.PIN1CTRL = PORT_ISC_RISING_gc; //Set INT pin as Interrupt Source (Rising)
    TCA_PWM_init();
    TCB_INT_int();
    sei();
    //Fan Test. Run at half PWM speed for 4 seconds and record rpm
    PORTB.OUTSET = G_LED;
    TCA0.SINGLE.CMP2 = 50;
    _delay_ms(4000);
    uint16_t tach1 = fan_rpm;
    //Run at 80% speed for 4 seconds and test if higher than original * 1.33.
    PORTB.OUTCLR = G_LED;
    PORTB.OUTSET = R_LED;
    TCA0.SINGLE.CMP2 = 80;
    time_out = 0;
    _delay_ms(6000);
    //Configure based on result
    if (time_out == 1) {
        fan_detect = ERROR;
    } else if ((tach1 + (tach1/3)) < fan_rpm) {
        fan_detect = PWM_MODE;
        TCA0.SINGLE.CMP2 = set_speed;
    } else {
        fan_detect = DC_MODE;
    }
    
    //Prep main program
    if (fan_detect == PWM_MODE){
        fan_mode = PWM_MODE;
        PORTB.OUTSET = G_LED;
        PORTB.OUTCLR = R_LED;
    } else if (fan_detect == DC_MODE) {
        fan_mode = DC_MODE;
        TCA_DC_init();
        PORTB.OUTSET = R_LED | G_LED | 0x04; //Turn on both leds and set PWM pin to always on.
    } else {
        fan_mode = ERROR;
    }
    _delay_ms(500);
    time_out = 0; //reset for run-time
    while(1) {
        switch(fan_mode){
            case DC_MODE:
                if (!(PORTA.IN & ENC_SW)) { //Switch to off
                    fan_mode = OFF_MODE;
                    TCA0.SPLIT.HCMP1 = 0;
                    PORTA.OUTCLR = FAN_E;
                    PORTB.OUTCLR = G_LED | R_LED;
                    led_out(0xff);
                    _delay_ms(1000);
                }
                if (time_out) {
                    fan_mode = ERROR;
                }
                led_bar_tick((set_speed-TMR_MIN_LIMIT)/Led_step);
                break;
            case PWM_MODE:
                if (!(PORTA.IN & ENC_SW)) { //Switch to off
                    fan_mode = OFF_MODE;
                    TCA0.SINGLE.CMP2 = 0;
                    PORTA.OUTCLR = FAN_E;
                    PORTB.OUTCLR = G_LED | R_LED;
                    led_out(0xff);
                    _delay_ms(1000);
                }
                if (time_out) {
                    fan_mode = ERROR;
                }
                led_bar_tick((set_speed-TMR_MIN_LIMIT)/Led_step);
                break;
            case OFF_MODE:
                led_out(0xff);
                if (!(PORTA.IN & ENC_SW)) { //Switch to ON
                    if (fan_detect == PWM_MODE){
                        fan_mode = PWM_MODE;
                        PORTB.OUTSET = G_LED;
                        TCA0.SINGLE.CMP2 = set_speed;
                    } else {
                        fan_mode = DC_MODE;
                        PORTB.OUTSET = G_LED | R_LED;
                        TCA0.SPLIT.HCMP1 = set_speed * 2;
                    }
                    PORTA.OUTSET = FAN_E;
                    
                    
                    _delay_ms(1000);
                    time_out = 0;
                }
                break;
            default: //Error
                if (!locked) {
                    cli();  //Shut off interrupt
                    TCA0.SINGLE.CTRLA = 0;  //Shut off Timer
                    TCB0.CTRLA = 0;
                    PORTA.OUTCLR = FAN_E;
                    PORTA.DIR = 0;  //Disable all outputs except for the status leds
                    PORTB.DIR = 0x03;
                    locked = 1;
                }
                while(1) {
                    PORTB.OUTSET = G_LED | R_LED;
                    _delay_ms(1000);
                    PORTB.OUTCLR = G_LED | R_LED;
                    _delay_ms(1000);
                }
                break;
        }
        _delay_us(100);       
    }
}

