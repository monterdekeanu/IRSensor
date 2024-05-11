/*
 * File:   main.c
 * Author: user
 *
 * Created on May 11, 2024, 2:43 PM
 */


// PIC16F877A Configuration Bit Settings
// 'C' source line config statements
// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>



#define _XTAL_FREQ 20000000  // 20 MHz Crystal frequency
#define B0 PORTBbits.RB0 //0[1]00 B2=1 PORTB = 3 || 0011
#define B1 PORTBbits.RB1
#define B2 PORTBbits.RB2
#define B3 PORTBbits.RB3
#define A0 PORTAbits.RA0
#define A1 PORTAbits.RA1
#define A2 PORTAbits.RA2
#define IR_RIGHT PORTDbits.RD0
#define IR_CENTER PORTDbits.RD1
#define IR_LEFT PORTDbits.RD2
#define MOTOR_LEFT PORTBbits.RB0
#define MOTOR_RIGHT PORTBbits.RB1
#define WHITE_DETECTED 1
#define BLACK_DETECTED 0
void motorControl(int motor1, int motor2) {
    RB0 = motor1 & 0x01;
    RB1 = (motor1 >> 1) & 0x01;
    RB2 = motor2 & 0x01;
    RB3 = (motor2 >> 1) & 0x01;
}
void followLine() {
    unsigned char sensorState = PORTA & 0x07; // Read sensors (assuming active high)

    switch(sensorState) {
        case 0x01: // Right sensor active, turn right
            motorControl(0x02, 0x01);
            break;
        case 0x04: // Left sensor active, turn left
            motorControl(0x01, 0x02);
            break;
        case 0x02: // Middle sensor active, go straight
            motorControl(0x02, 0x02);
            break;
        default: // No line, stop or search for line
            motorControl(0x00, 0x00);
            break;
    }
}

void followLineSimplified(){
    PORTB = 0;
    //RD0 IS RIGHT SENSOR
    //RD1 IS CENTER
    //RD2 IS LEFT
    //ON LIGHT IS WHITE AND OFF LIGHT IS BLACK
    //1 is ON ||white detected and 0 is OFF || black detected
    if((IR_RIGHT == WHITE_DETECTED && IR_LEFT == WHITE_DETECTED) &&  IR_CENTER==BLACK_DETECTED ){ //move forward
        MOTOR_LEFT = 1;
        MOTOR_RIGHT = 1;
    }else if((IR_RIGHT==WHITE_DETECTED) && (IR_LEFT == BLACK_DETECTED  && IR_CENTER == BLACK_DETECTED)){//navigate to left
        MOTOR_LEFT = 0;
        MOTOR_RIGHT = 1;
    }else if((IR_LEFT==WHITE_DETECTED) && (IR_RIGHT == BLACK_DETECTED && IR_CENTER == BLACK_DETECTED)){//navigate to right
        MOTOR_LEFT = 1;
        MOTOR_RIGHT = 0;
    }else{
        MOTOR_LEFT = 0;
        MOTOR_RIGHT = 0;
    }
    
    //BY DEFAULT MO ON SIYA IF WAY BLACK
//    if(D0 == 1){
//        B0 = 1;
//    }
//    if(D1 == 1){
//        B1 = 1;
//    }
//    if(D2 == 1){
//        B2 = 1;
//    }

}

void followLineTwoSensor(){
    if(IR_RIGHT == BLACK_DETECTED && IR_LEFT == BLACK_DETECTED){//stop
        MOTOR_LEFT = 0;
        MOTOR_RIGHT = 0;
    }else if(IR_LEFT == BLACK_DETECTED && IR_RIGHT == WHITE_DETECTED){///turn right
        MOTOR_LEFT = 1;
        MOTOR_RIGHT = 0;
    }else if(IR_LEFT == WHITE_DETECTED && IR_RIGHT == BLACK_DETECTED){//turn left
        MOTOR_LEFT = 0;
        MOTOR_RIGHT = 1;
    }else if(IR_LEFT == WHITE_DETECTED && IR_RIGHT == WHITE_DETECTED){// forward
        MOTOR_LEFT = 1;
        MOTOR_RIGHT=1;
    }
}
void setup(){
    TRISB = 0x00; // Set PORTB as output for motor control
    TRISA = 0x07; // Set PORTA (RA0, RA1, RA2) as input for sensors
    PORTB = 0;
}

void setupSimplified(){
    TRISB = 0; // Set PORTB as output
    TRISD = 0xFF; // Set PORD as input
    PORTB = 0; // Init PORTB as 0
}
void main(void) {
    setupSimplified();
    while(1) {
        followLineTwoSensor();
        __delay_ms(50);
    }
    return;
}
