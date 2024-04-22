#include "header_initialize.h"
void initializePORTB(void); //inputs: photoresistors,button
void initializePORTD(void); //seven segment
void initializePORTA(void); //motor and relay

void initializePORTB(){
    PORTB=0;
    LATB=0;
    ANSELB=0;
    TRISB=0b11110111; // inputs
}
void initializePORTD(){
    PORTD=0;
    LATD=0;
    ANSELD=0;
    TRISD=0b00000000;
}
void initializePORTA(){
    PORTA=0;
    LATA=0;
    ANSELA=0;
    TRISA=0b00000000;
}
