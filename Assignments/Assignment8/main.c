




#include <xc.h> // must have this
#include "header_config.h"
#include "header_functions.h"


void main(void) {
    while(1){
    // Initialization  
    secret_code=11; // this can be changed
   // WPUB=0xFF;// enable the weak pull-ups are enabled for port B
    
    INTERRUPT_Initialize();// initialize the interrupt_initialization by calling the proper function
    initializePORTB();
    initializePORTD();
    initializePORTA();
    // main code here 
    x=0;
    number1=0;
    PORTD = sevenSegValues[number1];
    while (x<1){
    // read from photo resister1 
    if (PORTBbits.RB2==1){
        number1++;
        PORTD = sevenSegValues[number1];  
        __delay_ms(2000);

    }
    if (PORTBbits.RB7==1){
        x=x+1;//  button is pushed to show end of input1
        __delay_ms(2000);
    }
    
    }
    
    
    //read from photo resister2
    y=0;
    number2=0;
    PORTD = sevenSegValues[number2];
    while (y<1){
    // read from photo resister1 
    if (PORTBbits.RB1==1){
        number2++;
         PORTD = sevenSegValues[number2];
        __delay_ms(2000);
    }
    if (PORTBbits.RB7==1){
        y=y+1;//  button is pushed to show end of input2
        __delay_ms(2000);
    }
    }
 
    guess=(number1*10)+number2;
    
    //if secret code is correct (turn on motor)
    if (guess==secret_code)
    {
        
        PORTBbits.RB3=1;
       
        __delay_ms(2000);
        
        PORTBbits.RB3=0;
       
        
    }
    //if secret code is wrong buzzer will be turned on)
    else if (guess!=secret_code)
    {
        PORTAbits.RA1=1;
        __delay_ms(1000);
        PORTAbits.RA1=0;
    }
}
}


