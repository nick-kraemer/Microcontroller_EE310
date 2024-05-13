 //* File:   main.c
 //* Author: nkrae
 //*
 //* Created on April 28, 2024, 
 //*/
// Title: LCD
//-----------------------------
// Purpose: This code is used to convert an analog signal received on PORTA0
// to a digital  signal (voltage) being transmitted onto PORTB. In this code I also converted
//my voltage reading into LUX to measure the amount of light in a room using a photo resistor.
// Compiler: XC8
// Author: Nick Kraemer

  // OUTPUTS: 
  //PORTB is used to display the solution on LCD

  //input
//PORTA0 is used for the analog input signal
  
// Versions:
//  	V1.0: 4/28/24
//-----------------------------

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = LP     // External Oscillator Selection (LP (crystal oscillator) optimized for 32.768 kHz; PFM set to low power)
#pragma config RSTOSC = EXTOSC  // Reset Oscillator Selection (EXTOSC operating per FEXTOSC bits (device manufacturing default))

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write-protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)
#include <xc.h> // must have this
//#include "../../../../../Program Files/Microchip/xc8/v2.40/pic/include/proc/pic18f46k42.h"
//#include "C:\Program Files\Microchip\xc8\v2.40\pic\include\proc\pic18f46k42"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "PWM_final.h" // must have this
#include "configwords_final.h" // must have this -  XC8_ConfigFile.h
//

#define _XTAL_FREQ 4000000                 // Fosc  frequency for _delay()  library
#define FCY    _XTAL_FREQ/4
//
#define RS LATC6                  /* PORTD 0 pin is used for Register Select */
#define EN LATC5                /* PORTD 1 pin is used for Enable */
#define ldata LATB                 /* PORTB is used for transmitting data to LCD */

#define LCD_Port TRISB              
#define LCD_Control TRISC
#define Vref 5.0 // voltage reference 

#define PWM2_INITIALIZE_DUTY_VALUE 409

uint16_t checkdutyCycle;
char preScale;
_Bool pwmStatus;

int digital; // holds the digital value 
float voltage; // hold the analog value (volt))
float number;
float points;
float button;
int a;
int b;
int c;
int pwm_count;
float input1;


char data[10];
void ADC_Init(void);
void LCD_Init();
void LCD_Command(char );
void LCD_Char(char x);
void LCD_String(const char *);
void LCD_String_xy(char ,char ,const char*);
void check_keypad();
void initializePORTD(); //keypad


/*****************************Main Program*******************************/

void main(void)
{     
    OSCSTATbits.HFOR =1; // enable  HFINTOSC Oscillator (see clock schematic))
    OSCFRQ=0x02; // 00=1 MHZ, 02=4MHZ internal - see page 106 of data sheet
    TMR2_Initialize();
    TMR2_StartTimer();        
    
//    PWM_Output_D8_Enable();
    PWM2_Initialize();
    PWM2_LoadDutyValue(PWM2_INITIALIZE_DUTY_VALUE ); // initialize CCPR2H/L
   // PWM_Output_D8_Disable();
   // TMR2_StopTimer();  

    // Duty Cycle in percentage 
    checkdutyCycle =(uint16_t)((100UL*PWM2_INITIALIZE_DUTY_VALUE)/(4*(T2PR+1)));
    // binary value of Register T2CON.PRESCALE
    preScale = ((T2CON >> 4) & (0x0F)); 
    //PWM
    
    srand(time(NULL));
    //OSCCON=0x72;                   /* Use Internal Oscillator with Frequency 8MHZ */ 
    LCD_Init();                    /* Initialize 16x2 LCD */
     //ADC Initialization
    ADC_Init();
    initializePORTD();
    number=12345;
    points=0;
    while (1) {
        ADCON0bits.GO = 1; //Start conversion
        while (ADCON0bits.GO); //Wait for conversion done
        digital = (ADRESH*256) | (ADRESL);/*Combine 8-bit LSB and 2-bit MSB*/
        voltage= digital*((float)Vref/(float)(4096)); 
        
        
        
        //print on LCD 
        sprintf(data,"%.2f",number);
        strcat(data, " CODE             "); /*Concatenate result and unit to print*/
        LCD_String_xy(1,0,data);    /* Display string at location(row,location). */
        
       
       
       
       
    /*It is used to convert integer value to ASCII string*/
        sprintf(data,"%.2f",points);
        strcat(data, " points             "); /*Concatenate result and unit to print*/
    LCD_String_xy(2,4,data);/*Send string data for printing*/
    
    if (voltage > 3)
    {
    __delay_ms(5000);
    }
    else if (voltage<1)
    {
    __delay_ms(300);
    }
    
    else if (voltage<2)
    {
    __delay_ms(1000);
    }
    else if (voltage<3)
    {
    __delay_ms(2000);
    }
    
    LCD_String_xy(1,0,"                 "); // hides the code
//        check_keypad();
//        input1=input1+(button*100000);
//       __delay_ms(500);
    check_keypad();
        input1=input1+(button*10000);
       __delay_ms(500);
      check_keypad();
        input1=input1+(button*1000);
       __delay_ms(500);
       check_keypad();
       input1=input1+(button*100);
       __delay_ms(500);
       check_keypad();
       input1=input1+(button*10);
       __delay_ms(500);
       check_keypad();
       input1=input1+button;
      __delay_ms(500);
       if (input1==number)
       {
        points=points+1;
        pwm_count=0;
        
        while (pwm_count<10000)
        {
        pwmStatus = PWM2_OutputStatusGet();
        PORTCbits.RC2 = pwmStatus;
        //T2CON=0x00; // stop the timer & do what you have to do
        if (PIR4bits.TMR2IF == 1) {
            PIR4bits.TMR2IF = 0;
       }
        pwm_count=pwm_count+1;
        }
        PORTCbits.RC2 =0;
       }
       else if (input1!=number)
       {
           PORTCbits.RC3=1;
           __delay_ms(1000);
           PORTCbits.RC3=0;
           
       }
       input1=0;
       __delay_ms(2000);
      
     
     number=number+17257;
     if (number>90000)
     {
         number=number-89475;
     }
       
    }
    
              
}

/****************************Functions********************************/
void LCD_Init()
{
    __delay_ms(15);          /* 15ms,16x2 LCD Power on delay */
    LCD_Port = 0x00;       /* Set PORTB as output PORT for LCD data(D0-D7) pins */
    LCD_Control = 0x00;    /* Set PORTC as output PORT LCD Control(RS,EN) Pins */
    LCD_Command(0x01);     /* clear display screen */
    LCD_Command(0x38);     /* uses 2 line and initialize 5*7 matrix of LCD */
    LCD_Command(0x0c);     /* display on cursor off */
    LCD_Command(0x06);     /* increment cursor (shift cursor to right) */
}

void LCD_Clear()
{
        LCD_Command(0x01); /* clear display screen */
}

void LCD_Command(char cmd )
{
    ldata= cmd;            /* Send data to PORT as a command for LCD */   
    RS = 0;                /* Command Register is selected */
    EN = 1;                /* High-to-Low pulse on Enable pin to latch data */ 
    NOP();
    EN = 0;
    __delay_ms(3); 
}

void LCD_Char(char dat)
{
    ldata= dat;            /* Send data to LCD */  
    RS = 1;                /* Data Register is selected */
    EN=1;                  /* High-to-Low pulse on Enable pin to latch data */   
    NOP();
    EN=0;
    __delay_ms(1);
}


void LCD_String(const char *msg)
{
    while((*msg)!=0)
    {       
      LCD_Char(*msg);
      msg++;    
        }
}

void LCD_String_xy(char row,char pos,const char *msg)
{
    char location=0;
    if(row<=1)
    {
        location=(0x80) | ((pos) & 0x0f); /*Print message on 1st row and desired location*/
        LCD_Command(location);
    }
    else
    {
        location=(0xC0) | ((pos) & 0x0f); /*Print message on 2nd row and desired location*/
        LCD_Command(location);    
    }  
    LCD_String(msg);

}







/*This code block configures the ADC
for polling, VDD and VSS references, ADCRC
oscillator and AN0 input.
Conversion start & polling for completion
are included.
 */


void ADC_Init(void)
{
       //Setup ADC
    ADCON0bits.FM = 1;  //right justify
    ADCON0bits.CS = 1; //ADCRC Clock
    
    TRISAbits.TRISA0 = 1; //Set RA0 to input
    ANSELAbits.ANSELA0 = 1; //Set RA0 to analog
    // Added 
    ADPCH = 0x00; //Set RA0 as Analog channel in ADC ADPCH
    ADCLK = 0x00; //set ADC CLOCK Selection register to zero
    
    ADRESH = 0x00; // Clear ADC Result registers
    ADRESL = 0x00; 
    
    ADPREL = 0x00; // set precharge select to 0 in register ADPERL & ADPERH
    ADPREH = 0x00; 
    
    ADACQL = 0x00;  // set acquisition low and high byte to zero 
    ADACQH = 0x00;    
    
    ADCON0bits.ON = 1; //Turn ADC On 
}


void check_keypad(){
    a=0;
    
    while (a<1) // wait until a button is pressed
    {
        
        
    PORTDbits.RD3=1; // check first column
    if (PORTDbits.RD7==1) button=1, a=a+1;
    else if (PORTDbits.RD6==1) button=4, a=a+1;
    else if (PORTDbits.RD5==1) button=7, a=a+1;
    PORTDbits.RD3=0;
    
   
    
    PORTDbits.RD2=1; // check 2nd column
    if (PORTDbits.RD7==1) button=2, a=a+1;
    else if (PORTDbits.RD6==1) button=5, a=a+1;
    else if (PORTDbits.RD5==1) button=8, a=a+1;
    else if (PORTDbits.RD4==1) button=0, a=a+1;
     PORTDbits.RD2=0;
    
    PORTDbits.RD1=1; // check 3rd column
    if (PORTDbits.RD7==1) button=3, a=a+1;
    else if (PORTDbits.RD6==1) button=6, a=a+1;
    else if (PORTDbits.RD5==1) button=9, a=a+1;
    PORTDbits.RD1=0;
    
    }
}

void initializePORTD(){
    PORTD=0;
    LATD=0;
    ANSELD=0;
    TRISD=0b11110000;
}



//void INTERRUPT_Initialize (void)
//{
//    INTCON0bits.IPEN=1; // Enable interrupt priority bit in INTCON0 (check INTCON0 register and find the bit)
//    INTCON0bits.GIEH=1;// Enable high priority interrupts using bits in INTCON0
//    INTCON0bits.GIEL=1;// Enable low priority interrupts using bits in INTCON0
//    INTCON0bits.INT0EDG=0;// Interrupt on falling  edge of INT0 pin using bits in INTCON0
//    IPR1bits.INT0IP=1;// Set the interrupt high priority (IP) for INT0 - INT0IP
//    PIE1bits.INT0IE=1;// Enable the interrupt (IE) for INT0
//
//   PIR5bits.INT1IF=0; //Clear interrupt flag for INT01
//  
//    // Change IVTBASE by doing the following
//    IVTBASEU=0x00; // Set IVTBASEU to 0x00 
//    IVTBASEH=0x40; // Set IVTBASEH to  0x40; 
//    IVTBASEL=0x08; // Set IVTBASEL to 0x08; 
//}
//void __interrupt(irq(IRQ_INT0),base(0x4008)) INT0_ISR(void)
//{//check if emergency button has been pressed
//    if (PIR1bits.INT0IF==1) // Check if interrupt flag for INT0 is set to 1 - (note INT0 is your input)
//    {
//        points=0;
//        
//        
//       PIR1bits.INT0IF=0; // always clear the interrupt flag for INT0 when done
//        
//    }
//    
//}
