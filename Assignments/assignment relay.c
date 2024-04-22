



// PIC18F46K42 Configuration Bit Settings

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


#define _XTAL_FREQ 4000000  // Fosc  frequency for _delay()  library
#define FCY    _XTAL_FREQ/4
int x;
int y;
int number1;
int number2;
int guess;
int secret_code;
unsigned char sevenSegValues[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x67,0x00,0x79};

void initializePORTB(void); //inputs: photoresistors,button
void initializePORTD(void); //seven segment
void initializePORTA(void); //motor and relay
// Defining Interrupt ISR 
//emergency switch
void __interrupt(irq(IRQ_INT0),base(0x4008)) INT0_ISR(void)
{//check if emergency button has been pressed
    if (PIR1bits.INT0IF==1) // Check if interrupt flag for INT0 is set to 1 - (note INT0 is your input)
    {
       
        
        if (PORTBbits.RB3 ==0){ // preventing interrupt from accidentally going off when not supposed too
     
        PORTAbits.RA1=1;//beep a little bop
        __delay_ms(100);
        PORTAbits.RA1=0;
        __delay_ms(300);
        
        PORTAbits.RA1=1;
        __delay_ms(100);
        PORTAbits.RA1=0;
         __delay_ms(100);
         
        PORTAbits.RA1=1;
        __delay_ms(100);
        PORTAbits.RA1=0;
        __delay_ms(100);
        
        PORTAbits.RA1=1;
        __delay_ms(200);
        PORTAbits.RA1=0;
        __delay_ms(100);
        
        PORTAbits.RA1=1;
        __delay_ms(100);
        PORTAbits.RA1=0;
        __delay_ms(500);
        
        PORTAbits.RA1=1;
        __delay_ms(200);
        PORTAbits.RA1=0;
        __delay_ms(100);
        
        PORTAbits.RA1=1;
        __delay_ms(200);
        PORTAbits.RA1=0;
        }
       PIR1bits.INT0IF=0; // always clear the interrupt flag for INT0 when done
        
    }
    
}

void INTERRUPT_Initialize (void)
{
    INTCON0bits.IPEN=1; // Enable interrupt priority bit in INTCON0 (check INTCON0 register and find the bit)
    INTCON0bits.GIEH=1;// Enable high priority interrupts using bits in INTCON0
    INTCON0bits.GIEL=1;// Enable low priority interrupts using bits in INTCON0
    INTCON0bits.INT0EDG=0;// Interrupt on falling  edge of INT0 pin using bits in INTCON0
    IPR1bits.INT0IP=1;// Set the interrupt high priority (IP) for INT0 - INT0IP
    PIE1bits.INT0IE=1;// Enable the interrupt (IE) for INT0

   PIR5bits.INT1IF=0; //Clear interrupt flag for INT01
  
    // Change IVTBASE by doing the following
    IVTBASEU=0x00; // Set IVTBASEU to 0x00 
    IVTBASEH=0x40; // Set IVTBASEH to  0x40; 
    IVTBASEL=0x08; // Set IVTBASEL to 0x08; 
}

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
        PIR1bits.INT0IF=0; //fixed interrupt
        __delay_ms(2000);
        PIR1bits.INT0IF=0; //fixed interrupt
        PORTBbits.RB3=0;
        PIR1bits.INT0IF=0; //fixed interrupt
        
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
