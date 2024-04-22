int x;
int y;
int number1;
int number2;
int guess;
int secret_code;
unsigned char sevenSegValues[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x67,0x00,0x79};



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


