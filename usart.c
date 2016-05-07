/* Authors:       Sergiy Kolodyazhnyy, Anthony Thao, 
 * Course :       Interface Techniques , EET 4340 , Sprint 2016
 * Assignment:    Final project
 * Description:   The following program uses PIC 18F87J11 microcontroller
 *                to interface it with Raspberry Pi 2 over asynchronous serial 
 *                PIC performs certain operations upon receiving one of 3
 *                inputs from Raspberry
 * Tools used: MPLAB , XC8, PICDem board 
 */



#include <xc.h>
#include "LCD.h"
#include <stdio.h>

/* LCD.h provided by the instructor
 */

#if defined __18F8722
#pragma config OSC=HSPLL
#pragma config WDT=OFF
#pragma config LVP=OFF
#pragma config XINST=OFF
#elif defined __18F87J11
#define _XTAL_FREQ=40000000
#pragma config FOSC=HSPLL
#pragma config WDTEN=OFF
#pragma config XINST=OFF
#else
#error Invalid processor selection
#endif


void InitPins(void);
void ConfigInterrupts(void);
void ConfigPeriph(void);
unsigned int ReadPot(void);
unsigned char ReadSerial(void);

unsigned char received;
unsigned char received2;
unsigned char output[20] ;
unsigned int potentiometer;

void main(void) 
{

    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    LCDClear();
    InitPins();
    ConfigPeriph();
    ConfigInterrupts();

    while (1) {
	    received = ReadSerial();
	    LCDClear();
            switch(received) {
                case 'a':
                         received2 = ReadSerial();  
                         sprintf(output,"Received char:%c",received2); 
			/*
			 *  FIXME : PIC18 doesn't transmit back to RPi
			 *  python script hangs on serial.read(1)
			 *  Same behavior with cat -A /dev/ttyACM0 
			 *
			 * 
			 * _delay(2); // 2 cycle delay
			 *while(!PIR1bits.TX1IF);
			 *sprintf(output,"TRANSMIT");
			 *LCDWriteLine(output,0);
			 *TXREG1=received2;
                         *
			 */
			 break ;
                case 'b':
                         potentiometer = ReadPot();
                         sprintf(output,"Pot value: %d", potentiometer);
                         LATD =~ LATD;
                         break;
                  
                case 'c':
			   received2=ReadSerial();
			   sprintf(output,"Set LED to 0x%X",received2);
                           LATD = received2;
                           break;
                default:
                    sprintf(output, ">>UNKNOWN %c",received);
                    break;
             
            }
        LCDWriteLine(output,0);
	
	
    }
}

void InitPins(void){
    LATD = 0; //LED's are outputs
    TRISD = 1; //Turn on all LED's
    
    //Set TRIS bits for any required peripherals here.
    TRISC = 0b10000000; //RC7 is RX, RC6 is TX
    //Set up for ADC
    TRISA = 0b00000001;
    ADCON1 = 0b10111010; //Right justify, No calibration, 20 Tad, FOSC/32
    WDTCONbits.ADSHR = 1; //Switch to alternate address to access ANCON0
    ANCON0 = 0b11111110; //AN0 analog - all others digital
    WDTCONbits.ADSHR = 0; //Back to default address
}

unsigned char ReadSerial(void)
{
 /* Wait for a byte and return it
  */
  while (PIR1bits.RC1IF == 0);
  return RCREG1; 
}

void ConfigInterrupts(void) 
{
    RCONbits.IPEN = 0; //no priorities.  This is the default.
    //Configure your interrupts here
    //Interrupt on USART transmit buffer empty
    PIE1bits.TX1IE = 1; //enable tx interrupt
    INTCONbits.PEIE = 1; //turn on peripheral interrupts
    INTCONbits.GIE = 1; //Turn on interrupts
}

void ConfigPeriph(void)
{
    /*  Configure the USART for 9600 baud asychronous transmission
     *  Frequency of oscillation is 40 MHz , hence SPBRG = 1040
     *  Raspberry runs at 115200 by default so , SPBRG=86 may be a better
     *  alternative
     */
    SPBRG1 = 1040; 
    SPBRGH1 = 1040 >> 8;
    TXSTA1bits.BRGH = 1;
    BAUDCON1bits.BRG16 = 1;
    TXSTA1bits.SYNC = 0; //asynchronous mode
    RCSTA1bits.SPEN = 1; //Enable the serial port
    RCSTA1bits.CREN = 1; //Enable reception
}

unsigned int ReadPot(void)
{
    ADCON0bits.CHS = 0; //channel 0
    ADCON0bits.ADON = 1; 
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1);
    ADCON0bits.ADON = 0;
    return ADRES;
}
