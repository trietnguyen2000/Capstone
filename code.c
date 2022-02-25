// Tentacle Arm
// code.c
// Triet Nguyen
// 01/17/2022

// Encoder board            Mega128 board 
// --------------      ----------------------    
//     sh/ld              PORTE bit 6
//     clk_inh            PORTE bit 7
//     sck                PORTB bit 1
//     ser_out            PORTB bit 3
//     vdd1                   vcc
//     gnd1                  ground

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
//DP,G,F,E,D,C,B,A
#define ZERO  0b11000000    // A, B, C, D, E, F
#define ONE   0b11111001    // B, C
#define TWO   0b10100100    // A, B, D, E, G
#define THREE 0b10110000    // A, B, C, D, G
#define FOUR  0b10011001    // B, C, F, G
#define FIVE  0b10010010    // A, C, D, F, G
#define SIX   0b10000010    // A, C, D, E, F, G
#define SEVEN 0b11111000    // A, B, C
#define EIGHT 0b10000000    // A, B, C, D, E, F, G
#define NINE  0b10011000    // A, B, C, F, G
#define COLON 0b11111100    // :
#define DP    0b01111111    // PM

#define SNOOZE_TIME 10

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "uart_functions.h"

uint8_t  count  = 0x00;
uint16_t sum    = 0x0000;

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

// Encoder status
uint8_t old_A[2];
uint8_t old_B[2];

// Display
uint8_t minutes = 0; // Clock minutes
uint8_t hours   = 0; // Clock hours

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {
  ZERO, 
  ONE, 
  TWO, 
  THREE, 
  FOUR, 
  FIVE, 
  SIX, 
  SEVEN, 
  EIGHT, 
  NINE, 
  COLON, 
  DP
};

/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){//MOSI
  DDRB  |=    (1<<PB0)  | (1<<PB2) | (1<<PB1); //Turn on SS, MOSI, SCLK (SRCLK)
  SPCR  |=    (1<<MSTR) | (1<<SPE);            //enable SPI, master mode 
  //SPSR  |=    (1<<SPI2X);                      // double speed operation
}//spi_init (Bar Graph)

//******************************************************************************
//                            spi_read
//  Reads the SPI port.
//******************************************************************************
uint8_t spi_read(void){
  SPDR = 0x00;                      //"dummy" write
  while(bit_is_clear(SPSR,SPIF)){}  //wait till 8 clock cycles are done
  return (SPDR);                    //return incoming data from SPDR
}

/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void){
  //timer counter 0 setup, running off i/o clock
  TIMSK |= (1<<TOIE0);                        //enable interrupts
  ASSR  |= (1<<AS0);                          //use external oscillator
  TCCR0 |= (0<<CS02) | ((0<<CS01) |1<<CS00);  //normal mode, no prescaler
}//tcnt0_init

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t hours, uint16_t minutes) {
  //determine how many digits there are
  //break up decimal sum into 4 digit-segments
  for(int i = 0; i < sizeof(segment_data); i++){
    if(i < 2){
        segment_data[i] = dec_to_7seg[minutes % 10];
        minutes /= 10;
    }
    else{
        segment_data[i] = dec_to_7seg[hours % 10];
        hours /= 10;
    }
  }
}//segment_sum

//***********************************************************************************
//                        Read encoder
//*************************************************************************/
void encoder_chk(uint8_t h, uint8_t m){
    uint8_t new_A[2], new_B[2];

    PORTE |= 0x80;                    // SH/LD low  (bit 6)
    PORTE &= ~(0x40);                 // CLK_INH high (bit 7)

    _delay_ms(1);

    PORTE |= 0x40;                    // CLK_INH low
    PORTE &= ~(0x80);                 // SH/LD high 

    uint8_t data = spi_read();        // Read current encoder state

    //Read new state of encoders (See which one is spinning)
    new_B[0] = ((data & 0x01) == 0) ? 0 : 1;
    new_A[0] = ((data & 0x02) == 0) ? 0 : 1;
    new_B[1] = ((data & 0x04) == 0) ? 0 : 1;
    new_A[1] = ((data & 0x08) == 0) ? 0 : 1;

    //sum = new_A[0];
    int counter = 0;
    // int return_val = -1; // default return value , no change
    for(int i = 0; i < 2; i++){
        if ((new_A[i] != old_A[i]) || (new_B[i] != old_B[i])){ //if change occurred 
        if ((new_A[i] == 0) && (new_B[i] == 0)) {
            if (old_A[i] == 1){
            if(i == 1){h++;}  
            else      {m++;} // sum += count;   
            } 
            else {
            if(i == 1){h--;}  
            else      {m--;} // sum -= count; 
            }
        }
        else if ((new_A[i] == 0) && (new_B[i] == 1)) {
            if (old_A[i] == 0){
            if(i == 1){h++;}  
            else      {m++;}// sum+=count;
            } 
            else {
            if(i == 1){h--;}  
            else      {m--;}// sum-=count;
            }
        }
        else if ((new_A[i] == 1) && (new_B[i] == 1)) {//detent position
            if (old_A[i] == 0){ if(counter ==  3){sum += count;}} //one direction 
            else              { if(counter == -3){sum -= count;}} //or the other
            counter = 0; //count is always reset in detent position 
        }
        else if ((new_A[i] == 1) && (new_B[i] == 0)) { 
            if (old_A[i] == 1) {
            if(i == 1){h++;}  
            else      {m++;}// sum+=count;
            }
            else {
            if(i == 1){h--;}  
            else      {m--;}// sum-=count;
            }
        }
        } //if change occurred
        old_A[i] = new_A[i]; //save what are now old values
        old_B[i] = new_B[i];
    }
    hours = h;
    minutes = m;
}

/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented. Every 7680 interrupts the minutes counter is incremented.
//TCNT0 interrupts come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_OVF_vect){
    
    uint8_t tempA = PORTA;
    uint8_t tempB = PORTB;
    uint8_t DDRA_temp = DDRA;
    uint8_t DDRB_temp = DDRB;

    encoder_chk(hours, minutes);
    segsum(hours,minutes);

    PORTA = tempA;
    PORTB = tempB;
    DDRA = DDRA_temp;
    DDRB = DDRB_temp;

}    

//********************************************************************************
//              MAIN
//********************************************************************************
int main()
{
  tcnt0_init();
  spi_init();
  sei();

  //set port bits 4-7 B as outputs
  DDRB  |= 0xF0; //Set to all outputs, change back to 4-7
  DDRE  |= 0xFF; //Set PORTE to output
  PORTE |= 0x00; //COM_LVL On
  PORTE |= 0x40; //SH/LD On

  DDRC  |= 0xFF;
  // DDRD  |= 0xFF; //Speaker on
  // DDRD  &= ~(0x20);

  while(1){

    PORTB = 0x00;
    
    /***********************************************
    //               7 Seg Display Output        
    ***********************************************/
    DDRA = 0xFF;
    int i = 0;
    while(i < 5){
      // if(segment_data[i] == COLON){
      //   segment_data[i] = 0xFF;
      // }
      //send 7 segment code to LED segments
      PORTA = segment_data[i];
      //send PORTB the digit to display
      PORTB = (i<<4);
      _delay_ms(1);
      i++;
    }
  }//while
  return 0;
}//main  