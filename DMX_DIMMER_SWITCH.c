/*------------------------------------------------------------------------------
 Copyright:      Radig Ulrich  mailto: mail@ulrichradig.de
 Author:         Radig Ulrich
 Remarks:
 known Problems: none
 Version:        19.04.2017
 Description:    DMX_8 Kanal_Dimmer
 Modified by Aliste.ware mailto: alister.ware@ntlworrld.com
 changed pins fro dip switch
 *reconfigure switch 10 & switch 9 for debug mode
------------------------------------------------------------------------------*/


#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

#ifndef F_CPU		//I prefer to define freq in the make file
#define F_CPU 12000000
#endif

#define LED_OUT		DDRD |= (1<<PD5);
#define LED_TOGGLE	PORTD ^= (1<<PD5);
#define LED_ON	PORTD |= (1<<PD5);
#define LED_OFF PORTD &= !(1<<PD5);

#define DMX_BAUD 250000
#define DMX_LOST_TIMEOUT 8000
// map table with 255 values. Three times the same value scaled down at the end
//uint8_t valMap[] = {90,89,89,88,88,87,87,...7,7,6,6,5,5,4,4,3,3,2,2,1,0,0};

volatile unsigned char dmx_buffer[513];
volatile unsigned int dmx_lost = DMX_LOST_TIMEOUT;

volatile unsigned int dmx_adresse = 0;
volatile unsigned char phase_on_count = 0;
volatile unsigned int live_counter = 0;

volatile unsigned char val[8];
volatile int fade_delay = 0;
volatile int fade_pos =0;
uint8_t fade_dir = 1;

#define SPI_DDR                 DDRB
#define SPI_PORT                PORTB
#define SPI_SCK                 5
#define SPI_MOSI                3
#define CS_595                  2

#define CS_595_HI()				PORTB &= ~(1<<CS_595)
#define CS_595_LO()				PORTB |= (1<<CS_595)

void out_ext (unsigned char value);

ISR (TIMER2_COMPA_vect)
{
	unsigned char out = 0;
	unsigned char tmp = 0;
	phase_on_count++;

	if(phase_on_count > 240){
		phase_on_count = 0;
	}

	for(tmp = 0;tmp <8;tmp++){
		if(phase_on_count>val[tmp]) out |= (1<<tmp);
//		if(phase_on_count>65) out |= (1<<tmp);
	}
	out_ext(out);

	if(dmx_lost<DMX_LOST_TIMEOUT){
		dmx_lost++;
	}
}

 ISR (TIMER1_COMPA_vect)
 {
	if (fade_dir){
		fade_pos++;
		if (fade_pos ==0){
			fade_dir = 0;
		}
	} else {
		fade_pos--;
		if (fade_pos ==0xff){
			fade_dir = 1;
		}
	}
}

//############################################################################
//Receiving routine for DMX
ISR (USART_RX_vect){
	static unsigned int dmx_channel_rx_count = 0;
	static unsigned char dmx_valid = 0;
	unsigned char tmp = 0;
	tmp =  UDR0;
//                                                                                                                                                                                                                                                                                 	LED_ON
	live_counter=0;
	if(UCSR0A&(1<<FE0)){
		if(dmx_channel_rx_count > 1){
			dmx_lost = 0;
		}
		dmx_channel_rx_count = 0;
		dmx_buffer[0] = tmp;
		if(dmx_buffer[0] == 0){
			dmx_valid = 1;
			dmx_channel_rx_count++;
		}else{
			dmx_valid = 0;
		}
		return;
	}

	if(dmx_valid){
		dmx_buffer[dmx_channel_rx_count] = tmp;
		if(dmx_channel_rx_count < 513)
		{
			dmx_channel_rx_count++;
		}
		return;
	}
}

//############################################################################
//Port extentions OUTPUT 1 - 8 via SPI-BUS
void out_ext (unsigned char value){
	CS_595_LO();
	SPDR = value;
	while( !(SPSR & (1<<SPIF)) ) ;
	CS_595_HI();
}


//############################################################################
//
static inline void spi_init(void){
	// configure pins MOSI, SCK and CS595 as output
	SPI_DDR |= (1<<SPI_MOSI) | (1<<SPI_SCK) | (1<<CS_595);
	// pull SCK high
	SPI_PORT |= (1<<SPI_SCK);
	//SPI: enable, master, positive clock phase, msb first, SPI speed fosc/2
	SPCR = (1<<SPE) | (1<<MSTR);
	SPSR = (1<<SPI2X);

	out_ext(0x00);
}

//############################################################################
//Time is counted here (Tick 100µs
//############################################################################
//ISR from external Interrupt (ICP
ISR (PCINT0_vect){
	TCNT2 = 0;
	phase_on_count = 0;
}

void fade_test(void){
	}

void clear_all(){
	unsigned char tmp = 0;
	for(tmp=0;tmp<8;tmp++){
		val[tmp]=0xff;
	}
}

void debug(void){
	char temp = 0;
	clear_all();
		if(!(PIND&(1<<PD7))) val[7] = temp;   //switch 1    PD4   11	 PD7
		if(!(PIND&(1<<PB1))) val[6] = temp;   //switch 2    PD3   13   PB1
		if(!(PINC&(1<<PC0))) val[5] = temp;   //switch 3    PC5   23   PC0
		if(!(PINC&(1<<PC1))) val[4] = temp;   //switch 4    PC4   24   PC1
		if(!(PINC&(1<<PC2))) val[3] = temp;   //switch 5    PC3   25   PC2
		if(!(PINC&(1<<PC3))) val[2] = temp;   //switch 6    PC2   26   PC3
		if(!(PINC&(1<<PC4))) val[1] = temp;   //switch 7    PC1   27   PC4
		if(!(PINC&(1<<PC5))) val[0] = temp;   //switch 8    PC0   28   PC5
		dmx_lost = 0; //disable check for DMX timeout as not req.
}

//############################################################################
//Main programm
int main (void)
{
	unsigned int dmx_adresse_tmp = 0;
	unsigned char tmp = 0;

	LED_OUT;
	spi_init();

	//PIN CHANGE INTERRUPT ON PHASE SYNC
	PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT0);

	//Init usart DMX-BUS
	UBRR0   = (F_CPU / (DMX_BAUD * 16L) - 1);
	UCSR0B|=(1 << RXEN0 | 1<< RXCIE0);
	UCSR0C|=(1<<USBS0); //USBS0 2 Stop bits
	sei();//Globale Interrupts Enable

	//Switch 75176 to INPUT RXD
	DDRD |= (1<<PD2);
	PORTD &= ~(1<<PD2);

	//Timer for the phase angle calculation
	TCCR2A |= (1<<WGM21);
	TCCR2B |= (1<<CS21|1<<CS20); // :32
	TIMSK2 |= (1<<OCIE2A);

	// :32 Teiler
	// :2 Clock Source
	// :50 Hz
	// :92 - 1 Stepping

	OCR2A = (F_CPU/32/2/50/240) - 1;
	//Timer for fade up/fade down
	TCCR1A |= (1<<WGM21);
	TCCR1B |= (0<<CS21|1<<CS20); // :32

	TIMSK1 |= (1<<OCIE1A);

	// :32 Teiler
	// :2 Clock Source
	// :50 Hz
	// :92 - 1 Stepping

	OCR1A = (F_CPU/32/2/50/240) - 1;

	//Pullup for DIP Switch
	PORTD |= (1<<PD3)|(1<<PD4)|(1<<PD7);
	PORTC |= (1<<PC5)|(1<<PC4)|(1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0);
	PORTB |= (1<<PB1);
	//Infinate loop
	while(1){
		//Read DMX Address from Switch
		dmx_adresse_tmp = 0;                            //           orig        new
		if(!(PIND&(1<<PD7))) dmx_adresse_tmp |= 0x01;   //switch 1    PD4   11	 PD7
 		if(!(PIND&(1<<PB1))) dmx_adresse_tmp |= 0x02;   //switch 2    PD3   13   PB1
		if(!(PINC&(1<<PC0))) dmx_adresse_tmp |= 0x04;   //switch 3    PC5   23   PC0
		if(!(PINC&(1<<PC1))) dmx_adresse_tmp |= 0x08;   //switch 4    PC4   24   PC1
		if(!(PINC&(1<<PC2))) dmx_adresse_tmp |= 0x10;   //switch 5    PC3   25   PC2
		if(!(PINC&(1<<PC3))) dmx_adresse_tmp |= 0x20;   //switch 6    PC2   26   PC3
		if(!(PINC&(1<<PC4))) dmx_adresse_tmp |= 0x40;   //switch 7    PC1   27   PC4
		if(!(PINC&(1<<PC5))) dmx_adresse_tmp |= 0x80;   //switch 8    PC0   28   PC5
		if(!(PINB&(1<<PD3))) dmx_adresse_tmp |= 0x0100; //switch 9    PB1    2   PD3
		                  //	        				  switch 10   PD7    1   PD4
		if(dmx_adresse_tmp == 0) dmx_adresse_tmp = 1;
		if(dmx_adresse_tmp > 505) dmx_adresse_tmp = 505;
		if(dmx_adresse != dmx_adresse_tmp) dmx_adresse =  dmx_adresse_tmp;

		//new calculation of val

		if(PIND&(1<<PD4)){   //	      PD7				  switch 10   PD7    1   PD4
			for(tmp = 0;tmp <8;tmp++){
				val[tmp] = (255 - dmx_buffer[dmx_adresse+tmp]);
			}
		}else{   //debug code goes here
			debug();
		}
		if(dmx_lost>=DMX_LOST_TIMEOUT){
			for(tmp = 0;tmp <8;tmp++){
				dmx_buffer[dmx_adresse + tmp] = 0;
			}
		}
		if((live_counter++) > 1000) {
//			LED_OFF
			live_counter = 0;
		}

	}
}
