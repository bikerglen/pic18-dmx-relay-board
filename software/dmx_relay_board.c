/*
 * File:   dmx_relay_board.c
 * Author: glen
 *
 * Created on September 28, 2017, 3:47 PM
 * 
 * For use with DMX Driver Puck v2/untitled.brd.
 * 
 * Relays 1 to 4 correspond to four channels of DMX. 0-127 => relay off;
 * 128-255 => relay on. First DMX address is set by holding down button
 * during reset / power up then sending address of ch1 == 0xff. Relays 1 to 4
 *  then correspond to ch1 to ch4.
 * 
 * When DMX is not present, software falls through to simple
 * timer on/off of relays. Currently these are timed to control
 * a Spirit Halloween Hazmat Zombie prop with a fog machine.
 * 
 * LED is steady in standalone / simple timer mode.
 * LED blinks 1 2 pause, 1 2 pause, 1 2 pause when valid DMX received.
 * LED blinks quick 1 off, 1 off, 1 off when in addressing mode.
 */


#include <xc.h>

#include <P18cxxx.H>
#include <P18F1320.H>

// TMR1 preset value is (65536 - (FCY/PRESCALE/RATE))
// FCY is defined below, 4 x OSC FREQ if in HSPLL mode otherwise, OSC FREQ
// PRESCALE is 16
// RATE is 50

#pragma config WDT = OFF
#pragma config OSC = HSPLL
#pragma config MCLRE = OFF
#pragma config LVP = OFF

// #define ZOMBIE_FOGGER

#define FCY 40000000
#define U1BAUDRATE 250000
#define TMR1H_VAL 0x3c
#define TMR1L_VAL 0xb0
#define DEFAULT_MODE MODE_DMX_OK

// compute baud rate generator rate
#define U1BRGVAL (((FCY/U1BAUDRATE)/16) - 1)

#define RELAY_1_MASK		0x01
#define RELAY_2_MASK		0x02
#define RELAY_3_MASK    	0x04
#define RELAY_4_MASK        0x08

#define DEFAULT_DMX_ADDR    0x001   // dmx addresses 1,2,3,4 after power up until EEPROM read
#define EE_DMX_ADDR_HI      0x00    // address of high byte of dmx address in eeprom
#define EE_DMX_ADDR_LO      0x01    // address of low byte of dmx address in eeprom

// init hardware
void Init (void);

// variables to hold new relay values until their written to LATA
unsigned char new_level_ch1;
unsigned char new_level_ch2;
unsigned char new_level_ch3;
unsigned char new_level_ch4;

// receive state machine
unsigned char  rx_state;
unsigned short rx_addr;
unsigned char  rx_level;
unsigned short rx_ch1_addr;
unsigned short rx_ch2_addr;
unsigned short rx_ch3_addr;
unsigned short rx_ch4_addr;

enum {
	MODE_DMX_LOST = 0,
	MODE_DMX_OK = 1
};

unsigned char mode;
unsigned char timer;
unsigned char count;
unsigned short count2;

unsigned char green_led_timer;

// variables and prototypes used to assign a new dmx address
// if the button on RA4 is held down during power up
unsigned char startup_timer;
void GetNewDmxAddress (void);
unsigned char eeread (unsigned char address);
void eewrite (unsigned char address, unsigned char data);
unsigned short dmx_address;

unsigned short event_timer;


void main (void)
{
	Init ();
	
	mode = DEFAULT_MODE;
	timer = 0;
	count = 0;
	count2 = 0;

	green_led_timer = 0;
    event_timer = 0;

	// check if button held down during power up
	if (PORTAbits.RA4 == 0) {
		// must continue to be low for 50 times in the next 1 second
		startup_timer = 50;
		do {
			TMR1H = TMR1H_VAL; // @16MHz use 0xb1e0
			TMR1L = TMR1L_VAL; // @40MHz use 0x3cb0
			PIR1bits.TMR1IF = 0;

			if (PORTAbits.RA4) {
				startup_timer = 255;
				break;
			}

			while (PIR1bits.TMR1IF == 0) {
			}
		} while (--startup_timer);

		if (startup_timer == 0) {
			// get new dmx address store in eeprom
			GetNewDmxAddress ();
		}
	}
	
    // pull DMX address from EEPROM and constrain to 1 to 512
    dmx_address = (eeread (EE_DMX_ADDR_HI) << 8) | eeread (EE_DMX_ADDR_LO);
	if (dmx_address < 1) {
		dmx_address = 1;
	}
	if (dmx_address > 510) {
		dmx_address = 510;
	}

    // calculate new DMX addresses
    rx_ch1_addr = dmx_address + 0;
    rx_ch2_addr = dmx_address + 1;
    rx_ch3_addr = dmx_address + 2;
    rx_ch4_addr = dmx_address + 3;

	while (1) {

		// clear over run bit
		if (RCSTAbits.OERR) {
			RCSTAbits.CREN = 0;
			RCSTAbits.CREN = 1;
		}

		// DMX signal lost or not present
		if (mode == MODE_DMX_LOST) {
			// set timer1 counter to rollover after 20ms
			// that way main loop executes 50 times / second
			TMR1H = TMR1H_VAL; // @16MHz use 0xb1e0
			TMR1L = TMR1L_VAL; // @40MHz use 0x3cb0
			PIR1bits.TMR1IF = 0;

			if (timer == 6) {
				count = 0;
			} else {
				timer++;
			}

			// green LED is steady in stand alone mode
			LATBbits.LATB0 = 0;

            // process events

#ifdef ZOMBIE_FOGGER
            // turn on fog machine for 1.5 seconds every 30 seconds
            LATAbits.LATA2 = ((event_timer >= 0) && (event_timer <= 74)) ? 1 : 0;
            
            // turn on prop trigger for 0.5 seconds every 30 seconds
            LATAbits.LATA0 = ((event_timer >= 50) && (event_timer <= 74)) ? 1 : 0;
            
            // turn on hood lights all the time
            LATAbits.LATA3 = 1;

            // 30 seconds / 50 Hz = 1500 counts
            event_timer++;
            if (event_timer == 1500) {
                event_timer = 0;
                LATAbits.LATA0 = 0;
                LATAbits.LATA1 = 0;
                LATAbits.LATA2 = 0;
                LATAbits.LATA3 = 0;
            }
#endif
                        
			// wait until timer 1 rolls over -- after 50 breaks received in a row 
			// within the alotted time, switch to MODE_DMX_OK
			while (PIR1bits.TMR1IF == 0) {
				if (PIR1bits.RCIF) {
					if (RCSTAbits.FERR) {
						rx_level = RCREG;
						timer = 0;
						count++;
						if (count == 50) {
							count = 0;
							mode = MODE_DMX_OK;
						}
					} else {
						rx_level = RCREG;
					}
				}
			}

		// DMX signal present
		} else if (mode == MODE_DMX_OK) {
			if (PIR1bits.RCIF) {
				if (RCSTAbits.FERR) {
					rx_level = RCREG;
					rx_addr = 0;
					rx_state = 1;
				} else {
					rx_level = RCREG;
                    // uncomment this if DMX just required to be present to stay in DMX mode
                    count2 = 0;
					if (rx_state == 1) {
						if (rx_addr <= 512) {
							if (rx_addr == rx_ch1_addr) {
								new_level_ch1 = rx_level;
							} else if (rx_addr == rx_ch2_addr) {
								new_level_ch2 = rx_level;
							} else if (rx_addr == rx_ch3_addr) {
								new_level_ch3 = rx_level;
							} else if (rx_addr == rx_ch4_addr) {
								new_level_ch4 = rx_level;

                                LATAbits.LATA0 = (new_level_ch1 >= 128) ? 1 : 0;
                                LATAbits.LATA1 = (new_level_ch2 >= 128) ? 1 : 0;
                                LATAbits.LATA2 = (new_level_ch3 >= 128) ? 1 : 0;
                                LATAbits.LATA3 = (new_level_ch4 >= 128) ? 1 : 0;

								rx_state = 0;
                                // uncomment this if relays required to be addressed to leave DMX mode
                                // count2 = 0;
							}
							rx_addr++;
						}
					}
				}
			}
	
			// TMR1 is used to switch betwen modes and blink the front panel LED
			if (PIR1bits.TMR1IF) {
				// if DMX signal is lost, return to MODE_DMX_LOST
				TMR1H = TMR1H_VAL; // @16MHz use 0xb1e0
				TMR1L = TMR1L_VAL; // @40MHz use 0x3cb0
				PIR1bits.TMR1IF = 0;
				count2++;
				if (count2 == 500) { // 500 counts * .02 seconds / count = 10 seconds
					count2 = 0;
                    event_timer = 0;
                    LATAbits.LATA0 = 0;
                    LATAbits.LATA1 = 0;
                    LATAbits.LATA2 = 0;
                    LATAbits.LATA3 = 0;
					mode = MODE_DMX_LOST;
				}
		
				// blink green front panel led -- two quick blinks, a little slower pause
				green_led_timer++;
				if (green_led_timer == 1) {
					LATBbits.LATB0 = 0; // on
				} else if (green_led_timer == 7) {
					LATBbits.LATB0 = 1; // off
				} else if (green_led_timer == 13) {
					LATBbits.LATB0 = 0; // on
				} else if (green_led_timer == 19) {
					LATBbits.LATB0 = 1; // off
				} else if (green_led_timer == 50) {
					green_led_timer = 0;
				}
			}

		// handle illegal values of mode gracefully
		} else {
			mode = MODE_DMX_LOST;
		}
	}	
}


void Init (void)
{
	// configure ports a and b for digital I/O
	ADCON1 = 0x7f;

	// initialize port a
	LATA = 0x00;				// all LED drivers off
	TRISA = 0x10;				// only RA4 is an input

	// initialize port b
	LATB = 0x00;				// all outputs off
	TRISB = 0x10;				// RB0, RB1, RB2, RB3, RB5, RB6, RB7 are outputs; RX should be set as inputs

	// initialize uart
	RCSTA = 0x90;
	TXSTA = 0x24;
	BAUDCTL = 0x00;
	SPBRGH = 0x00;
	SPBRG = U1BRGVAL;

	// initialize timer 0
	T0CON = 0xC8;				// 8-bit mode clocked at Fosc / 4

	// initialize timer 1
	T1CON = 0xa1;				// 1:4 prescale, internal clock (Fosc/4)
								// final rate = 1MHz w/ 16MHz clock
								// final rate = 2.5MHz w/ 40MHz clock
	
	// initialize dmx receiver variables
	rx_state = 0;
	rx_level = 0;
	rx_addr = 0;

	// place RS-485 transceiver in receive mode
	LATBbits.LATB2 = 0;
	LATBbits.LATB3 = 0;
}


void GetNewDmxAddress (void)
{
	rx_level = 0;
	rx_addr = 0;
	rx_state = 0;
	green_led_timer = 0;

	while (1) {

		// clear over run bit if set
		if (RCSTAbits.OERR) {
			RCSTAbits.CREN = 0;
			RCSTAbits.CREN = 1;
		}

		// if a character is received
		if (PIR1bits.RCIF) {

			// if character had a framing error, assume it is a DMX break
			if (RCSTAbits.FERR) {
				rx_level = RCREG;
				rx_addr = 0;
				rx_state = 1;
			} else {
				// get level
				rx_level = RCREG;

				// ignore level if not in state 1
				if (rx_state == 1) {
					if ((rx_addr >= 1) && (rx_addr <= 512)) {

						// search for first channel with max value
						if (rx_level == 0xff) {
							// we have the new dmx address in rx_addr
							break;
						}
					}

					// increment dmx address and check bounds
					rx_addr++;
					if (rx_addr > 512) {
						rx_state = 0;
					}
				}
			}
		}
	
		// one quick blink and a long pause
		if (PIR1bits.TMR1IF) {
			TMR1H = TMR1H_VAL; // @16MHz use 0xb1e0
			TMR1L = TMR1L_VAL; // @40MHz use 0x3cb0
			PIR1bits.TMR1IF = 0;
			green_led_timer++;
			if (green_led_timer == 3) {
				LATBbits.LATB0 = 1;
			} else if (green_led_timer == 30) {
				LATBbits.LATB0 = 0;
				green_led_timer = 0;
			}
		}
	}	

	// store new light number in new_level_red in to eeprom
	eewrite (EE_DMX_ADDR_HI, (rx_addr >> 8) & 0xff);
	eewrite (EE_DMX_ADDR_LO, (rx_addr >> 0) & 0xff);
    
    // turn all relays off
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
        
    // turn LED on
	LATBbits.LATB0 = 0;

	// wait one second
	startup_timer = 50;
	green_led_timer = 0;
	do {
		TMR1H = TMR1H_VAL; // @16MHz use 0xb1e0
		TMR1L = TMR1L_VAL; // @40MHz use 0x3cb0
		PIR1bits.TMR1IF = 0;

		green_led_timer++;
		if (green_led_timer == 3) {
			LATBbits.LATB0 = 0;
		} else if (green_led_timer == 15) {
			LATBbits.LATB0 = 1;
			green_led_timer = 0;
		}

		while (PIR1bits.TMR1IF == 0) {
		}
	} while (--startup_timer);
}


unsigned char eeread (unsigned char address)
{
	EEADR = address;
	EECON1bits.CFGS = 0;
	EECON1bits.EEPGD = 0;
	EECON1bits.RD = 1;
	return EEDATA;
}


void eewrite (unsigned char address, unsigned char data)
{
	INTCONbits.GIEH = 0;		// disable high priority interrupts
	INTCONbits.GIEL = 0;		// disable low priority interrupts

	EECON1bits.CFGS = 0;
	EECON1bits.EEPGD = 0;
	EECON1bits.WREN = 1;
	EEADR = address;
	EEDATA = data;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	while (!PIR2bits.EEIF)
	;
	PIR2bits.EEIF = 0;

	INTCONbits.GIEH = 1;		// enable high priority interrupts
	INTCONbits.GIEL = 1;		// enable low priority interrupts
}
