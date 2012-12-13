#include <hidef.h>      /* common defines and macros */
#include <MC9S12C128.h> /* derivative information */
#include "PLL.h"


void setclk24() // scale ECLOCK to 24 MHz from 4 MHz OSC
{   // scale PLLCLK to 2*desired ECLOCK . PLLCLK = 2 * OSCCLK * (SYNR + 1)/(REFDV + 1) 

	SYNR =  5;  
	REFDV = 0;
	PLLCTL = 0x60;   
	while(!(CRGFLG & 0x08));  // wait for PLL to lock onto target frequency
	CLKSEL = 0x80;
}

