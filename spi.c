// spi.c
// Functions to drive the SPI1 module on the PIC18F26Q71 microcontroller
// as a master device.
//
// PJ, 2026-03-11

#include <xc.h>
#include <stdint.h>
#include "global_defs.h"
#include "spi.h"

static uint8_t spi1_is_initialized = 0;
static uint8_t upin_ss = 0;

void spi1_init(uint8_t upin, uint8_t ckp, uint8_t cke, uint8_t smp)
{
    upin_ss = upin & 0b11; // Only values 0 through 3 are valid options.
    switch (upin_ss) {
        case 0:
            LATAbits.LATA4 = 1;
            ANSELAbits.ANSELA4 = 0;
            TRISAbits.TRISA4 = 0;
            ODCONAbits.ODCA4 = 0;
            WPUAbits.WPUA4 = 0;
            break;
        case 1:
            LATAbits.LATA5 = 1;
            ANSELAbits.ANSELA5 = 0;
            TRISAbits.TRISA5 = 0;
            ODCONAbits.ODCA5 = 0;
            WPUAbits.WPUA5 = 0;
            break;
        case 2:
            LATAbits.LATA6 = 1;
            ANSELAbits.ANSELA6 = 0;
            TRISAbits.TRISA6 = 0;
            ODCONAbits.ODCA6 = 0;
            WPUAbits.WPUA6 = 0;
            break;
        case 3:
            LATAbits.LATA7 = 1;
            ANSELAbits.ANSELA7 = 0;
            TRISAbits.TRISA7 = 0;
            ODCONAbits.ODCA7 = 0;
            WPUAbits.WPUA7 = 0;
            break;
    }
    //
    // Configure the SPI1 module as master and assign IO pins.
    SPI1CON0bits.EN = 0;
    SPI1CLK = 0b00010; // MFINTOSC (500 kHz)
    SPI1BAUD = 0; // 250 kHz should be slow enough for all sorts of devices
    SPI1CON0bits.MST = 1;
    SPI1CON1bits.CKP = ckp;
    SPI1CON1bits.CKE = cke;
    SPI1CON1bits.SMP = smp;
    // SCK1 = RC2, SDO1 = RC1, SDI1 = RC0
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    SPI1SCKPPS = 0b010010; // RC2
    RC2PPS = 0x1D; // SPI1 SCK
    RC1PPS = 0x1E; // SPI1 SDO
    SPI1SDIPPS = 0b010000; // RC0
    switch (upin_ss) {
        case 0:
            SPI1SSPPS = 0b000100; // RA4
            RA4PPS = 0x1F; // SPI1 SS
            break;
        case 1:
            SPI1SSPPS = 0b000101; // RA5
            RA5PPS = 0x1F; // SPI1 SS
            break;
        case 2:
            SPI1SSPPS = 0b000110; // RA6
            RA6PPS = 0x1F; // SPI1 SS
            break;
        case 3:
            SPI1SSPPS = 0b000111; // RA7
            RA7PPS = 0x1F; // SPI1 SS
            break;
    }
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELCbits.ANSELC0 = 0; // allow digital use
    ANSELCbits.ANSELC1 = 0;
    ANSELCbits.ANSELC2 = 0;
    TRISCbits.TRISC0 = 1; // SDI
    WPUCbits.WPUC0 = 1; // pull-up the SDI, in case there is nothing attached
    TRISCbits.TRISC1 = 0; // SD0
    TRISCbits.TRISC2 = 0; // SCK
    //
    SPI1CON0bits.EN = 1;
    spi1_is_initialized = 1;
}

void spi1_close(void)
{
    if (!spi1_is_initialized) return;
    //
    SPI1CON0bits.EN = 0;
    // Return the slave-select pin to the GPIO pool but
    // leave other assigned SPI port pins as they are..
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    switch (upin_ss) {
        case 0:
            RA4PPS = 0; // LATA4
            break;
        case 1:
            RA5PPS = 0; // LATA5
            break;
        case 2:
            RA6PPS = 0; // LATA6
            break;
        case 3:
            RA7PPS = 0; // LATA7
            break;
    }
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    // Set chip-select pin back to a safe mode. 
    switch (upin_ss) {
        case 0:
            ANSELAbits.ANSELA4 = 1;
            TRISAbits.TRISA4 = 1;
            ODCONAbits.ODCA4 = 0;
            WPUAbits.WPUA4 = 1;
            break;
        case 1:
            ANSELAbits.ANSELA5 = 1;
            TRISAbits.TRISA5 = 1;
            ODCONAbits.ODCA5 = 0;
            WPUAbits.WPUA5 = 1;
            break;
        case 2:
            ANSELAbits.ANSELA6 = 1;
            TRISAbits.TRISA6 = 1;
            ODCONAbits.ODCA6 = 0;
            WPUAbits.WPUA6 = 1;
            break;
        case 3:
            ANSELAbits.ANSELA7 = 1;
            TRISAbits.TRISA7 = 1;
            ODCONAbits.ODCA7 = 0;
            WPUAbits.WPUA7 = 1;
            break;
    }
    spi1_is_initialized = 0;
}

uint8_t spi1_exchange(uint8_t n, uint8_t* buf)
{
    uint8_t n_sent = 0;
    if (!spi1_is_initialized) return 0;
    // [TODO] let the hardware control the SS pin
    switch (upin_ss) {
        case 0:
            LATAbits.LATA4 = 0;
            break;
        case 1:
            LATAbits.LATA5 = 0;
            break;
        case 2:
            LATAbits.LATA6 = 0;
            break;
        case 3:
            LATAbits.LATA7 = 0;
            break;
    }
    __delay_us(10);
    // [TODO]
    __delay_us(10);
    switch (upin_ss) {
        case 0:
            LATAbits.LATA4 = 1;
            break;
        case 1:
            LATAbits.LATA5 = 1;
            break;
        case 2:
            LATAbits.LATA6 = 1;
            break;
        case 3:
            LATAbits.LATA7 = 1;
            break;
    }
    return n_sent;
}
