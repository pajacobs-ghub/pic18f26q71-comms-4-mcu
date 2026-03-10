// comms-4-mcu.c
// RS485 communications MCU using the PIC18F26Q71.
// A Pico2 is the DAQ-MCU.
//
// Peter J.
// 2025-10-07 First cut is to adapt the COMMS-2 firmware from the PIC18f16Q41
// 2025-10-04 Edit PCB tracks so that we can use negative-channel inputs to 
//            comparators 1 and 2 for the internal and external trigger signals.
// 2025-11-09 Clean up the functions and put them to use.
// 2025-11-11 Simple hardware trigger function, as used for PIC18F16Q41-COMMS-1.
// 2025-11-14 Utility pins are available for analog and digital functions.
// 2026-03-02 Adapt from the PIC18F26Q71-COMMS-3 firmware and add the SPI-clock
//            buffering from the PIC18F16Q41-COMMS-2 firmware
//
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config BBEN = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config DEBUG = OFF
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
// BBSIZE = No Setting

// CONFIG8
#pragma config SAFSZ = SAFSZ_NONE

// CONFIG9
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG10
#pragma config CPD = OFF

// CONFIG11
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>

#define VERSION_STR "v0.2 PIC18F26Q71 COMMS-4-MCU 2026-03-10"

// Each device on the RS485 network has a unique single-character identity.
// The master (PC) has identity '0'. Slave nodes may be 1-9A-Za-z.
// When programming each device, select a suitable value for MYID.
#define MYID 'F'

// The following definitions are for sensing.
#define PICO2_READY_PIN (PORTBbits.RB4)
#define EVENTn_PIN (PORTBbits.RB3)

// The following definition is for output.
#define PICO2_RESETn_PIN (LATBbits.LATB5)

void init_pins()
{
    // RB4 as digital-input for DAQ-MCU Ready/Busy# signal.
    TRISBbits.TRISB4 = 1;
    ANSELBbits.ANSELB4 = 0;
    //
    // RB5 as digital-output for restart of Pico2 DAQ_MCU,
    // open-drain, weak pull-up is not needed.
    ODCONBbits.ODCB5 = 1;
    // WPUBbits.WPUB5 = 1;
    PICO2_RESETn_PIN = 1;
    TRISBbits.TRISB5 = 0;
    ANSELBbits.ANSELB5 = 0;
    //
    // Initially, we will be sensing the EVENTn line,
    // to see if other devices have pulled the line low
    // to indicate the event.
    // Later it may be driven low by software command or
    // it may be driven by hardware (comparator latched by CLC3).
    ANSELBbits.ANSELB3 = 0;
    TRISBbits.TRISB3 = 1; // input EVENTn line
    // The following lines are compatible with the un-asserted state,
    // for when the pin is driven by this device.
    ODCONBbits.ODCB3 = 1;
    LATBbits.LATB3 = 1;
    //
    // Use RA3(out),RA2(in) to buffer SPICLK from Pico2
    //
    ANSELAbits.ANSELA2 = 0;
    TRISAbits.TRISA2 = 1; // input from Pico2
    ANSELAbits.ANSELA3 = 0;
    LATAbits.LATA3 = 0;
    TRISAbits.TRISA3 = 0; // output to MCP3301
    // Use CLC2 to reflect RA2 onto RA3.
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b01; // To select CLC2 registers for the following settings.
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = 0; // data1 gets CLCIN0PPS as input
    CLCnSEL1 = 0; // data2 gets CLCIN0PPS as input, also
    CLCnSEL2 = 0; // data3 gets CLCIN0PPS as input, but gets ignored in logic select
    CLCnSEL3 = 0; // data4 as for data3
    // Logic select into gates
    CLCnGLS0 = 0b0010; // data1 goes through true to gate 1
    CLCnGLS1 = 0b1000; // data2 goes through true to gate 2
    CLCnGLS2 = 0; // gate 3 gets logic 0
    CLCnGLS3 = 0; // gate 4 gets logic 0
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is AND-OR
    CLCnCONbits.MODE = 0b000;
    CLCnPOLbits.POL = 0;
    // Connect CLC2 to the RA5,RA4 pins.
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    CLCIN0PPS = 0b000010; // RA2
    RA3PPS = 0x02; // CLC2OUT
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    // Now that the connections are made and function selected, enable it.
    CLCnCONbits.EN = 1;
    //
    // Analog pins
    // External trigger signal is at RA0.
    TRISAbits.TRISA0 = 1;
    ANSELAbits.ANSELA0 = 1;
    // V_REF_A input pin ANA1/RA1.
    TRISAbits.TRISA1 = 1;
    ANSELAbits.ANSELA1 = 1;
    // V_REF_B is also of interest on RB1
    TRISBbits.TRISB1 = 1;
    ANSELBbits.ANSELB1 = 1;
    return;
}

static inline void assert_event_pin()
{
    // Actively pull the event pin low.
    LATBbits.LATB3 = 0;
    ODCONBbits.ODCB3 = 1;
    TRISBbits.TRISB3 = 0; // output EVENTn line
}

static inline void release_event_pin()
{
    // Release the drive on the pin and allow it to be pulled high.
    LATBbits.LATB3 = 1;
    TRISBbits.TRISB3 = 1; // input EVENTn line
}

void FVR_init()
{
    // We want to supply both the ADC and the DAC with the same reference
    // voltage that suits a 5V system.
    FVRCONbits.ADFVR = 3;  // 4v096
    FVRCONbits.CDAFVR = 3; // 4v096
    FVRCONbits.EN = 1;
    while (!FVRCONbits.RDY) { /* should be less than 25 microseconds */ }
    return;
}

void FVR_close()
{
    FVRCONbits.EN = 0;
    FVRCONbits.ADFVR = 0;
    FVRCONbits.CDAFVR = 0;
    return;
}

#define ADC_RA0 0x00
#define ADC_RA1 0x01
#define ADC_RB1 0x09

void ADC_init()
{
    // Set up the ADC to look at one of the analog pins. 
    //
    ADCON0bits.CS = 1; // use dedicated RC oscillator
    ADCON0bits.IC = 0; // single-ended input
    ADCON0bits.FM = 1; // right-justified result
    ADCON2bits.MD = 0b011; // burst-average mode
    PIR1bits.ADIF = 0;
    ADREFbits.NREF = 0; // negative reference is Vss
    ADREFbits.PREF = 0b11; // positive reference is FVR
    ADACQ = 0x10; // 16TAD acquisition period
    ADNCH = 0b1110111; // select VSS
    ADPCH = ADC_RA0;
    ADCON0bits.ON = 1; // power on the device
    return;
}

void ADC_select(uint8_t byte_value)
{
    ADPCH = byte_value;
    return;
}

uint16_t ADC_read()
{
    // Returns the value from the ADC when looking at the input pin
    // for the comparator.
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { /* wait, should be brief */ }
    PIR1bits.ADIF = 0;
    return ADRES;
}

void ADC_close()
{
    ADCON0bits.ON = 0;
    return;
}

void set_VREF_AB(uint8_t levelA, uint8_t levelB)
{
    // Assuming that the fixed voltage reference is on at 4v096,
    // take a fraction of that reference voltage and feed it through the
    // 8-bit DAC2 and then through the OPA1 to the external pin (OPA1OUT/RA1)
    // to get VREF_A.
    //
    DAC2CONbits.PSS = 0b10; // FVR Buffer 2
    DAC2CONbits.NSS = 0; // VSS
    DAC2CONbits.EN = 1;
    DAC2DATL = levelA;
    //
    OPA1CON0bits.UG = 1; // unity gain
    OPA1CON0bits.CPON = 1; // charge-pump is on, high power mode
    OPA1CON2bits.PCH = 0b101; // DAC2_OUT
    OPA1CON0bits.EN = 1;
    //
    // Take a fraction of the same fixed voltage reference and feed it
    // through the 8-bit DAC3 buffered by OPA2 then to the external pin 
    // (OPA2OUT/RB1) to get VREF_B
    //
    DAC3CONbits.PSS = 0b10; // FVR Buffer 2
    DAC3CONbits.NSS = 0; // VSS
    DAC3CONbits.EN = 1;
    DAC3DATL = levelB;
    //
    OPA2CON0bits.UG = 1; // unity gain
    OPA2CON0bits.CPON = 1; // charge-pump is on, high power mode
    OPA2CON2bits.PCH = 0b110; // DAC3_OUT
    OPA2CON0bits.EN = 1;
    //
    __delay_ms(1);
}

void disable_VREF_AB()
{
    OPA1CON0bits.EN = 0;
    DAC2CONbits.EN = 0;
    OPA2CON0bits.EN = 0;
    DAC3CONbits.EN = 0;
}

uint8_t enable_external_trigger(uint16_t level, int8_t slope)
{
    // The external trigger signal is connected to C1IN0-/RA0.
    // It uses comparator 1 to sense the analog signal,
    // 10-bit DAC1 as the reference, and CLC1 as the SR latch.
    //
    // Input:
    // level sets the trigger voltage
    // slope=1 to trigger on exceeding level
    // slope=0 to trigger on going below level
    //
    // Returns:
    // 0 if successfully set up,
    // 1 if the comparator is already high.
    //
    // Use DAC2 for the reference level.
    DAC1CONbits.PSS = 0b10; // FVR Buffer 2
    DAC1CONbits.NSS = 0; // VSS
    DAC1CONbits.EN = 1;
    DAC1DATH = (uint8_t)(level >> 8);
    DAC1DATL = (uint8_t)level;
    __delay_ms(1);
    //
    // Our external signal goes into the inverting input of the comparator
    // and we use the DAC2 output as the reference on the noninverting input.
    CM1NCH = 0b000; // C1IN0- pin
    CM1PCH = 0b011; // DAC1_Output
    if (slope) {
        // Invert comparator output-polarity to get trigger on positive slope.
        CM1CON0bits.POL = 1;
    } else {
        CM1CON0bits.POL = 0;
    }
    CM1CON0bits.HYS = 0; // no hysteresis
    CM1CON0bits.SYNC = 0; // async output
    CM1CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC1OUT) {
        // Fail early because the comparator is already triggered.
        return 1;
    }
    // Use CLC3 to latch the comparator output to RB3.
    //
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b010; // To select CLC3 registers for the following settings.
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = 0x20; // data1 gets CMP1_OUT as input
    CLCnSEL1 = 0; // data2 gets CLCIN0PPS as input, but gets ignored in logic select
    CLCnSEL2 = 0; // data3 as for data2
    CLCnSEL3 = 0; // data4 as for data2
    // Logic select into gates
    CLCnGLS0 = 0b10; // data1 goes through true to gate 1 (S-R set)
    CLCnGLS1 = 0; // gate 2 gets logic 0 (S-R set)
    CLCnGLS2 = 0; // gate 3 gets logic 0 (S-R reset)
    CLCnGLS3 = 0; // gate 4 gets logic 0 (S-R reset)
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is S-R latch
    CLCnCONbits.MODE = 0b011;
    CLCnPOLbits.POL = 1; // Invert output; EVENT# is active low.
    // Now that the S-R latch is set up, enable it.
    CLCnCONbits.EN = 1;
    // Connect CLC1OUT to the RB3 pin.
    ODCONBbits.ODCB3 = 1;
    TRISBbits.TRISB3 = 0; // output
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB3PPS = 0x03; // CLC3OUT to EVENTn_PIN
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    return 0; // Success is presumed.
}

void disable_hardware_trigger()
{
    // Disables external trigger.
    TRISBbits.TRISB3 = 1; // input, if we are not wanting to drive it.
    CLCSELECT = 0b010; // To select CLC3 for the following setting.
    CLCnCONbits.EN = 0;
    CM1CON0bits.EN = 0;
    CM2CON0bits.EN = 0;
    DAC1CONbits.EN = 0;
    // Reconnect LATB to the RB3 pin,
    // so we may drive EVENTn via software.
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB3PPS = 0x00; // LATB3 to EVENTn_PIN
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    return;
}

uint8_t read_utility_pins()
{
    // Although the pins happen to be grouped together in the current design,
    // we'll set the bits one at a time, in case we ever reassign them.
    uint8_t bits = 0;
    if (PORTAbits.RA7) bits |= 0b1000;
    if (PORTAbits.RA6) bits |= 0b0100;
    if (PORTAbits.RA5) bits |= 0b0010;
    if (PORTAbits.RA4) bits |= 0b0001;
    return bits;
}

void write_utility_pins(char action, uint8_t bits)
{
    // Although the pins happen to be grouped together in the current design,
    // we'll set the bits one at a time, in case we ever reassign them.
    switch (action) {
    case 'A':
    case 'a':
        ANSELAbits.ANSELA7 = (bits & 0b1000) ? 1 : 0;
        ANSELAbits.ANSELA6 = (bits & 0b0100) ? 1 : 0;
        ANSELAbits.ANSELA5 = (bits & 0b0010) ? 1 : 0;
        ANSELAbits.ANSELA4 = bits & 0b0001;
        break;
    case 'T':
    case 't':
        TRISAbits.TRISA7 = (bits & 0b1000) ? 1 : 0;
        TRISAbits.TRISA6 = (bits & 0b0100) ? 1 : 0;
        TRISAbits.TRISA5 = (bits & 0b0010) ? 1 : 0;
        TRISAbits.TRISA4 = bits & 0b0001;
        break;
    case 'O':
    case 'o':
        ODCONAbits.ODCA7 = (bits & 0b1000) ? 1 : 0;
        ODCONAbits.ODCA6 = (bits & 0b0100) ? 1 : 0;
        ODCONAbits.ODCA5 = (bits & 0b0010) ? 1 : 0;
        ODCONAbits.ODCA4 = bits & 0b0001;
        break;
    case 'W':
    case 'w':
        WPUAbits.WPUA7 = (bits & 0b1000) ? 1 : 0;
        WPUAbits.WPUA6 = (bits & 0b0100) ? 1 : 0;
        WPUAbits.WPUA5 = (bits & 0b0010) ? 1 : 0;
        WPUAbits.WPUA4 = bits & 0b0001;
        break;
    case 'L':
    case 'l':
        LATAbits.LATA7 = (bits & 0b1000) ? 1 : 0;
        LATAbits.LATA6 = (bits & 0b0100) ? 1 : 0;
        LATAbits.LATA5 = (bits & 0b0010) ? 1 : 0;
        LATAbits.LATA4 = bits & 0b0001;
        break;
    default:
        /* do nothing */ ;
    }
    return;
}

// For incoming RS485 comms
#define NBUFA 80
char bufA[NBUFA];
// For outgoing RS485 comms
#define NBUFB 268
char bufB[NBUFB];
// For incoming DAQ-MCU responses
#define NBUFC 256
char bufC[NBUFC];
// For I2C and SPI communication with AFE boards
#define NBUF_I2C 32
uint8_t buf_I2C[NBUF_I2C];
#define NBUF_DIGITS 8
char buf_digits[NBUF_DIGITS];

int find_char(char* buf, int start, int end, char c)
// Returns the index of the character if found, -1 otherwise.
// start is the index of the first character to check.
// end is the index of the last character to check.
{
    for (int i = start; i <= end; i++) {
        if (buf[i] == '\0') return -1;
        if (buf[i] == c) return i;
    }
    return -1;
}

char* trim_RS485_command(char* buf, int nbytes)
// Returns a pointer to the command text string, within buf.
// The resulting string may be zero-length.
//
// A valid incoming command from the RS485 will be of the form
// "/cXXXXXXXX!"
// where the components are
//    / is the start character
//    ! is the end character
//    c is the MYID character, identifying the receiving node
//    XXXXXXX is the command text
//
// This format is described in the text:
// J.M. Hughes
// Real World Instrumentation
// O'Rielly 2010
// Chapter 11 Instrumentation Data I/O, Unique Protocols.
//
{
    // printf("DEBUG: buf=%s", buf);
    int start = find_char(buf, 0, nbytes-1, '/');
    if (start == -1) {
        // Did not find starting '/'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    int end = find_char(buf, start, nbytes-1, '!');
    if (end == -1) {
        // Did not find terminating '!'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, we have a valid incoming command.
    if (buf[start+1] != MYID) {
        // The incoming message is not for this node, so discard it.
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, the message is for us.
    buf[end] = '\0'; // Trim off the '!' character.
    // On return, omit the MYID character from the front.
    return &buf[start+2];
} // end trim_command()

void interpret_RS485_command(char* cmdStr)
// We intend that valid commands are answered quickly
// so that the supervisory PC can infer the absence of a node
// by the lack of a prompt response.
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "/0v %s#\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 't':
            // Software trigger to assert EVENTn line low.
            disable_hardware_trigger(); // Can enable only one at a time.
            assert_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0t Software trigger#\n");
            uart1_putstr(bufB);
            break;
        case 'z':
            // Release EVENTn line (from software trigger).
            release_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0z Release EVENTn line#\n");
            uart1_putstr(bufB);
            break;
        case 'Q':
            // Query the status signals.
            // READY/BUSYn is from the AVR.
            // EVENTn is a party line that anyone may pull low.
            nchar = snprintf(bufB, NBUFB, "/0Q %d %d#\n", EVENTn_PIN, PICO2_READY_PIN);
            uart1_putstr(bufB);
            break;
        case 'F':
            // Flush the RX2 buffer for incoming text from the DAQ MCU.
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0F Flushed RX2 buffer#\n");
            uart1_putstr(bufB);
            break;
        case 'R':
            // Restart the attached DAQ MCU.
            PICO2_RESETn_PIN = 0;
            __delay_ms(1);
            PICO2_RESETn_PIN = 1;
            // Wait until we are reasonably sure that the DAQ MCU has restarted
            // and then flush the incoming serial buffer.
            __delay_ms(350);
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0R DAQ_MCU restarted#\n");
            uart1_putstr(bufB);
            break;
        case 'a':
            // Report the ADC value for the analog signal on the selected pin.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume that it is an integer
                // that specifies which pin to sample.
                // 0x00  ANA0 (external trigger)
                // 0x01  OPA1OUT/ANA1 (V_REF_A)
                // 0x09  OPA2OUT/ANB1 (V_REF_B)
                uint8_t pin = (uint8_t)atoi(token_ptr);
                ADC_select(pin);
                uint16_t aValue = ADC_read();
                nchar = snprintf(bufB, NBUFB, "/0a %u#\n", aValue);
            } else {
                nchar = snprintf(bufB, NBUFB, "/0a error: no pin specified#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'e':
            // Enable external trigger, to pull EVENTn line low on the
            // external trigger signal.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume trigger level
                // and, maybe, slope flag.  Default slope flag is 1
                // (for trigger on a positive slope).
                int16_t level = atoi(token_ptr);
                if (level > 1023) level = 1023;
                if (level < 0) level = 0;
                int8_t slope = 1;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    slope = (int8_t) atoi(token_ptr);
                }
                uint8_t flag = enable_external_trigger((uint16_t)level, slope);
                if (flag) {
                    nchar = snprintf(bufB, NBUFB, "/0e error: comparator already triggered#\n");
                } else {
                    nchar = snprintf(bufB, NBUFB, "/0e %d %d#\n", level, slope);
                }
            } else {
                // There was no text to give a trigger level.
                nchar = snprintf(bufB, NBUFB, "/0e error: no level supplied#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'd':
            // Disable trigger and release the EVENTn line.
            disable_hardware_trigger();
            release_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0d Disable hardware trigger#\n");
            uart1_putstr(bufB);
            break;
            break;
        case 'u':
            // Interact with the 'utility' pins RA7, RA6, RA5 and RA4.
            //
            // The form of the command is
            // 'u' <action> <bits>
            // <action> is one of
            //   'P' read PORT bits (for input; <bits> is not required)
            //   'T' set TRIS bits (output-drive control 1=input 0=output)
            //   'O' set ODC bits (open-drain control)
            //   'W' set WPU bits (weak pull-up)
            //   'L' set LAT bits (latch for output)
            //   'A' set ANSEL bits (1=analog, 0=digital input buffer)
            // <bits> are mapped to relevant pins as
            //   bit     7    6    5    4    3    2    1    0
            //   pin     X    X    X    X  RA7  RA6  RA5  RA4
            //
            // Note that the relevant register for each pin will be written
            // with new data for each call with a write action.
            // If you want to leave some pins unchanged from their current state,
            // you will need to send appropriate data.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume is is a single character
                // that specifies the action, as described above.
                char action = (char) *token_ptr;
                if (action == 'P' || action == 'p') {
                    uint8_t bits = read_utility_pins();
                    nchar = snprintf(bufB, NBUFB, "/0u %c 0x%02x#\n", action, bits);
                } else {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        uint8_t bits = (uint8_t) atoi(token_ptr);
                        write_utility_pins(action, bits);
                        nchar = snprintf(bufB, NBUFB, "/0u %c 0x%02x#\n", action, bits);
                    } else {
                        nchar = snprintf(bufB, NBUFB, "/0u error: no bits specified#\n");
                    }
                }
            } else {
                nchar = snprintf(bufB, NBUFB, "/0u error: no action specified#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'b':
            // Read or write data bytes to the AFE board, via the I2C1 module.
            // The form of the command is
            // 'b' <action> <addr7bit> <nbytes> <byte0> <byte1> ...
            // <action> is one of 'w' or 'r'
            // <addr7bit> is the 7-bit address of the I2C slave device
            // <nbytes> is the number of bytes to read or write
            // <byte0> <byte1> ... are the bytes to write
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume is is a single character
                // that specifies the action, as described above.
                char action = (char) *token_ptr;
                if (action == 'R' || action == 'r') {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        uint8_t addr7bit = (uint8_t) atoi(token_ptr);
                        token_ptr = strtok(NULL, sep_tok);
                        if (token_ptr) {
                            uint8_t nbytes = (uint8_t) atoi(token_ptr);
                            if (nbytes > NBUF_I2C) nbytes = NBUF_I2C;
                            for (uint8_t j=0; j < NBUF_I2C; ++j) { buf_I2C[j] = 0; }
                            nbytes = i2c1_read(addr7bit, nbytes, buf_I2C, 30);
                            if (nbytes > 0) {
                                // Successfully read some bytes, so assemble response.
                                nchar = snprintf(bufB, NBUFB, "/0b r %d %d", addr7bit, nbytes);
                                for (uint8_t j=0; j < nbytes; ++j) {
                                    nchar = snprintf(buf_digits, NBUF_DIGITS, " %d", buf_I2C[j]);
                                    strncat(bufB, buf_digits, NBUF_DIGITS);
                                }
                                nchar = snprintf(buf_digits, NBUF_DIGITS, "#\n");
                                strncat(bufB, buf_digits, NBUF_DIGITS);
                            } else {
                                nchar = snprintf(bufB, NBUFB, "/0b r %d %d error: no bytes read#\n", addr7bit, nbytes);                                
                            }
                        } else {
                            nchar = snprintf(bufB, NBUFB, "/0b r %d error: nbytes not specified#\n", addr7bit);
                        }
                    } else {
                        nchar = snprintf(bufB, NBUFB, "/0b r error: address not specified#\n");
                    }
                } else if (action == 'W' || action == 'w') {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        uint8_t addr7bit = (uint8_t) atoi(token_ptr);
                        // [TODO]
                        nchar = snprintf(bufB, NBUFB, "/0b w %d error: not implemented#\n", addr7bit);
                    } else {
                        nchar = snprintf(bufB, NBUFB, "/0b error: address not specified#\n");
                    }
                } else {
                    nchar = snprintf(bufB, NBUFB, "/0b error: action is not read nor write#\n");
                }
            } else {
                nchar = snprintf(bufB, NBUFB, "/0b error: no read/write action specified#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'c':
            // Read or write data bytes to the AFE board, via SPI.
            nchar = snprintf(bufB, NBUFB, "/0c error: SPI exchange not implemented#\n");
            uart1_putstr(bufB);
            break;
        case 'w':
            // Enable V_REF_A output to feed MCP3301 chips 0-3 and
            // enable V_REF_B output to feed MCP3301 chips 4-7.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume levelA followed by levelB.
                // levelB is optional and, if omitted, get set the same as levelA.
                int16_t levelA = atoi(token_ptr);
                int16_t levelB = levelA;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    levelB = atoi(token_ptr);
                }
                if (levelA > 255) levelA = 255;
                if (levelA < 0) levelA = 0;
                if (levelB > 255) levelB = 255;
                if (levelB < 0) levelB = 0;
                set_VREF_AB((uint8_t)levelA, (uint8_t)levelB);
                nchar = snprintf(bufB, NBUFB, "/0w V_REF_A=%d V_REF_B=%d#\n", levelA, levelB);
            } else {
                // There was no text to indicate action.
                nchar = snprintf(bufB, NBUFB, "/0w error: missing levelA and levelB#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'X':
            // Pass through a command to the Pico2 DAQ MCU.
            if (PICO2_READY_PIN) {
                // DAQ MCU is ready and waiting for a command.
                // nchar = snprintf(bufB, NBUFB, "DEBUG About to send command:%s\n", &cmdStr[1]); uart1_putstr(bufB);
                uart2_putstr(&cmdStr[1]);
                uart2_putch('\n');
                // The DAQ MCU may start its response within 200us,
                // so start listening for that response immediately.
                uart2_getstr(bufC, NBUFC);
                nchar = snprintf(bufB, NBUFB, "/0X %s#\n", &bufC[0]);
            } else {
                // DAQ MCU is not ready for a command.
                nchar = snprintf(bufB, NBUFB, "/0X error: DAQ MCU busy#\n");
            }
            uart1_putstr(bufB);
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "/0%c error: Unknown command#\n", cmdStr[0]);
            uart1_putstr(bufB);
    }
} // end interpret_RS485_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200); // RS485 comms
    uart2_init(230400); // comms to Pico2 DAQ-MCU
    uart2_flush_rx(); // Discard any characters already arrived from Pico2 MCU
    FVR_init();
    ADC_init();
    set_VREF_AB(255, 255); // Full 4v096 reference, by default.
    i2c1_init();
    // Wait until we are reasonably sure that the Pico2 has restarted
    // and then flush the incoming serial buffer.
    __delay_ms(100);
    uart2_flush_rx();
    // We will operate this COMMS_MCU as a slave,
    // waiting for commands and only responding when spoken to.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // NL (Ctrl-J) signals end of incoming string.
        m = uart1_getstr(bufA, NBUFA);
        if (m > 0) {
            char* cmd = trim_RS485_command(bufA, NBUFA);
            // Note that the cmd string may be of zero length,
            // with the null character in the first place.
            // If that is the case, do nothing with it.
            if (*cmd) {
                interpret_RS485_command(cmd);
            }
        }
    }
    disable_hardware_trigger();
    release_event_pin();
    i2c1_close();
    disable_VREF_AB();
    ADC_close();
    FVR_close();
    uart2_flush_rx();
    uart2_close();
    uart1_flush_rx();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
