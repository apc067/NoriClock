//
// nori_clock.c
//
// Nori Clock - Clock with the Noritake GU140X32F-7003 vacuum fluorescent display
// (Revival of the PCMF hardware)
//
// Written by Peter Csaszar (Nixiana.com)
//
//=======================
// Hardware configuration
//=======================
//
// CPU clock:
//  - Source: Internal Oscillator Block (8 MHz)
//  - Postscaler: 1:1 -> Fosc = 8 MHz
//
// Peripheral modules:
//
//  - Timer 0:
//     - Use: System tick timer
//     - Clock source: Internal
//     - Clock: 2 MHz @ Fosc = 8 MHz (prescaler: none)
//     - Interrupt-on-Overflow: Yes
//     - Interrupt priority: Single (only)
//     - Mode: 16-bit
//
//  - Timer 1:
//     - Use: Delay time base
//     - Clock source: Internal
//     - Clock: 1 MHz @ Fosc = 8 MHz (prescaler: 1:2)
//     - Interrupt-on-Overflow: No
//     - Mode: 16-bit
//     - T1 Oscillator: Disabled
//     - Synchronization: No
//    Notes:
//     - The prescaler is adjusted such that the timer frequency is 1 MHz or lower. This allows
//       using the timer for de-facto microsecond-resolution delays up to 65 ms; for oscillator
//       frequencies not an even multiple of 4 MHz, delay_us() will err on the side of caution
//       (i.e., longer-than-specified delays).
//     - The delay will not be accurate for very short periods (below ~10 us) due to overhead,
//       esp. at lower Fosc values.
//
//  - Master Synchronous Serial Port (MSSP):
//     - Use: Serial interface to VFD
//     - Feature: SPI
//     - Clock: 500 kHz @ Fosc = 8 MHz (prescaler: 1/16, selected according to F_VFD_SPI_MAX)
//     - Mode: Master (selected clock prescaler ensures frequency not to exceed 1.5 MHz)
//     - Bus Mode: 11 (CKP/CPOL=1 [High idle clock polarity]; CKE/CPHA=1 [Output on idle->active])
//     - Input Sample Phase: Don't-care (set to SMP=0 [middle of data output period])
//
// Port bits:
//
//  - RB1 (I): User Button (sampled)
//
//  - RC2 (O): Debug signal for testpoint (repurposed from legacy PCMF ring signal input)
//  - RC3 (O/SPI-O): SPI SCK [Serial Clock] -> VFD: SCK input
//  - RC4 (I/SPI-I): SPI SDI [Serial Data In] (not used - see note below)
//  - RC5 (O/SPI-O): SPI SDO [Serial Data Out] -> VFD: SIN input
//  - RC6 (I): VFD Busy -> VFD: SBUSY output
//  - RC7 (O): VFD Reset -> VFD: RESET input
//
//  - RD2 (O): Red LED
//
// Note: The VFD does not send data, therefore it doesn't even have a MISO line, but the PIC's SPI
//       module configures RC4 as input. To prevent floating, it is grounded on the PCB.
//
// Interrupts (single priority mode):
//
//  - Timer 0 overflow

// Chip Configuration (overriding defaults)

#pragma config OSC = INTIO67    // Internal oscillator, RA6&7 available (Default: RC)
#pragma config PWRT = ON        // Power-on Timer enabled (Default: OFF)
#pragma config WDTPS = 2048     // WD Timer postscaler -> 8 s (Default: 32768 -> 131 s)
#pragma config PBADEN = OFF     // PORTB A/D channels disabled (Default: ON)
#pragma config LVP = OFF        // Single-supply ICSP disabled (Default: ON)

// Include files

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <p18f4520.h>
#include <pconfig.h>
#include <delays.h>
#include <spi.h>
#include <timers.h>

// Firmware version

#define FW_VERSION          "1.2.0"

// Customization

#define LONG_PRESS_MS       750                             // Long button press min duration [ms]
#define SPLASH_SCREEN_S     3                               // Splash screen duration [s]

// Fixed system parameters

#define F_OSC               8000000UL                       // MCU oscillator frequency
#define CLK_PER_INSTR       4                               // MCU clock cycles per instruction
#define F_ICYCLE            (F_OSC / CLK_PER_INSTR)         // MCU instruction cycle frequency
#define F_TICK              100                             // Timer 0 interrupt frequency (10 ms)
#define F_VFD_SPI_MAX       1500000UL                       // VFD SPI max clock frequency (well within spec)
#define VFD_RESET_PULSE_US  2000                            // VFD reset pulse duration [us]
#define VFD_RESET_RECOV_US  5                               // VFD post-reset recovery time [us]
#define TIMER0_RELOAD_CORR  486                             // Timer 0 reload value correction from DOE
#define TIMER0_PRESC_DIV    1                               // Timer 0 prescaler divider
#define TIMER0_PRESC_CFG    T0_PS_1_1                       // Timer 0 prescaler config constant

// Adjusted system parameters

#if F_ICYCLE > F_VFD_SPI_MAX                               // SPI clock prescaler
#define SPI_PRESC_CFG       SPI_FOSC_16
#else
#define SPI_PRESC_CFG       SPI_FOSC_4
#endif

#if F_ICYCLE > 4000000UL                                    // Timer 1 prescaler
#define TIMER1_PRESC_DIV    8
#define TIMER1_PRESC_CFG    T1_PS_1_8
#elif F_ICYCLE > 2000000UL
#define TIMER1_PRESC_DIV    4
#define TIMER1_PRESC_CFG    T1_PS_1_4
#elif F_ICYCLE > 1000000UL
#define TIMER1_PRESC_DIV    2
#define TIMER1_PRESC_CFG    T1_PS_1_2
#else
#define TIMER1_PRESC_DIV    1
#define TIMER1_PRESC_CFG    T1_PS_1_1
#endif

// GPIOs

#define PIN_BUTTON          PORTBbits.RB1                   // User button input (active low)
#define PIN_DEBUG           PORTCbits.RC2                   // Debug signal for testpoint
#define PIN_VFDSCK          PORTCbits.RC3                   // VFD SPI SCLK output (initially GPIO)
#define PIN_VFDSIN          PORTCbits.RC5                   // VFD SPI MOSI output (initially GPIO)
#define PIN_VFDBUSY         PORTCbits.RC6                   // VFD busy signal input
#define PIN_VFDRESET        PORTCbits.RC7                   // VFD reset output (active low)
#define PIN_LED             PORTDbits.RD2                   // Red LED output

// Timer-related calculations

#define F_TIMER0            (F_ICYCLE / TIMER0_PRESC_DIV)   // Timer 0 frequency
#define TIMER0_RELOAD_CALC  (0x10000 - (F_TIMER0 / F_TICK)) // Calculated Timer 0 reload value
#define TIMER0_RELOAD       (TIMER0_RELOAD_CALC + TIMER0_RELOAD_CORR)   // Corrected Timer 0 reload value
#define LONG_PRESS_TICK     (LONG_PRESS_MS / (1000 / F_TICK))           // Min long button press ticks

// Clock-related definitions

enum {                                                      // Clock & config dials
    CLOCK_SECONDS = 0,
    CLOCK_MINUTES,
    CLOCK_HOURS,
    CLOCK_DAY,
    CLOCK_MONTH,
    CLOCK_YEAR,
    CONFIG_HR_FORMAT,
    NUM_ALL_DIALS
};

#define NUM_CLOCK_DIALS     (CLOCK_YEAR + 1)
#define NUM_MONTHS          12
#define MONTH_FEBRUARY      1
#define YEAR_CENTURY        20

// Configuration-related definitions

enum {                                                      // Hour formats
    HR_FORMAT_24 = 0,
    HR_FORMAT_12,
    NUM_HR_FORMATS
};

// ISR-related definitions

enum {                                                      // ISR events
    ISR_NONE = 0,
    ISR_SECOND_PASSED,
    ISR_REGULAR_PRESS,
    ISR_SETTING_PRESS,
    ISR_ENTER_SETTING,
    ISR_CLOCK_SETTING,
    ISR_CONFIG_SETTING,
    ISR_EXIT_SETTING,
    ISR_SECONDS_FROZEN,
    ISR_SECONDS_UNFROZEN
};

enum {                                                      // Button actions
    BA_NONE = 0,
    BA_SHORT_PRESS,
    BA_LONG_PRESS
};

// VFD control sequences

#define VFD_CLS             "\x0c"                          // Clear screen
#define VFD_HOME            "\x0b"                          // Home cursor, without clear screen
#define VFD_MAG_ON          "\x1f(g@\x02\x02"               // Magnified font on
#define VFD_MAG_OFF         "\x1f(g@\x01\x01"               // Magnified font off
#define VFD_REV_ON          "\x1fr\x01"                     // Reverse font on
#define VFD_REV_OFF         "\x1fr\xff"                     // Reverse font off
#define VFD_SHORT_BLINK     "\x1f(a\x11\x02\x02\x02\x01"    // One short reverse-flash
#define VFD_LONG_BLINK      "\x1f(a\x11\x02\x02\x02\x03"    // Three short reverse-flashes
#define VFD_FLASH_SCR_ON    "\x1f(a\x11\x01\x23\x23\xff"    // Turn on screen-flashing
#define VFD_FLASH_SCR_OFF   "\x1f(a\x11\xff\x1\x1\xff"      // Turn off screen-flashing
#define VFD_BRIGHTNESS      "\x1fX"                         // Brightness control

// VFD geometry

#define VFD_ROWS            4                               // Number of VFD rows
#define VFD_COLS            20                              // Number of VFD columns

// Other definitions

#define MSG_BUFF_SIZE       64                              // VFD message buffer size
#define DISPLAY_STR_SIZE    20                              // Display field string size for each clock dial

#define IN_SETTING(f)       ((f) != -1)                     // Check if in setting mode

// Typedefs

typedef unsigned char       byte;
typedef char                sbyte;
typedef unsigned int        word;

// Global variables modified in ISR & read in main code

volatile byte isr_event = ISR_NONE;                         // ISR event indicator
volatile byte sec_freeze = 1;                               // Seconds are frozen if adjusted or upon boot
volatile sbyte selected_field = 0;                          // Display field under setting (-1: not in setting mode)
volatile byte clock[NUM_ALL_DIALS] = {
    0, 0, 0, 0, 0, 26,                                      // THE CLOCK: Seconds, minutes, hours, day, month, year
    HR_FORMAT_24                                            // Config: Hour format
};

// Other global variables

char msg_buff[MSG_BUFF_SIZE];                               // VFD message buffer
char display_strings[NUM_CLOCK_DIALS][DISPLAY_STR_SIZE];    // Display field strings for each clock dial

// Clock-related lookup tables (mostly read-only)

byte clock_limits[NUM_ALL_DIALS] = {                        // Limit for each dial
    60, 60, 24, 0, 12, 100,                                 // Actual clock dials (with caveat for month)
    NUM_HR_FORMATS                                          // Config dials
};
const rom byte field_to_dial_lut[NUM_ALL_DIALS] = {         // Display field to clock dial lookup
    2, 1, 0, 4, 3, 5,
    CONFIG_HR_FORMAT
};
const rom byte clock_digits[NUM_CLOCK_DIALS] = {            // Number of digits to display for each clock dial
    2, 2, 2, 2, 2, 4
};
const rom word clock_offsets[NUM_CLOCK_DIALS] = {           // Offset to add to each clock dial for display
    0, 0, 0, 1, 1, YEAR_CENTURY * 100
};
byte month_lengths[NUM_MONTHS] = {                          // Days in each month (with caveat for February)
    31, 0, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};
const rom byte month_offsets[2][NUM_MONTHS] = {             // Month offsets for day-of-week calculation
    { 0, 3, 3, 6, 1, 4, 6, 2, 5, 0, 3, 5 },
    { 0, 3, 4, 0, 2, 5, 0, 3, 6, 1, 4, 6 }
};
const rom char *dow_strings[] = {                           // Day-of-week strings
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

// Configuration strings

const rom char *hour_formats[] = { "24-hour", "12-hour" };  // Hour format strings


//==============================
// Main Code
//==============================

// Blocking max 65.5 ms delay for peripheral interactions (see header note for limitations)

void delay_us(word us)
{
    WriteTimer1(0);
    while (ReadTimer1() <= us)
        ;
}


// Initialize MCU

void init_pic(void)
{
    // GPIO setup (unused pins set as outputs with 0 value)
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    INTCON2bits.RBPU = 0;   // Port B pullups on
    TRISA = 0b00000000;
    TRISB = 0b00000010;     // Inputs: RB1 (User Button)
    TRISC = 0b01010000;     // Inputs: RC4 (SPI SDI), RC6 (VFD Busy)
    TRISD = 0b00000010;     // Inputs: RD1 (mfg. mistake: short to VFD Busy on PCB)
    TRISE = 0b00000000;

    // Internal oscillator setup (Fosc = 8 MHz)
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;

    // Interrupt setup
    RCONbits.IPEN = 0;      // Single priority mode
    INTCONbits.GIE = 1;     // Enable global interrupts
    INTCONbits.TMR0IE = 1;  // Enable Timer 0 interrupt

    // Peripheral setup (except SPI; done later)
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & TIMER0_PRESC_CFG);
    OpenTimer1(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_INT & TIMER1_PRESC_CFG &
               T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);
}


// Initialize VFD
//
// Note: Right after reset, the VFD's synchronous serial interface has already got two
// bits of random value in its hopper; the exact cause of this is unknown. Because of
// this, exactly 6 "dummy" bits need to be dialed into the interface in order to get it
// back in sync, before the PIC's SPI module can be activated. If all the dummy bits are
// zero, the resulting character will be disregarded by the VFD, so no Clear Screen
// action is necessary. (Noritake Tech Support remains baffled by this phenomenon.)

void init_vfd(void)
{
    byte i;

    // Reset VFD
    PIN_VFDSIN = 0;
    PIN_VFDSCK = 1;
    delay_us(2000);
    PIN_VFDRESET = 1;       // Release VFD from reset

    // Recover from reset
    while (PIN_VFDBUSY)
        ;
    delay_us(5);

    // Clock in 6 "dummy" bits of 0
    for (i = 0; i < 6; i++) {
        Delay1TCY();
        PIN_VFDSCK = 0;
        Delay1TCY();
        PIN_VFDSCK = 1;
    }

    // Now ready to bring up the SPI module
    OpenSPI(SPI_PRESC_CFG, MODE_11, SMPMID);

}


// Write a character to VFD
//
// unsigned char c: Character to be written (can also be 0)

void putc_vfd(unsigned char c)
{
    byte s, d;
    unsigned char rev_c = 0;

    // Reverse the order of bits (PIC-VFD SPI incompatibility)
    for (s = 0x80, d = 0x01; s; s >>= 1, d <<= 1) {
        if (c & s) {
            rev_c |= d;
        }
    }

    // Wait while the VFD is busy
    while (PIN_VFDBUSY)
        ;

    // Send character to the VDF
    WriteSPI(rev_c);
}


// Write a string to VFD
//
// char *p: String to be written (null-terminated; actual zeros represented by 0xff)

void puts_vfd(char *p)
{
    unsigned char c;

    while ((c = *p++)) {
        putc_vfd(c == 0xff ? '\0' : c);
    }
}


// Show the splash screen

void splash_screen(void)
{
    byte ctdn = SPLASH_SCREEN_S;

    puts_vfd(strcpypgm2ram(msg_buff, VFD_CLS VFD_MAG_ON "Nori Clock" VFD_MAG_OFF));
    puts_vfd(strcpypgm2ram(msg_buff, "   by Nixiana.com   "));
    puts_vfd(strcpypgm2ram(msg_buff, "  FW Version " FW_VERSION));
    while (ctdn--) {
        isr_event = ISR_NONE;
        while (!isr_event)
            ;
    }
    puts_vfd(strcpypgm2ram(msg_buff, VFD_CLS));
}


// Return the number of days in February, based on current year (lucky 2000!)

byte month_length_feb(void)
{
    return clock[CLOCK_YEAR] % 4 ? 28 : 29;
}


// Print the clock to the VFD

void print_clock(void)
{
    byte field, dial;
    byte col;
    byte cent_year, dow_jan1, dow;
    byte hour;
    // Arguments for "%s" (must be in the data memory)
    char rev_str[4], unrev_str[4];
    char dow_str[4];
    char time_pre_str[4], time_post_str[8];

    // Handle the hour format
    if (clock[CONFIG_HR_FORMAT] == HR_FORMAT_24) {
        hour = clock[CLOCK_HOURS];
        strcpypgm2ram(time_pre_str, "  ");
        strcpypgm2ram(time_post_str, "\n  ");
    } else {
        hour = clock[CLOCK_HOURS] % 12;
        if (hour == 0) {
            hour = 12;
        }
        strcpypgm2ram(time_pre_str, " ");
        strcpypgm2ram(time_post_str, clock[CLOCK_HOURS] < 12 ? " A\n\bM " : " P\n\bM ");
    }

    // Calculate string for each display field
    for (field = 0; field < NUM_CLOCK_DIALS; field++) {
        dial = field_to_dial_lut[field];
        strcpypgm2ram(rev_str, field == selected_field ? VFD_REV_ON : "");
        strcpypgm2ram(unrev_str, field == selected_field ? VFD_REV_OFF : "");
        sprintf(display_strings[field],
                dial == CLOCK_HOURS && clock[CONFIG_HR_FORMAT] == HR_FORMAT_12 ? "%s%*u%s" : "%s%0*u%s",
                rev_str,
                clock_digits[dial],
                dial == CLOCK_HOURS ? hour : clock[dial] + clock_offsets[dial],
                unrev_str);
    }

    // Print time
    sprintf(msg_buff, VFD_HOME "%s" VFD_MAG_ON "%s:%s:%s" VFD_MAG_OFF "%s",
            time_pre_str, display_strings[0], display_strings[1], display_strings[2], time_post_str);
    puts_vfd(msg_buff);

    // Print separator row
    for (col = 0; col < VFD_COLS; col++) {
        putc_vfd(col == clock[CLOCK_SECONDS] / (60 / VFD_COLS) ? '|' : '-');
    }

    // Calculate day of week (Gauss's algorithm)
    cent_year = clock[CLOCK_YEAR] - 1;
    dow_jan1 = (1 + 5 * (cent_year % 4) + 3 * cent_year + 5 * (YEAR_CENTURY % 4)) % 7;
    dow = (dow_jan1 + month_offsets[!(clock[CLOCK_YEAR] % 4)][clock[CLOCK_MONTH]] + clock[CLOCK_DAY]) % 7;
    strcpypgm2ram(dow_str, dow_strings[dow]);

    // Print date
    sprintf(msg_buff, "  %s-%s-%s (%s)",
            display_strings[3], display_strings[4], display_strings[5], dow_str);
    puts_vfd(msg_buff);
}


// Update the display

void update_display(void)
{
    char cfg_str[12];

    if (selected_field <= CLOCK_YEAR) {         // Not in setting mode or setting the clock
        print_clock();
    } else if (selected_field == CONFIG_HR_FORMAT) {
        strcpypgm2ram(cfg_str, hour_formats[clock[CONFIG_HR_FORMAT]]);
        sprintf(msg_buff, VFD_HOME "Hour Format:\r\n\n" VFD_MAG_ON "%s" VFD_MAG_OFF, cfg_str);
        puts_vfd(msg_buff);
    }
}


// Main

void main(void)
{
    byte vfd_dim = 0;

    // Startup
    init_pic();
    init_vfd();
    splash_screen();

    // Main loop
    while (1) {
        // Wait for an event
        PIN_DEBUG = 1;            // Indicate idle state with a high level
        while (!isr_event)
            ;
        PIN_DEBUG = 0;

        // Pet the poodle
        ClrWdt();

        // Give visual feedback for the event
        switch (isr_event) {
        case ISR_ENTER_SETTING:
            strcpypgm2ram(msg_buff, VFD_SHORT_BLINK);
            break;
        case ISR_CONFIG_SETTING:
            strcpypgm2ram(msg_buff, VFD_CLS);
            break;
        case ISR_EXIT_SETTING:
            strcpypgm2ram(msg_buff, VFD_CLS VFD_LONG_BLINK);
            break;
        case ISR_SECONDS_FROZEN:
            strcpypgm2ram(msg_buff, VFD_FLASH_SCR_ON);
            break;
        case ISR_SECONDS_UNFROZEN:
            strcpypgm2ram(msg_buff, VFD_FLASH_SCR_OFF);
            break;
        case ISR_REGULAR_PRESS:
            vfd_dim = !vfd_dim;
            sprintf(msg_buff, VFD_BRIGHTNESS "%c", vfd_dim ? 4 : 8);
            break;
        case ISR_SECOND_PASSED:
        case ISR_CLOCK_SETTING:
        case ISR_SETTING_PRESS:
            *msg_buff = '\0';
            break;

        }
        puts_vfd(msg_buff);
        isr_event = ISR_NONE;

        // Update the display
        update_display();
    }
}


//==============================
// Interrupt Handling
//==============================

// Increment the clock dials(s)
//
// sbyte set_dial: Clock dial being adjusted in setting mode (-1: regular clock increment)

void increment_clock(sbyte set_dial)
{
    byte dial, start_dial, end_dial;

    // Setup
    month_lengths[MONTH_FEBRUARY] = month_length_feb();
    clock_limits[CLOCK_DAY] = month_lengths[clock[CLOCK_MONTH]];

    if (IN_SETTING(set_dial)) {
        start_dial = set_dial;
        end_dial = set_dial;
    } else {
        start_dial = CLOCK_SECONDS;
        end_dial = CLOCK_YEAR;
    }

    // Turn the dial(s)
    for (dial = start_dial; dial <= end_dial; dial++) {
        clock[dial]++;
        if (clock[dial] >= clock_limits[dial]) {    // >=: In case day is beyond limit due to changed month/year
            clock[dial] = 0;
        } else {
            break;
        }
    }

    // Freeze seconds if adjusted in setting mode
    if (set_dial == CLOCK_SECONDS) {
        sec_freeze = 1;
    }

    // Recalculate February day limit if year is adjusted in setting mode
    // (Note: month adjustment will not affect day value until exiting setting mode)
    if (set_dial == CLOCK_YEAR) {
        month_lengths[MONTH_FEBRUARY] = month_length_feb();
    }
}


// Timer 0 ISR

#pragma interrupt timer0_isr
void timer0_isr(void)
{
    static byte second_tick_count = 0;
    static byte button_tick_count = 0;
    static byte last_button_state = 0;
    static byte suppress_short_press = 0;
    byte current_button_state;
    byte button_action;

    // Start the ISR
    WriteTimer0(TIMER0_RELOAD);
    PIN_DEBUG = ~PIN_DEBUG;                  // "In ISR" inverse pulse
    INTCONbits.TMR0IF = 0;
    button_action = BA_NONE;

    // Advance the time
    second_tick_count++;
    if (second_tick_count == F_TICK) {
        second_tick_count = 0;
        isr_event = ISR_SECOND_PASSED;
        if (!sec_freeze) {
            increment_clock(-1);
        }
    }

    // Determine the button action
    current_button_state = !PIN_BUTTON;
    if (current_button_state) {                     // Currently pressed
        if (!last_button_state) {                   // Just pressed
            if (!IN_SETTING(selected_field) && sec_freeze) {    // Unfreeze seconds immediately
                suppress_short_press = 1;           // Suppress short press action
                WriteTimer0(TIMER0_RELOAD);
                second_tick_count = 0;
                sec_freeze = 0;
                isr_event = ISR_SECONDS_UNFROZEN;
            }
        }
        if (button_tick_count < 255) {
            button_tick_count++;
        }
        if (button_tick_count == LONG_PRESS_TICK) {
            button_action = BA_LONG_PRESS;
        }
    } else {                                        // Currently not pressed
        if (last_button_state) {                    // Just released
            if (button_tick_count < LONG_PRESS_TICK) {
                button_action = BA_SHORT_PRESS;
            }
            button_tick_count = 0;
         }
    }
    last_button_state = current_button_state;

    // Interpret the button action
    if (button_action == BA_SHORT_PRESS) {
        if (!IN_SETTING(selected_field)) {
            if (!suppress_short_press) {
                isr_event = ISR_REGULAR_PRESS;
            }
            suppress_short_press = 0;
        } else {
            increment_clock(field_to_dial_lut[selected_field]);     // Adjust selected field's clock dial
            isr_event = ISR_SETTING_PRESS;
        }
    } else if (button_action == BA_LONG_PRESS) {
        selected_field++;                           // Advance to next field
        if (selected_field == 0) {
            isr_event = ISR_ENTER_SETTING;
        } else if (selected_field < NUM_CLOCK_DIALS) {
            isr_event = ISR_CLOCK_SETTING;
        } else if (selected_field < NUM_ALL_DIALS) {
            isr_event = ISR_CONFIG_SETTING;
        } else {
            selected_field = -1;                    // Exit setting mode
            if (clock[CLOCK_DAY] >= month_lengths[clock[CLOCK_MONTH]]) {
                clock[CLOCK_DAY] = month_lengths[clock[CLOCK_MONTH]] - 1;
            }
            if (sec_freeze) {
                isr_event = ISR_SECONDS_FROZEN;
            } else {
                isr_event = ISR_EXIT_SETTING;
            }
        }
    }

    // Wrap up the ISR
    PIN_DEBUG = ~PIN_DEBUG;
}


// Interrupt Vector

#pragma code high_vector = 0x08
void timer0_isr_vector(void)
{
    _asm GOTO timer0_isr _endasm
}
#pragma code
