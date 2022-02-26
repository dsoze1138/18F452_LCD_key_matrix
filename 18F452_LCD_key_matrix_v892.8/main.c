/*
 * File:   main.c
 * Author: dan1138
 * Target: PIC18F452
 * Compiler: XC8 v1.45
 * IDE: MPLAB v8.92
 *
 * Created on February 25, 2022, 9:08 PM
 *
 *                                 PIC18F452
 *                          +---------:_:---------+
 *              VPP MCLR -> :  1 VPP       PGD 40 : <> RB7 LCD_RS/PGD
 *          SENSE_A0 RA0 <> :  2           PGC 39 : <> RB6 LCD_RW/PGC 
 *          SENSE_A1 RA1 <> :  3           PGM 38 : <> RB5 LCD_EN
 *          SENSE_A2 RA2 <> :  4               37 : <> RB4 LCD_D4
 *          SENSE_A3 RA3 <> :  5               36 : <> RB3 LCD_D5
 *          SENSE_A4 RA4 <> :  6 T0CKI         35 : <> RB2 LCD_D6
 *          SENSE_A5 RA5 <> :  7               34 : <> RB1 LCD_D7
 *          SENSE_E0 RE0 <> :  8          INT0 33 : <> RB0 
 *         SELECT_E1 RE1 <> :  9               32 : <- VDD 5v0
 *         SELECT_E2 RE2 <> : 10               31 : <- VSS GND
 *               5v0 VDD -> : 11               30 : <> RD7
 *               GND VSS -> : 12               29 : <> RD6
 *    20MHZ_CRYSTAL OSC1 -> : 13 OSC1          28 : <> RD5
 *    20MHZ_CRYSTAL OSC2 <- : 14 OSC2          27 : <> RD4
 *                   RC0 <> : 15 T1CKI         26 : <> RC7
 *                   RC1 <> : 16               25 : <> RC6
 *         SELECT_C2 RC2 <> : 17               24 : <> RC5
 *                   RC3 <> : 18               23 : <> RC4
 *                   RD0 <> : 19               22 : <> RD3
 *                   RD1 <> : 20               21 : <> RD2
 *                          +---------------------+
 *                                  DIP-40
 *
 * Description:
 * 
 *  Show 4-bit parallel interface for an HD44780 based LCD module
 *  and scan a matrix of 17 buttons.
 *
 * See: https://www.microchip.com/forums/FindPost/1200456
 *
 */

#pragma config OSC = HS, OSCS = OFF
#pragma config PWRT = OFF, BOR = OFF, BORV = 20
#pragma config WDT = OFF, WDTPS = 128, CCP2MUX = OFF
#pragma config STVR = OFF, LVP = OFF
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF, EBTRB = OFF
/*
 * Define the system oscillator frequency this code will configure
 */
#define _XTAL_FREQ 20000000UL
#define FCYC (_XTAL_FREQ/4UL)
/*
 * Include definitions for target specific registers
 */
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
/*
 * Initialize this PIC
 */
void PIC_Init(void)
{
    INTCON = 0;                 /* disable all interrupts */
    INTCON3 = 0;
    PIE1 = 0;
    PIE2 = 0;
    
    RCONbits.IPEN = 0;          /* use legacy interrupt mode */
    
    ADCON1 = 0x07;              /* make GIIO pins digital I/O */
}
/* Define the LCD port pins */
#define LCD_DATA_BITS_MASK  0x1E
#define LCD_PORT_OUT        LATB
#define LCD_PORT_DIR        TRISB

#define LCD_RS_PIN          LATBbits.LATB7
#define LCD_RW_PIN          LATBbits.LATB6
#define LCD_EN_PIN          LATBbits.LATB5
#define LCD_D4_PIN          LATBbits.LATB4
#define LCD_D5_PIN          LATBbits.LATB3
#define LCD_D6_PIN          LATBbits.LATB2
#define LCD_D7_PIN          LATBbits.LATB1

#define LCD_RS_DIR          TRISBbits.TRISB7
#define LCD_RW_DIR          TRISBbits.TRISB6
#define LCD_EN_DIR          TRISBbits.TRISB5
#define LCD_D4_DIR          TRISBbits.TRISB4
#define LCD_D5_DIR          TRISBbits.TRISB3
#define LCD_D6_DIR          TRISBbits.TRISB2
#define LCD_D7_DIR          TRISBbits.TRISB1

/* Clear display command */
#define CLEAR_DISPLAY       0b00000001

/* Return home command */
#define RETURN_HOME         0b00000010

/* Display ON/OFF Control defines */
#define DON                 0b00001111  /* Display on      */
#define DOFF                0b00001011  /* Display off     */
#define CURSOR_ON           0b00001111  /* Cursor on       */
#define CURSOR_OFF          0b00001101  /* Cursor off      */
#define BLINK_ON            0b00001111  /* Cursor Blink    */
#define BLINK_OFF           0b00001110  /* Cursor No Blink */

/* Cursor or Display Shift defines */
#define SHIFT_CUR_LEFT      0b00010011  /* Cursor shifts to the left   */
#define SHIFT_CUR_RIGHT     0b00010111  /* Cursor shifts to the right  */
#define SHIFT_DISP_LEFT     0b00011011  /* Display shifts to the left  */
#define SHIFT_DISP_RIGHT    0b00011111  /* Display shifts to the right */

/* Function Set defines */
#define FOUR_BIT            0b00101111  /* 4-bit Interface               */
#define EIGHT_BIT           0b00111111  /* 8-bit Interface               */
#define LINE_5X7            0b00110011  /* 5x7 characters, single line   */
#define LINE_5X10           0b00110111  /* 5x10 characters               */
#define LINES_5X7           0b00111011  /* 5x7 characters, multiple line */

/* Start address of each line */
#define LINE_ONE    0x00
#define LINE_TWO    0x40

static void LCD_EN_Pulse(void)
{
    LCD_EN_PIN = 1;
    __delay_us(4);
    LCD_EN_PIN = 0;
    __delay_us(4);
}

static void LCD_DelayPOR(void)
{
    __delay_ms(15);
}

static void LCD_Delay(void)
{
    __delay_ms(5);
}

static void LCD_PutByte(unsigned char LCD_Data)
{
    LCD_PORT_DIR &= ~LCD_DATA_BITS_MASK; /* make LCD data bits outputs */
    
    /* send first(high) nibble */
    LCD_PORT_OUT &= ~LCD_DATA_BITS_MASK;
    if(LCD_Data & 0x10) LCD_D4_PIN = 1;
    if(LCD_Data & 0x20) LCD_D5_PIN = 1;
    if(LCD_Data & 0x40) LCD_D6_PIN = 1;
    if(LCD_Data & 0x80) LCD_D7_PIN = 1;
    LCD_EN_Pulse();
    
    /* send second(low) nibble */
    LCD_PORT_OUT &= ~LCD_DATA_BITS_MASK;
    if(LCD_Data & 0x01) LCD_D4_PIN = 1;
    if(LCD_Data & 0x02) LCD_D5_PIN = 1;
    if(LCD_Data & 0x04) LCD_D6_PIN = 1;
    if(LCD_Data & 0x08) LCD_D7_PIN = 1;
    LCD_EN_Pulse();

    LCD_PORT_DIR |= LCD_DATA_BITS_MASK; /* make LCD data bits inputs */
}

void LCD_SetPosition(unsigned char data)
{
    LCD_RS_PIN = 0;
    LCD_PutByte((unsigned char)(data | 0x80));
    __delay_us(40);
}

void LCD_WriteCmd(unsigned char data)
{
    LCD_RS_PIN = 0;
    LCD_PutByte(data);
    __delay_ms(4);
}

void LCD_WriteData(unsigned char data)
{
    LCD_RS_PIN = 1;
    LCD_PutByte(data);
    __delay_us(40);
}

void LCD_Init(void) 
{
    unsigned char LCD_Data;
    
    LCD_PORT_DIR &= ~LCD_DATA_BITS_MASK;    /* make LCD data bits outputs */
    LCD_RW_PIN = 0;
    LCD_EN_DIR = 0;                         /* make LCD Enable strobe an output */
    LCD_RS_DIR = 0;                         /* make LCD Register select an output */
    LCD_RW_PIN = 0;
    LCD_EN_PIN = 0;                         /* set LCD Enable strobe to not active */
    LCD_RS_PIN = 0;                         /* set LCD Register select to command group */
    LCD_PORT_OUT &= ~LCD_DATA_BITS_MASK;    /* set LCD data bits to zero */
    LCD_DelayPOR();                         /* wait for LCD power on to complete */

    /* Force LCD to 8-bit mode */
    LCD_PORT_OUT &= ~LCD_DATA_BITS_MASK;    /* set LCD data bits to zero */
    LCD_D4_PIN = 1;
    LCD_D5_PIN = 1;
    LCD_EN_Pulse();
    LCD_Delay();
    LCD_EN_Pulse();
    LCD_Delay();
    LCD_EN_Pulse();
    LCD_Delay();
    
    /* Set LCD to 4-bit mode */
    LCD_PORT_OUT &= ~LCD_DATA_BITS_MASK;    /* set LCD data bits to zero */
    LCD_D5_PIN = 1;
    LCD_EN_Pulse();
    LCD_Delay();

    /* Initialize LCD mode */
    LCD_WriteCmd(FOUR_BIT & LINES_5X7);

    /* Turn on display, Setup cursor and blinking */
    LCD_WriteCmd(DOFF & CURSOR_OFF & BLINK_OFF);
    LCD_WriteCmd(DON & CURSOR_OFF & BLINK_OFF);
    LCD_WriteCmd(CLEAR_DISPLAY);
    LCD_WriteCmd(SHIFT_CUR_LEFT);

    /* Set first position on line one, left most character */
    LCD_SetPosition(LINE_ONE);
}
void LCD_puthex(uint8_t Data)
{
    uint8_t Nibble;
    
    Nibble = (Data >> 4)+'0';
    if (Nibble > '9') Nibble += 'A'-('9'+1);
    LCD_WriteData(Nibble);
    Nibble = (Data & 0x0F)+'0';
    if (Nibble > '9') Nibble += 'A'-('9'+1);
    LCD_WriteData(Nibble);
}
void LCD_puts(void *Str)
{
    uint8_t * pStr = (uint8_t *)Str;
    while(*pStr)
    {
        LCD_WriteData(*pStr++);
    }
}
/* Define the key matrix */
typedef union { 
    struct { 
        uint8_t group1;
        uint8_t group2;
        uint8_t group3;
    };
    struct { 
        unsigned E1A0 : 1;
        unsigned E1A1 : 1;
        unsigned E1A2 : 1;
        unsigned E1A3 : 1;
        unsigned E1A4 : 1;
        unsigned E1A5 : 1;
        unsigned E1E0 : 1;
        unsigned      : 1;
        unsigned E2A0 : 1;
        unsigned E2A1 : 1;
        unsigned E2A2 : 1;
        unsigned E2A3 : 1;
        unsigned E2A4 : 1;
        unsigned E2A5 : 1;
        unsigned E2E0 : 1;
        unsigned      : 1;
        unsigned      : 1;
        unsigned      : 1;
        unsigned      : 1;
        unsigned C2A3 : 1;
        unsigned C2A4 : 1;
        unsigned C2A5 : 1;
        unsigned      : 1;
        unsigned      : 1;
    };
}KeyMatrix_t;

volatile struct {
    KeyMatrix_t State;
    KeyMatrix_t Change;
} gKeys;
/*
 * Initialize key matrix
 */
void KeyMatrix_Init(void)
{
    gKeys.State.group1 = 0;
    gKeys.State.group2 = 0;
    gKeys.State.group3 = 0;
    gKeys.Change.group1 = 0;
    gKeys.Change.group2 = 0;
    gKeys.Change.group3 = 0;
    
    LATCbits.LATC2 = 0;
    LATEbits.LATE1 = 0;
    LATEbits.LATE2 = 0;
    TRISA |= 0b00111111;
    TRISEbits.TRISE0 = 1;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    TRISCbits.TRISC2 = 0;
}
/*
 * Scan key matrix for button changes
 * 
 * Returns non-zero if any button changed
 */
#define DEBOUNCE_COUNT_RESET (20)
static void SampleDelayBetweenGroups(void)
{
    __delay_us(20);
}
uint8_t KeyMatrix_Scan(void)
{
    static uint8_t DebounceCount = 0;
    static KeyMatrix_t Stable;
    KeyMatrix_t Sample;

    SampleDelayBetweenGroups();
    LATEbits.LATE1 = 1;
    Sample.group1 = 0;
    if (PORTAbits.RA0) Sample.E1A0 = 1;
    if (PORTAbits.RA1) Sample.E1A1 = 1;
    if (PORTAbits.RA2) Sample.E1A2 = 1;
    if (PORTAbits.RA3) Sample.E1A3 = 1;
    if (PORTAbits.RA4) Sample.E1A4 = 1;
    if (PORTAbits.RA5) Sample.E1A5 = 1;
    if (PORTEbits.RE0) Sample.E1E0 = 1;
    LATEbits.LATE1 = 0;
    SampleDelayBetweenGroups();
    LATEbits.LATE2 = 1;
    Sample.group2 = 0;
    if (PORTAbits.RA0) Sample.E2A0 = 1;
    if (PORTAbits.RA1) Sample.E2A1 = 1;
    if (PORTAbits.RA2) Sample.E2A2 = 1;
    if (PORTAbits.RA3) Sample.E2A3 = 1;
    if (PORTAbits.RA4) Sample.E2A4 = 1;
    if (PORTAbits.RA5) Sample.E2A5 = 1;
    if (PORTEbits.RE0) Sample.E2E0 = 1;
    LATEbits.LATE2 = 0;
    SampleDelayBetweenGroups();
    LATCbits.LATC2 = 1;
    Sample.group3 = 0;
    if (PORTAbits.RA3) Sample.C2A3 = 1;
    if (PORTAbits.RA4) Sample.C2A4 = 1;
    if (PORTAbits.RA5) Sample.C2A5 = 1;
    LATCbits.LATC2 = 0;
    if (Stable.group1 != Sample.group1)
    {
        Stable.group1 = Sample.group1;
        DebounceCount = DEBOUNCE_COUNT_RESET;
    }
    else if (Stable.group2 != Sample.group2)
    {
        Stable.group2 = Sample.group2;
        DebounceCount = DEBOUNCE_COUNT_RESET;
    }
    else if (Stable.group3 != Sample.group3)
    {
        Stable.group3 = Sample.group3;
        DebounceCount = DEBOUNCE_COUNT_RESET;
    }
    else
    {
        if (DebounceCount)
        {
            DebounceCount--;
            if (0 == DebounceCount)
            {
                gKeys.Change.group1 = gKeys.State.group1 ^ Stable.group1;
                gKeys.Change.group2 = gKeys.State.group2 ^ Stable.group2;
                gKeys.Change.group3 = gKeys.State.group3 ^ Stable.group3;
                if ((gKeys.Change.group1) || (gKeys.Change.group2) || (gKeys.Change.group3))
                {
                    gKeys.State.group1 = Stable.group1;
                    gKeys.State.group2 = Stable.group2;
                    gKeys.State.group3 = Stable.group3;
                    return 1;
                }
            }
        }
    }
    
    return 0;
}
/*
 * Main application
 */
void main(void)
{
    /*
     * Initialize application
     */
    PIC_Init();
    LCD_Init();
    KeyMatrix_Init();
    /*
     * Show startup on 16x2 LCD module
     */
    LCD_SetPosition(LINE_ONE);
    LCD_puts("  18F452 Start  ");
    __delay_ms(1000);
    LCD_SetPosition(LINE_ONE);
    LCD_puts("At ");LCD_puts(__TIME__);LCD_puts("     ");
    LCD_SetPosition(LINE_TWO);
    LCD_puts("On ");LCD_puts(__DATE__);LCD_puts("   ");
    /*
     * Application loop
     */
    for(;;)
    {
        __delay_ms(1);
        if(KeyMatrix_Scan())
        {
            LCD_SetPosition(LINE_ONE);
            LCD_puts("Matrix: "); LCD_puthex(gKeys.State.group1); LCD_puthex(gKeys.State.group2); LCD_puthex(gKeys.State.group3);
            LCD_SetPosition(LINE_TWO);
            LCD_puts("Change: "); LCD_puthex(gKeys.Change.group1); LCD_puthex(gKeys.Change.group2); LCD_puthex(gKeys.Change.group3);
        }
    }
}
