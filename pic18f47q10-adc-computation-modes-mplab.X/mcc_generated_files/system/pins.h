/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.1.0
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA0 aliases
#define POT_DATA_TRIS                 TRISAbits.TRISA0
#define POT_DATA_LAT                  LATAbits.LATA0
#define POT_DATA_PORT                 PORTAbits.RA0
#define POT_DATA_WPU                  WPUAbits.WPUA0
#define POT_DATA_OD                   ODCONAbits.ODCA0
#define POT_DATA_ANS                  ANSELAbits.ANSELA0
#define POT_DATA_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define POT_DATA_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define POT_DATA_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define POT_DATA_GetValue()           PORTAbits.RA0
#define POT_DATA_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define POT_DATA_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define POT_DATA_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define POT_DATA_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define POT_DATA_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define POT_DATA_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define POT_DATA_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define POT_DATA_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set RA1 aliases
#define FG_DATA_TRIS                 TRISAbits.TRISA1
#define FG_DATA_LAT                  LATAbits.LATA1
#define FG_DATA_PORT                 PORTAbits.RA1
#define FG_DATA_WPU                  WPUAbits.WPUA1
#define FG_DATA_OD                   ODCONAbits.ODCA1
#define FG_DATA_ANS                  ANSELAbits.ANSELA1
#define FG_DATA_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define FG_DATA_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define FG_DATA_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define FG_DATA_GetValue()           PORTAbits.RA1
#define FG_DATA_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define FG_DATA_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define FG_DATA_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define FG_DATA_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define FG_DATA_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define FG_DATA_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define FG_DATA_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define FG_DATA_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RA4 aliases
#define LED_D2_TRIS                 TRISAbits.TRISA4
#define LED_D2_LAT                  LATAbits.LATA4
#define LED_D2_PORT                 PORTAbits.RA4
#define LED_D2_WPU                  WPUAbits.WPUA4
#define LED_D2_OD                   ODCONAbits.ODCA4
#define LED_D2_ANS                  ANSELAbits.ANSELA4
#define LED_D2_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define LED_D2_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define LED_D2_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define LED_D2_GetValue()           PORTAbits.RA4
#define LED_D2_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define LED_D2_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define LED_D2_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define LED_D2_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define LED_D2_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define LED_D2_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define LED_D2_SetAnalogMode()      do { ANSELAbits.ANSELA4 = 1; } while(0)
#define LED_D2_SetDigitalMode()     do { ANSELAbits.ANSELA4 = 0; } while(0)

// get/set RA5 aliases
#define LED_D3_TRIS                 TRISAbits.TRISA5
#define LED_D3_LAT                  LATAbits.LATA5
#define LED_D3_PORT                 PORTAbits.RA5
#define LED_D3_WPU                  WPUAbits.WPUA5
#define LED_D3_OD                   ODCONAbits.ODCA5
#define LED_D3_ANS                  ANSELAbits.ANSELA5
#define LED_D3_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LED_D3_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LED_D3_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LED_D3_GetValue()           PORTAbits.RA5
#define LED_D3_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LED_D3_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LED_D3_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define LED_D3_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define LED_D3_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define LED_D3_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define LED_D3_SetAnalogMode()      do { ANSELAbits.ANSELA5 = 1; } while(0)
#define LED_D3_SetDigitalMode()     do { ANSELAbits.ANSELA5 = 0; } while(0)

// get/set RC5 aliases
#define SW2_TRIS                 TRISCbits.TRISC5
#define SW2_LAT                  LATCbits.LATC5
#define SW2_PORT                 PORTCbits.RC5
#define SW2_WPU                  WPUCbits.WPUC5
#define SW2_OD                   ODCONCbits.ODCC5
#define SW2_ANS                  ANSELCbits.ANSELC5
#define SW2_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define SW2_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define SW2_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define SW2_GetValue()           PORTCbits.RC5
#define SW2_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define SW2_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define SW2_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define SW2_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define SW2_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define SW2_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define SW2_SetAnalogMode()      do { ANSELCbits.ANSELC5 = 1; } while(0)
#define SW2_SetDigitalMode()     do { ANSELCbits.ANSELC5 = 0; } while(0)

// get/set RC6 aliases
#define UART_TX1_TRIS                 TRISCbits.TRISC6
#define UART_TX1_LAT                  LATCbits.LATC6
#define UART_TX1_PORT                 PORTCbits.RC6
#define UART_TX1_WPU                  WPUCbits.WPUC6
#define UART_TX1_OD                   ODCONCbits.ODCC6
#define UART_TX1_ANS                  ANSELCbits.ANSELC6
#define UART_TX1_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define UART_TX1_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define UART_TX1_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define UART_TX1_GetValue()           PORTCbits.RC6
#define UART_TX1_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define UART_TX1_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define UART_TX1_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define UART_TX1_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define UART_TX1_SetPushPull()        do { ODCONCbits.ODCC6 = 0; } while(0)
#define UART_TX1_SetOpenDrain()       do { ODCONCbits.ODCC6 = 1; } while(0)
#define UART_TX1_SetAnalogMode()      do { ANSELCbits.ANSELC6 = 1; } while(0)
#define UART_TX1_SetDigitalMode()     do { ANSELCbits.ANSELC6 = 0; } while(0)

// get/set RC7 aliases
#define UART_RX1_TRIS                 TRISCbits.TRISC7
#define UART_RX1_LAT                  LATCbits.LATC7
#define UART_RX1_PORT                 PORTCbits.RC7
#define UART_RX1_WPU                  WPUCbits.WPUC7
#define UART_RX1_OD                   ODCONCbits.ODCC7
#define UART_RX1_ANS                  ANSELCbits.ANSELC7
#define UART_RX1_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define UART_RX1_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define UART_RX1_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define UART_RX1_GetValue()           PORTCbits.RC7
#define UART_RX1_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define UART_RX1_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define UART_RX1_SetPullup()          do { WPUCbits.WPUC7 = 1; } while(0)
#define UART_RX1_ResetPullup()        do { WPUCbits.WPUC7 = 0; } while(0)
#define UART_RX1_SetPushPull()        do { ODCONCbits.ODCC7 = 0; } while(0)
#define UART_RX1_SetOpenDrain()       do { ODCONCbits.ODCC7 = 1; } while(0)
#define UART_RX1_SetAnalogMode()      do { ANSELCbits.ANSELC7 = 1; } while(0)
#define UART_RX1_SetDigitalMode()     do { ANSELCbits.ANSELC7 = 0; } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/