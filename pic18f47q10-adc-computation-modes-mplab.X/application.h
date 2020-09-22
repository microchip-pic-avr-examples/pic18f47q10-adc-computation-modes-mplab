/* 
 * File:   application.h
 * Author: I16092
 *
 * Created on May 22, 2018, 7:54 PM
 */

#ifndef APPLICATION_H
#define	APPLICATION_H

#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/tmr0.h"
#include "mcc_generated_files/eusart1.h"
#include "mcc_generated_files/adcc.h"

#ifdef	__cplusplus
extern "C" {
#endif

    
// global variables
volatile uint8_t count = 0;
volatile bool adcReadyFlag = false;
volatile bool basicModeInit = false;
volatile bool avgModeInit = false;
volatile bool burstAvgModeInit = false;
volatile bool lpfModeInit = false;

// #define
#define BASIC_MODE      0
#define AVG_MODE        1
#define BURST_AVG_MODE  2
#define LPF_MODE        3

#define TIMER0_10ms     0x13
#define TIMER0_1ms      0x01

// set the analog channel for ADCC
void ADCC_SetChannel(adcc_channel_t channel);

// application task
void ApplicationTask(void);


#ifdef	__cplusplus
}
#endif

#endif	/* APPLICATION_H */

