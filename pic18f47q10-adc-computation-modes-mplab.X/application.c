#include "application.h"

void ADCC_SetChannel(adcc_channel_t channel) {
    // select the A/D channel
    ADPCH = channel;
}

void ApplicationTask (void) {
    // change the ADCC modes based on button press
    switch(count) {
        case BASIC_MODE:
            if(!basicModeInit) {
                // stop the timer and configure it to 10ms for ADC auto conversion
                TMR0_StopTimer();
                TMR0_Reload(TIMER0_10ms);
                // turn off both LEDs
                LED_D3_SetLow();
                LED_D2_SetLow();
                // initialize ADC in basic mode with potentiometer as input
                ADCC_Initialize_BasicMode();
                ADCC_SetChannel(POT_DATA);
                // start Timer which in turn starts ADC conversion
                TMR0_StartTimer();
                basicModeInit = true;
            } else {
                // if ADC conversion is ready, print the data on serial port
                if(adcReadyFlag) {
                    adcReadyFlag = false;
                    printf ("Basic Mode - ADRES=%d \r\n", ADRES);
                }
            }
            break;
        case AVG_MODE:
            if(!avgModeInit) {
                // stop the timer
                TMR0_StopTimer();
                // turn on LED 1
                LED_D3_SetLow();
                LED_D2_SetHigh();
                // initialize ADC in average mode with potentiometer as input
                ADCC_Initialize_AverageMode();
                ADCC_SetChannel(POT_DATA);
                // start Timer which in turn starts ADC conversion
                TMR0_StartTimer();
                avgModeInit = true;
                basicModeInit = false;
            } else {
                // if ADC value is within threshold limits, print the actual value
                if(adcReadyFlag) {
                    adcReadyFlag = false;
                    printf ("AVG Mode - ADFLT=%d \r\n", ADFLTR);
                }
                // check if upper threshold is crossed
                if(ADCC_HasErrorCrossedUpperThreshold()) {
                    printf ("AVG Mode - ADFLT=%d - SP=%d - UT=%d - UT Crossed\r\n", ADFLTR,ADSTPT,ADUTH);
                // check if lower threshold is crossed
                } else if(ADCC_HasErrorCrossedLowerThreshold()) {
                    printf ("AVG Mode - ADFLT=%d - SP=%d - LT=%d - LT Crossed\r\n", ADFLTR,ADSTPT,ADLTH);
                } 
            }
            break;
        case BURST_AVG_MODE:
            if(!burstAvgModeInit) {
                // stop the timer
                TMR0_StopTimer();
                // turn on LED 2
                LED_D3_SetHigh();
                LED_D2_SetLow();
                // initialize ADC in burst average mode with potentiometer as input
                ADCC_Initialize_BurstAverageMode();
                ADCC_SetChannel(POT_DATA);
                // start Timer which in turn starts ADC conversion
                TMR0_StartTimer();
                burstAvgModeInit = true;
                avgModeInit = false;
            } else {
                // if ADC value is within threshold limits, print the actual value
                if(adcReadyFlag) {
                    adcReadyFlag = false;
                    printf ("Burst AVG Mode - ADFLT=%d \r\n", ADFLTR);
                }
                // check if upper threshold is crossed
                if(ADCC_HasErrorCrossedUpperThreshold()) {
                    printf ("Burst AVG Mode - ADFLT=%d - SP=%d - UT=%d - UT Crossed\r\n", ADFLTR,ADSTPT,ADUTH);
                // check if lower threshold is crossed
                } else if(ADCC_HasErrorCrossedLowerThreshold()) {
                    printf ("Burst AVG Mode - ADFLT=%d - SP=%d - LT=%d - LT Crossed\r\n", ADFLTR,ADSTPT,ADLTH);
                } 
            }
            break;
        case LPF_MODE:
            if(!lpfModeInit) {
                // stop the timer and configure it to 1ms for ADC auto conversion
                TMR0_StopTimer();
                TMR0_Reload(TIMER0_1ms);
                // turn on both LEDs
                LED_D3_SetHigh();
                LED_D2_SetHigh();
                // initialize ADC in low pass filter mode with DC offset having noise as input
                ADCC_Initialize_LowPassFilterMode();
                ADCC_SetChannel(FG_DATA);
                // start Timer which in turn starts ADC conversion
                TMR0_StartTimer();
                lpfModeInit = true;
                burstAvgModeInit = false;
            } else {
                // send the low pass filter value to the data visualizer
                if(adcReadyFlag) {
                    adcReadyFlag = false;
                    EUSART1_Write(0x03); 
                    EUSART1_Write(ADRESL);                                                   // Visualizer reads low byte first
                    EUSART1_Write(ADRESH);                                                   // Then reads the high byte// These are commands sent to the Data Visualizer, 0x03 = Start                                              
                    EUSART1_Write(ADFLTRL);                                                   // Visualizer reads low byte first
                    EUSART1_Write(ADFLTRH);                                                   // Then reads the high byte
                    EUSART1_Write(0xFC);                                                      // Stop command 
                }
            }
            break; 
        default:
            count = BASIC_MODE;
            lpfModeInit = false;
            break;
    }
}
