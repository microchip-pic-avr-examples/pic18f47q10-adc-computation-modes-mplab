#include "application.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/timer/tmr0.h"
#include "mcc_generated_files/uart/eusart1.h"
#include "mcc_generated_files/adc/adcc.h"

void ADCC_Initialize_BasicMode(void);
void ADCC_Initialize_AverageMode(void);
void ADCC_Initialize_BurstAverageMode(void);
void ADCC_Initialize_LowPassFilterMode(void);
void ADCC_SetChannel(adcc_channel_t channel);

// global variables
volatile uint8_t count = 0;
volatile bool adcReadyFlag = false;
volatile bool basicModeInit = false;
volatile bool avgModeInit = false;
volatile bool burstAvgModeInit = false;
volatile bool lpfModeInit = false;

void ApplicationTask(void) 
{
    // change the ADCC modes based on button press
    
    switch (count) 
    {     
        case BASIC_MODE:
           
            if (!basicModeInit)
            {
                // stop the timer and configure it to 10ms for ADC auto conversion
                Timer0.Stop();
                Timer0.PeriodCountSet(TIMER0_10ms);
                // turn off both LEDs
                LED_D3_SetLow();
                LED_D2_SetLow();             
                // initialize ADC in basic mode with potentiometer as input
                ADCC_Initialize_BasicMode();
                ADCC_SetChannel(POT_DATA);
                // start Timer which in turn starts ADC conversion
                Timer0.Start();
                basicModeInit = true;
            } 
                // if ADC conversion is ready, print the data on serial port
            if (adcReadyFlag)
            {
                adcReadyFlag = false;
                printf("Basic Mode - ADRES=%d \r\n", ADRES);
            }
            
            break;
        case AVG_MODE:
            
            if (!avgModeInit)
            {
                // stop the timer
                Timer0.Stop();
                // turn on LED 1
                LED_D3_SetLow();
                LED_D2_SetHigh();       
                // initialize ADC in average mode with potentiometer as input
                ADCC_Initialize_AverageMode();
                ADCC_SetChannel(POT_DATA);
                // start Timer which in turn starts ADC conversion
                Timer0.Start();
                avgModeInit = true;
                basicModeInit = false;
            } 
              // if ADC value is within threshold limits, print the actual value
            if (adcReadyFlag)
            {
                adcReadyFlag = false;
                printf("AVG Mode - ADFLT=%d \r\n", ADFLTR);
            }
               // check if upper threshold is crossed
            if (ADCC_HasErrorCrossedUpperThreshold()) 
            {
                printf("AVG Mode - ADFLT=%d - SP=%d - UT=%d - UT Crossed\r\n", ADFLTR, ADSTPT, ADUTH);
                    
            } 
               // check if lower threshold is crossed
           else if (ADCC_HasErrorCrossedLowerThreshold()) 
            {
                    printf("AVG Mode - ADFLT=%d - SP=%d - LT=%d - LT Crossed\r\n", ADFLTR, ADSTPT, ADLTH);
           }
            break;
        case BURST_AVG_MODE:
            if (!burstAvgModeInit) 
            {
                // stop the timer
                Timer0.Stop();
                // turn on LED 2
                LED_D3_SetHigh();
                LED_D2_SetLow();            
                // initialize ADC in burst average mode with potentiometer as input
                ADCC_Initialize_BurstAverageMode();
                ADCC_SetChannel(POT_DATA);
                // start Timer which in turn starts ADC conversion
                Timer0.Start();
                burstAvgModeInit = true;
                avgModeInit = false;
            } 
         
             // if ADC value is within threshold limits, print the actual value
           if (adcReadyFlag)
            {
                adcReadyFlag = false;
                printf("Burst AVG Mode - ADFLT=%d \r\n", ADFLTR);
            }
               // check if upper threshold is crossed
           if (ADCC_HasErrorCrossedUpperThreshold())
            {
                printf("Burst AVG Mode - ADFLT=%d - SP=%d - UT=%d - UT Crossed\r\n", ADFLTR, ADSTPT, ADUTH);
                  
            } 
                 // check if lower threshold is crossed
           else if (ADCC_HasErrorCrossedLowerThreshold())
            {
               printf("Burst AVG Mode - ADFLT=%d - SP=%d - LT=%d - LT Crossed\r\n", ADFLTR, ADSTPT, ADLTH);
            }
           
            break;
        case LPF_MODE:
            if (!lpfModeInit) 
            {
                // stop the timer and configure it to 1ms for ADC auto conversion
                Timer0.Stop();
                Timer0.PeriodCountSet(TIMER0_1ms);
                // turn on both LEDs
                LED_D3_SetHigh();
                LED_D2_SetHigh();          
                // initialize ADC in low pass filter mode with DC offset having noise as input
                ADCC_Initialize_LowPassFilterMode();
                ADCC_SetChannel(FG_DATA);
                // start Timer which in turn starts ADC conversion
                Timer0.Start();
                lpfModeInit = true;
                burstAvgModeInit = false;
            }
                // send the low pass filter value to the data visualizer
            if (adcReadyFlag) 
            {    
                adcReadyFlag = false;
                while(!UART1.IsTxReady());
                EUSART1_Write(0x03);
                while(!UART1.IsTxReady());
                EUSART1_Write(ADRESL); // Visualizer reads low byte first
                while(!UART1.IsTxReady());
                EUSART1_Write(ADRESH); // Then reads the high byte// These are commands sent to the Data Visualizer, 0x03 = Start  
                while(!UART1.IsTxReady());
                EUSART1_Write(ADFLTRL); // Visualizer reads low byte first
                while(!UART1.IsTxReady());
                EUSART1_Write(ADFLTRH); // Then reads the high byte
                while(!UART1.IsTxReady());
                EUSART1_Write(0xFC); // Stop command  
                                    
            }
           
            break;
        default:
            count = BASIC_MODE;
            lpfModeInit = false;
            break;
    }
}

void ADCC_Initialize_BasicMode(void) 
{
    // ADCRS 0; ADMD Basic_mode; ADACLR disabled; ADPSIS ADRES; 
    ADCON2 = 0x00;
    // ADCALC First derivative of Single measurement; ADTMD enabled; ADSOI ADGO not cleared; 
    ADCON3 = 0x07;
    // ADACT TMR0; 
    ADACT = 0x02;
    // ADCS FOSC/64; 
    ADCLK = 0x1F;
    // ADGO stop; ADFM right; ADON enabled; ADCONT disabled; ADCS FOSC/ADCLK; 
    ADCON0 = 0x84;
}

void ADCC_Initialize_AverageMode(void) 
{
    // ADLTHL 56; 
    ADLTHL = 0x38;
    // ADLTHH 255;      // Lower threshold set to -200 diff from setpoint 
    ADLTHH = 0xFF;
    // ADUTHL 200; 
    ADUTHL = 0xC8;
    // ADUTHH 0;        // Upper threshold set to +200 diff from setpoint
    ADUTHH = 0x00;
    // ADSTPTL 255; 
    ADSTPTL = 0xFF;
    // ADSTPTH 1;       // Setpoint set to 511
    ADSTPTH = 0x01;
    // ADRPT 16; 
    ADRPT = 0x10;
    // ADCRS 4; ADMD Average_mode; ADACLR disabled; ADPSIS ADRES; 
    ADCON2 = 0x42;
    // ADCALC Actual result vs setpoint; ADTMD ADERR > ADLTH and ADERR < ADUTH; ADSOI ADGO not cleared; 
    ADCON3 = 0x13;
    // ADACT TMR0; 
    ADACT = 0x02;
    // ADCS FOSC/64; 
    ADCLK = 0x1F;
    // ADGO stop; ADFM right; ADON enabled; ADCONT disabled; ADCS FOSC/ADCLK; 
    ADCON0 = 0x84;
}

void ADCC_Initialize_BurstAverageMode(void)
{
    // ADLTHL 56; 
    ADLTHL = 0x38;
    // ADLTHH 255;      // Lower threshold set to -200 diff from setpoint 
    ADLTHH = 0xFF;
    // ADUTHL 200; 
    ADUTHL = 0xC8;
    // ADUTHH 0;        // Upper threshold set to +200 diff from setpoint
    ADUTHH = 0x00;
    // ADSTPTL 255; 
    ADSTPTL = 0xFF;
    // ADSTPTH 1;       // Setpoint set to 511
    ADSTPTH = 0x01;
    // ADRPT 16; 
    ADRPT = 0x10;
    // ADCRS 4; ADMD Burst_average_mode; ADACLR disabled; ADPSIS ADRES; 
    ADCON2 = 0x43;
    // ADCALC Actual result vs setpoint; ADTMD ADERR > ADLTH and ADERR < ADUTH; ADSOI ADGO not cleared; 
    ADCON3 = 0x13;
    // ADACT TMR0; 
    ADACT = 0x02;
    // ADCS FOSC/64; 
    ADCLK = 0x1F;
    // ADGO stop; ADFM right; ADON enabled; ADCONT disabled; ADCS FOSC/ADCLK; 
    ADCON0 = 0x84;
}

void ADCC_Initialize_LowPassFilterMode(void) 
{
    // ADRPT 16; 
    ADRPT = 0x10;
    // ADCRS 3; ADMD Low_pass_filter_mode; ADACLR disabled; ADPSIS ADRES; 
    ADCON2 = 0x34;
    // ADCALC Filtered value vs setpoint; ADTMD enabled; ADSOI ADGO not cleared; 
    ADCON3 = 0x57;
    // ADACT TMR0; 
    ADACT = 0x02;
    // ADCS FOSC/64; 
    ADCLK = 0x1F;
    // ADGO stop; ADFM right; ADON enabled; ADCONT disabled; ADCS FOSC/ADCLK; 
    ADCON0 = 0x84;
}
 
void ADCC_SetChannel(adcc_channel_t channel)
{
    // select the A/D channel
    ADPCH = channel;
}
void ADCUserInterrupt(void)
{
    adcReadyFlag = true; 
}

void TMR4UserInterrupt(void)
{
    if (SW2_GetValue()) 
    {
        count++;
    }
} 