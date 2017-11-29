/* 
 * File:   button_test.c
 * Author: Lab User
 *
 * Created on November 28, 2017, 6:48 PM
 */

#include "synth.h"
static struct pt pt_button;

static PT_THREAD (protothread_button(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1) {
        
    }
    PT_END(pt);
}

void main(void) 
{

    #define ISR_PERIOD 
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_4, 508);   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); 
    
    /// SPI setup ////////////////////////////////
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | 
            SPI_OPEN_CKE_REV , spiClkDiv);
   
  
    ANSELA = 0; ANSELB = 0; CM1CON = 0; CM2CON = 0;
    
    // === config threads ==========
    // turns OFF UART support and debugger pin
    PT_setup();

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();
    
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(1); // Use tft_setRotation(1) for 320x240
    
    pt_init(&pt_button);
    
    while(1) {
        PT_SCHEDULE(protothread_button(&pt_button));
    }
}

