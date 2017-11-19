/* 
 * File:   main.c
 * Author: Sean Carroll  (swc63)
 *
 * Created on September 30, 2016
 */
#define _SUPPRESS_PLIB_WARNING 1
#include "config.h"
#include "pt_cornell_1_2_1.h.h"
#include "port_expander.h"

// Blink LED every BLINK_LED milliseconds
#define BLINK_LED 1000

// === Thread Structures =======================================================
static struct pt pt_timer, pt_blink;

// system 1 second interval tick
int sys_time_seconds;

// === External Interrupt Service Routine ======================================
// Update the output pins on port expander to mirror input pins
void __ISR(_EXTERNAL_1_VECTOR, ipl2)ExternalInt1(void) {
  mINT1ClearIntFlag();
  unsigned char out = readPE(GPIOZ);
  writePE(GPIOY, out);
} // external ISR

// === Timer Thread ============================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt)){
    PT_BEGIN(pt);
    while(1){
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
    } // END WHILE(1)
    PT_END(pt);
} // timer thread

volatile int state = 0;
// blinks the on board LED every wait_LED mSec
static PT_THREAD (protothread_blink(struct pt *pt)){
    PT_BEGIN(pt);
    while(1){
        // blink LED
        mPORTAToggleBits(BIT_0);
        PT_YIELD_TIME_msec(BLINK_LED);
    } // END WHILE(1)
    PT_END(pt);
} // blink thread

int main(void) {
  ANSELA = 0; ANSELB = 0; 

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();    

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // === Set up timer ========================================================
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 0xFFFF);
  ConfigIntTimer2(T2_INT_OFF);
  mT2ClearIntFlag(); // and clear the interrupt flag

  // === set up IO Ports =======================================================
  mPORTASetBits(BIT_0); // makes sure it is off
  mPORTASetPinsDigitalOut(BIT_0); // sets port to output

  // === set up Port Expander ports ============================================
  initPE();
  // Outputs
  mPortYSetPinsOut(BIT_7 | BIT_6);
  // Inputs
  mPortZSetPinsIn(BIT_7 | BIT_6);
  // Input pull up resistors
  mPortZEnablePullUp(BIT_7 | BIT_6);
  // Set interrupt enabled on inputs
  mPortZIntEnable(BIT_7 | BIT_6);
  
  // Init interrupt pin on PIC32
  PPSInput(4, INT1, RPA3); // RPA3 (Pin 10)
  ConfigINT1(EXT_INT_PRI_2 | FALLING_EDGE_INT | EXT_INT_ENABLE);

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_blink);

  // Init LEDs
  unsigned char out = readPE(GPIOZ);
  writePE(GPIOY, out);
    
  // round-robin scheduler for threads
  while(1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      PT_SCHEDULE(protothread_blink(&pt_blink));
  } // round-robin loop
} // main

