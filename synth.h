/* 
 * File:   synth.h
 * Author: Lab User
 *
 * Created on November 21, 2017, 7:22 PM
 */

#ifndef SYNTH_H
#define	SYNTH_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "config.h"
#include "pt_cornell_1_2.h"
#include "tft_master.h"
#include "tft_gfx.h"
#include "port_expander_brl4.h"
#include <math.h>

// CS to allow for accessing DAC and PE
#define start_spi2_critical_section INTEnable(INT_T2, 0);
#define end_spi2_critical_section INTEnable(INT_T2, 1);
// DAC Control Bits
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
    
// === 16:16 fixed point macros ==========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))* \
        (( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)
#define onefix16 0x00010000 // int2fix16(1)

/* *** Keypad Macros *** */
// PORT B
#define EnablePullDownB(bits) CNPUBCLR = bits; CNPDBSET = bits;
#define DisablePullDownB(bits) CNPDBCLR = bits;
#define EnablePullUpB(bits) CNPDBCLR = bits; CNPUBSET = bits;
#define DisablePullUpB(bits) CNPUBCLR = bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR = bits; CNPDASET = bits;
#define DisablePullDownA(bits) CNPDACLR = bits;
#define EnablePullUpA(bits) CNPDACLR = bits; CNPUASET = bits;
#define DisablePullUpA(bits) CNPUACLR = bits;

#define ONE_SECOND 1000  // yield macro 

#ifdef	__cplusplus
}
#endif

#endif	/* SYNTH_H */

