/* synth.h
 * Joao Pedro Carvao
 *
 * Header for main.c for our final project in ECE 4760
 * 
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
    
// === 16:16 fixed point macros ==============================================
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

#define Fs 20000.0  // 20kHz
#define two32 4294967296.0 // 2^32 
#define NUM_KEYS 13
//#define NUM_KEYS 2

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock

// === thread structures ======================================================
static struct pt pt_read_button, pt_read_inputs, pt_freq_tune, 
        pt_repeat_buttons, pt_cycle_button, pt_enter_button, pt_ui, 
        pt_ui_print, pt_read_mux;  

// DDS sine table
#define SINE_TABLE_SIZE 256
volatile fix16 sin_table[SINE_TABLE_SIZE];

//== Timer 2 interrupt handler ================================================
// actual scaled DAC 
volatile short DAC_data;

// actual frequencies 
volatile int frequencies[NUM_KEYS] = 
        {262, 277, 293, 311, 330, 349, 370, 392, 415, 440, 466 ,493, 523}; 
// for freq modulation
volatile int frequencies_set[NUM_KEYS] = 
        {262, 277, 293, 311, 330, 349, 370, 392, 415, 440, 466 ,493, 523};  

volatile int frequencies_FM[NUM_KEYS];
// the DDS units:
// for sine table
volatile unsigned int phase_accum_main[NUM_KEYS] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile unsigned int phase_incr_main[NUM_KEYS];
// for FM synthesis
volatile unsigned int phase_accum_FM[NUM_KEYS] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile unsigned int phase_incr_FM[NUM_KEYS];

//volatile int modulation_constant=0; 
volatile int ramp_done = 0;
volatile int ramp_counter = 0;
volatile int ramp_flag[NUM_KEYS]= {0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile int button_pressed[NUM_KEYS] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile int button_pressed_in[NUM_KEYS] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile int num_keys_pressed;

#define KEYPRESS_SIZE 256
volatile int keypresses[KEYPRESS_SIZE];
volatile int keypress_ID[KEYPRESS_SIZE];
volatile int keypress_count = 0;
volatile int start_recording = 0;

volatile int repeat_mode_on = 1;
volatile int valid_size;
volatile float tempo = 1;

//FX stuff
volatile float fx;
volatile float fm_ratio = 3;
volatile int DELAY_RAMP_PERIOD = 200;
volatile fix16 attack_main = float2fix16(0.05);
volatile fix16 dk_main = float2fix16(0.97);

// Flanging Stuff (Phase Shifting)
#define MAX_FLANGER_SIZE 2048
volatile short flange_buffer[MAX_FLANGER_SIZE];
volatile unsigned int current_flanger_delay = 0;
volatile int flange_counter = 0;
volatile int delay_counter = 0;
volatile int flange_flag=-1;

volatile short delay_signal;
volatile int delay_on=0;

// FM Synthesis Stuff
volatile fix16 dk_fm=float2fix16(0.99), attack_fm=float2fix16(0.02);
volatile fix16 dk_state_fm[NUM_KEYS], attack_state_fm[NUM_KEYS];
volatile fix16 dk_state_main[NUM_KEYS], attack_state_main[NUM_KEYS];
volatile fix16 fm_depth=float2fix16(2);
volatile fix16 env_fm[NUM_KEYS] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile fix16 env_main[NUM_KEYS] = {0,0,0,0,0,0,0,0,0,0,0,0,0};

volatile fix16 dk_state_fm_temp;
volatile fix16 dk_state_main_temp;
volatile fix16 attack_state_fm_temp;
volatile fix16 attack_state_main_temp;

volatile int dk_interval = 0;

volatile int sustain = 1;  // sustain on at startup

volatile int button_input=0;

// UI STUFF
volatile int flanger_on;
volatile int analog_noise_on;
volatile int fm_on = 1;  // donates if fm synth is on (on at startup)

// Enter Button Stuff
static int enter_pressed = 0;
static int enter_state = 0;
// Tempo Button Stuff
static short modified_tempo;
static short modified_pitch;
static int freq_adc = 0;
// flange stuff
static int flange_state = 0;  // state of flanger button state machine
static int flange_pressed;    // reads input for flanger button
// fm stuff 
static int fm_state = 1;  // fm synthesis on at start up 
static int fm_pressed;    // reads input for fm 
// Sustain button stuff 
static int sus_state = 1; // sustain on at start up
static int sus_pressed;   // reads input for sustain
// repeat button
static int repeat_pressed;
static int repeat_state = 1;


// Button State Machine Parameters
#define MOD_FM      0
#define MOD_FLANGER 1
#define MOD_ATK     2
#define MOD_DECAY   3

static int mod_param = 0;

static int cycle_pressed = 0;  // cycle button pressed 
static int cycle_state = 0;

/* Auxiliary Function definitions */
void adc_config(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SYNTH_H */

