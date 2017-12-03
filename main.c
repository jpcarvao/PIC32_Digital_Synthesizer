/* main.c
 * Joao Pedro Carvao
 * Albert Chu 
 * Francois Mertil
 *
 * Source code for our final project in ECE 4760 
 * 
*/

#include "synth.h"

#define Fs 20000.0  // 70kHz
#define two32 4294967296.0 // 2^32 
//#define NUM_KEYS 13
#define NUM_KEYS 2

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock

// === thread structures ============================================
static struct pt pt_read_button, pt_read_inputs, pt_read_repeat, pt_freq_tune, 
        pt_repeat_buttons, pt_cycle_button, pt_enter_button, pt_ui, 
        pt_ui_print, pt_read_mux;  

// DDS sine table
#define SINE_TABLE_SIZE 256
volatile fix16 sin_table[SINE_TABLE_SIZE];

//== Timer 2 interrupt handler ===========================================
// actual scaled DAC 
volatile short DAC_data;

// A4, C#4, and F#4 
//volatile int frequencies[NUM_KEYS] = {262, 277, 293, 311, 330, 349, 370, 392, 415, 440, 466 ,493, 523};  // actual frequencies 
//volatile int frequencies_set[NUM_KEYS] = {262, 277, 293, 311, 330, 349, 370, 392, 415, 440, 466 ,493, 523};  // for freq modulation
volatile int frequencies[NUM_KEYS] = {262, 277};
volatile int frequencies_set[NUM_KEYS] = {262, 277};
volatile int frequencies_FM[NUM_KEYS];
// the DDS units:
//volatile unsigned int phase_accum_main = 0, phase_incr_main = frequency*two32/Fs;
// for sine table
volatile unsigned int phase_accum_main[NUM_KEYS] = {0,0};
volatile unsigned int phase_incr_main[NUM_KEYS];
// for FM synthesis
volatile unsigned int phase_accum_FM[NUM_KEYS] = {0,0};
volatile unsigned int phase_incr_FM[NUM_KEYS];

//volatile int modulation_constant=0; 
volatile int ramp_done = 0;
volatile int ramp_counter = 0;
volatile int ramp_flag[NUM_KEYS]= {0, 0};
volatile int button_pressed[NUM_KEYS] = {0, 0};
volatile int button_pressed_in[NUM_KEYS] = {0, 0};
volatile int num_keys_pressed;

#define KEYPRESS_SIZE 256
volatile int keypresses[KEYPRESS_SIZE];
volatile int keypress_ID[KEYPRESS_SIZE];
volatile int keypress_count = 0;

volatile int repeat_mode_on = 0;
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
volatile fix16 env_fm[NUM_KEYS] = {0,0};
volatile fix16 env_main[NUM_KEYS] = {0,0};

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
volatile int fm_on = 1;     // donates if fm synth is on (on at startup)

/* Auxiliary Function definitions */
void adc_config(void);

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // 74 cycles to get to this point from timer event
    mT2ClearIntFlag();
    // main DDS phase
    DAC_data = 0;
    num_keys_pressed = 0;
    
    int i;
    static int delay_index;
    static int dk_flag;
    
    dk_flag = 0;
    
    
    dk_interval++;
    if (dk_interval == 255) {
        dk_flag = 1;
        dk_interval = 0;
    }
    // FM synthesis
    for (i = 0; i < NUM_KEYS; i++) {
        if (ramp_flag[i])
        {
            dk_state_fm[i] = fm_depth; 
            dk_state_main[i] = onefix16; 
            attack_state_fm[i] = fm_depth; 
            attack_state_main[i] = onefix16; 
        }
    
         if (dk_flag) {
            dk_state_fm_temp = dk_state_fm[i];
            dk_state_main_temp = dk_state_main[i];
            attack_state_fm_temp = attack_state_fm[i];
            attack_state_main_temp = attack_state_main[i];
            dk_state_fm_temp = multfix16(dk_state_fm_temp, dk_fm) ;
            dk_state_main_temp = multfix16(dk_state_main_temp, dk_main) ;
            attack_state_fm_temp = multfix16(attack_state_fm_temp, attack_fm);
            attack_state_main_temp = multfix16(attack_state_main_temp, attack_main);
            env_fm[i] = multfix16(fm_depth-attack_state_fm_temp, dk_state_fm_temp) ;
            
            if (sustain && button_pressed_in[i]) {
                dk_state_main[i] = onefix16;
                dk_state_fm[i] = fm_depth;
            }
            else {
                dk_state_main[i] = dk_state_main_temp;
                dk_state_fm[i] = dk_state_fm_temp;
            }
            env_main[i] = multfix16(onefix16-attack_state_main_temp, dk_state_main_temp);
            attack_state_fm[i] = attack_state_fm_temp;
            attack_state_main[i] = attack_state_main_temp;
        }
        
   		if (button_pressed_in[i] || ramp_flag[i]==0) {
            phase_accum_FM[i] += phase_incr_FM[i];
            if (fm_on) {//toggled by button in pt ui
                phase_accum_main[i] += phase_incr_main[i] + multfix16(env_fm[i],sin_table[phase_accum_FM[i]>>24]);
            }
            else {
                phase_accum_main[i] += phase_incr_main[i];
            }
		    DAC_data += ((fix2int16(env_main[i]<<9))*fix2int16(sin_table[phase_accum_main[i]>>24]))>>9;
            //DAC_data += fix2int16(sin_table[phase_accum_main[i]>>24]);
		    num_keys_pressed++;
    	}
    }
    // normalize key presses
    if (num_keys_pressed) {
        DAC_data = DAC_data/num_keys_pressed;
    }
    
    if (flanger_on) {  // toggled by button in pt_ui
        delay_counter++;
        if (delay_counter == DELAY_RAMP_PERIOD) {
            current_flanger_delay += flange_flag;
            delay_counter = 0;
        }
        if (current_flanger_delay <= 0){
            flange_flag = 1;
        }
        if (current_flanger_delay >= MAX_FLANGER_SIZE/2) {
            flange_flag = -1; 
        }

        flange_counter++;
        // MAX_FLANGER_SIZE-1 to avoid negative index in keys_pressed[i]
        if (flange_counter > MAX_FLANGER_SIZE-1) {
            flange_counter = 0;
            delay_on=1;
        }
        flange_buffer[flange_counter] = DAC_data;
        delay_index = 0;
        if (delay_on){
            delay_index = flange_counter - current_flanger_delay;
            if (delay_index < 0) {
                delay_index += MAX_FLANGER_SIZE;
            }
        }
        delay_signal = flange_buffer[delay_index];
    }    
        
    int junk;
    // === Channel A =============
    // CS low to start transaction
    SPI_Mode16();
     mPORTBClearBits(BIT_4); // start transaction
    // write to spi2 
    WriteSPI2( DAC_config_chan_A | ((DAC_data+delay_signal)>>1)+2048);
    //WriteSPI2( DAC_config_chan_A | (delay_signal)+2048);
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS high
    junk = ReadSPI2(); 
    mPORTBSetBits(BIT_4); // end transaction
}// end ISR TIMER2

static PT_THREAD (protothread_read_inputs(struct pt *pt))
{
	PT_BEGIN(pt);
    static int pressed_old[NUM_KEYS];
	while (1) {
		PT_YIELD_TIME_msec(1);
        int i;
		for (i=0; i < NUM_KEYS; i++) {
            pressed_old[i] = button_pressed_in[i];
            button_pressed_in[i] = button_pressed[i];
            ramp_flag[i] = 0;
			if (button_pressed_in[i]) {
				//ramp up if nothing was pressed before and now something is pressed, indicating a new sound
                if (!pressed_old[i]) {
                    ramp_flag[i] = 1;
                    //record time of press/release and which key was pressed/released
                    if (!repeat_mode_on) {
                        keypresses[keypress_count] = PT_GET_TIME();
                        keypress_ID[keypress_count] = i;
                        keypress_count++;
                    }
                }

			}
			else {                
                if (pressed_old[i]) {
                    //record time of press/release and which key was pressed/released
                    if (!repeat_mode_on) {
                        keypresses[keypress_count] = PT_GET_TIME();
                        keypress_ID[keypress_count] = i;
                        keypress_count++;
                    }
                }
			}
		}

	}
	PT_END(pt);
}


static PT_THREAD(protothread_read_repeat(struct pt *pt))
{
    PT_BEGIN(pt);
    mPORTASetPinsDigitalIn(BIT_0);
    static int state = 0;
    while (1) {
        PT_YIELD_TIME_msec(30);
        if (mPORTAReadBits(BIT_0)) {
            repeat_mode_on = 1;
            if (!state) {
                keypresses[keypress_count] = PT_GET_TIME();
                keypress_ID[keypress_count] = -1; 
                valid_size = keypress_count;
            }
            state = 1;
        }
        else {
            repeat_mode_on = 0;
            state = 0;
        }
    }
    PT_END(pt);
}


static PT_THREAD (protothread_read_button(struct pt *pt))
{
    PT_BEGIN(pt);
    static int pressed[NUM_KEYS];
    static int inputY;
    static int inputZ;
    static int input;
    start_spi2_critical_section;
    initPE();
    mPortYSetPinsIn(BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6);
    mPortYEnablePullUp(BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6);
    
    mPortZSetPinsIn(BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5);
    mPortZEnablePullUp(BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5);
    end_spi2_critical_section ;
        
    while (1) {
        PT_YIELD_TIME_msec(1);
        start_spi2_critical_section;
        inputY = readPE(GPIOY);
        inputZ = readPE(GPIOZ);
        end_spi2_critical_section;
        input = inputY<<6;
        input = input | inputZ;
        button_input = input;
        int i;
        for (i=0; i<NUM_KEYS;i++) {
            //ensure that important bit is least significant, then mask to make 
            //sure that entire number is 1 or 0
            button_pressed[i] = !((input>>i) & 1);
        }
    }
    PT_END(pt);
}

static PT_THREAD (protothread_repeat_buttons(struct pt *pt))
{
    PT_BEGIN(pt);
    static int yield_length;
    static int i;
    static int j;
    char buffer[256];
    while (1) {
        for (j = 0; j < NUM_KEYS; j++) {
            button_pressed[j]=0;
        }
        PT_YIELD_TIME_msec(keypresses[0]);
        for (i = 0; i < valid_size; i++) {
            yield_length = keypresses[i+1] - keypresses[i];
            if (!button_pressed[keypress_ID[i]]) {
                button_pressed[keypress_ID[i]] = 1; 
            }
            else {
                button_pressed[keypress_ID[i]] = 0;
            }
            if (i != valid_size-1) {
                PT_YIELD_TIME_msec(((int)yield_length*tempo));
            }
        }
    }
    PT_END(pt);
}


static PT_THREAD (protothread_freq_tune(struct pt *pt))
{
    PT_BEGIN(pt);
    char buffer[256];
    static float scale;
    static int adc_val;
    static int adc_freq;
    static float scale2;
    static int counter = 0;  // used to decrease rate of printing
    while (1) {
        PT_YIELD_TIME_msec(5); 
        // adc_val of 502 is 0 for freq modulation
        static int j;
        adc_val = ReadADC10(0);
        adc_freq = ReadADC10(1);
        scale = ((float)(1.5*(adc_freq-502))/1024.0)+1;
        scale2 = ((float)adc_val/1024.0);
        fx = scale2;
        for (j = 0; j < NUM_KEYS; j++) {
            frequencies[j] = ((int) frequencies_set[j] * scale);
            //frequencies[j] = ((int) frequencies_set[j] * 1);
            phase_incr_main[j]  = frequencies[j]*two32/Fs;
            // fm synth 
            frequencies_FM[j] = fm_ratio*frequencies[j];
            phase_incr_FM[j]  = frequencies_FM[j]*two32/Fs;
        }
        // print every 500 ms -- will be removed later anyway 
        if (counter%50) {  
            tft_fillRoundRect(0, 90, 100, 10, 1, ILI9340_BLACK);
            tft_setCursor(0,90);
            tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
            sprintf(buffer, "%d", button_input);
            tft_writeString(buffer);
        }
        counter++;
    }
    PT_END(pt);
}

static int mod_param = 0;

static int cycle_pressed = 0;  // cycle button pressed 
static int cycle_state = 0;

static PT_THREAD (protothread_cycle_button(struct pt *pt))
{
	PT_BEGIN(pt);
	// push button set up
    
	char buffer[60];
	while (1) {	
		PT_YIELD_TIME_msec(30);
        
		if (!cycle_state) {
			if (cycle_pressed) {
				cycle_state = 1;
				mod_param++;
				if (mod_param == 4) {
					mod_param = 0;
				}
			}
		}
		else {
			if (!cycle_pressed) {
				cycle_state = 0;
			}
		}
	}
	PT_END(pt);
}

static int enter_pressed = 0;
static int enter_state = 0;

#define MOD_FM      0
#define MOD_FLANGER 1
#define MOD_ATK     2
#define MOD_DECAY   3

static PT_THREAD (protothread_enter_button(struct pt *pt))
{
	PT_BEGIN(pt);
    char buffer[60];
	while (1) {
		PT_YIELD_TIME_msec(30);
		if (!enter_state) {
			if (enter_pressed) {
				enter_state = 1;
				switch (mod_param) {
					case MOD_FM:
                        fm_ratio = fx*3+1;
						break;
					case MOD_FLANGER:
                        DELAY_RAMP_PERIOD = fx*1000 + 100;
						break;
					case MOD_ATK:
                        attack_main = fx*.1 + 0;
						break;
					case MOD_DECAY:
                        dk_main = fx*.1 + .9;
						break;
					default:
                        break;
						// do something 
				}
			}
		}
        else {
            if (!enter_pressed) {
                enter_state=0;
            }
        }
	}
	PT_END(pt);
}


// UI globals
static short modified_fx;
static short modified_pitch;
static int freq_adc = 0;
// flange stuff
static int flange_state = 0;  // state of flanger button state machine
static int flange_pressed;    // reads input for flanger button
// fm stuff 
static int fm_state = 1;  // fm synthesis on at start up 
static int fm_pressed;    // reads input for fm 
// sustain stuff 
static int sus_state = 1; // sustain on at start up
static int sus_pressed;   // reads input for sustain
// tone stack stuff
static int stack_on;
// repeat button
static int repeat_pressed;

/* Thread that Controls the User Interface */
static PT_THREAD (protothread_ui(struct pt *pt))
{
    PT_BEGIN(pt);
    char buffer[256];
    while (1) {
        PT_YIELD_TIME_msec(30);
        // flanger button state machine ===================================== 
        if(!flange_state) {
            if (flange_pressed) {  
                flange_state = 1;
                // toggle flanger_on 
                if (!flanger_on) flanger_on = 1;
                else flanger_on = 0;  
            }
        }
        else if (!flange_pressed) {
                flange_state = 0;
        }
        
        
        // divide by 4 for visibility improvement
        modified_fx = ReadADC10(0)>>3;
        
        // pitch display setting
        modified_pitch = frequencies[1]>>3;
        // adc 2 test
        freq_adc = ReadADC10(1);
        
        // FM synth button state machine ======================================
        if(!fm_state) {
            if (fm_pressed) {  
                fm_state = 1;
                // toggle flanger_on 
                if (!fm_on) fm_on = 1;
                else fm_on = 0;  
            }
        }
        else if (!fm_pressed) {
                fm_state = 0;
        }
        
        // sustain button state machine =======================================
        if(!sus_state) {
            if (sus_pressed) {  
                sus_state = 1;
                // toggle flanger_on 
                if (!sustain) sustain = 1;
                else sustain = 0;  
            }
        }
        else if (!sus_pressed) {
                sus_state = 0;
        }
    }
    PT_END(pt);
}

static PT_THREAD (protothread_ui_print(struct pt *pt))
{
    PT_BEGIN(pt);
    char buffer[256];
    while (1) {
        // print every 500 ms to prevent synthesis failure
        PT_YIELD_TIME_msec(500);
        //tft_fillRoundRect(0, 40, 450, 250, 1, ILI9340_BLACK);
        // flanger print ==================================================
        tft_setCursor(1,40);
        tft_fillRoundRect(0, 40, 125, 10, 1, ILI9340_BLACK);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        if (flanger_on){
            sprintf(buffer, "Flanger: on, %d", flange_pressed);
        }
        else {
            sprintf(buffer, "Flanger: off, %d", flange_pressed );
        }
        tft_writeString(buffer);

        // repeat mode print ==================================================
        tft_fillRoundRect(0, 60, 125, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,60);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
//        if (repeat_mode_on) {
//            sprintf(buffer, "Repeat Mode: on");
//        }
//        else {
//            sprintf(buffer, "Repeat Mode: off");
//        }
        sprintf(buffer, "Repeat Mode: %d", repeat_pressed);
        tft_writeString(buffer);

        // analog noise print =================================================
        tft_fillRoundRect(0, 75, 125, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,75);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        if (analog_noise_on) {
            sprintf(buffer, "Analog Noise: on, %d", analog_noise_on);
        }
        else {
            sprintf(buffer, "Analog Noise: off, %d", analog_noise_on);
        }
        tft_writeString(buffer);

        // Tempo Display ======================================================
        tft_fillRoundRect(75, 100, 130, 25, 1, ILI9340_BLACK);
        tft_fillRoundRect(75, 100, modified_fx, 25, 1, ILI9340_RED );

        // Pitch Display ======================================================
        tft_fillRoundRect(75, 125, 130, 25, 1, ILI9340_BLACK);
        tft_fillRoundRect(75, 125, modified_pitch, 25, 1, ILI9340_GREEN );

        // ADC 2 Test -- Remove in final
        tft_fillRoundRect(0, 160, 200, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,160);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        sprintf(buffer, "freq_adc (AN5) : %d, %1.3f", freq_adc, 
                ((float)(1.5*(freq_adc-502))/1024.0)+1);
        tft_writeString(buffer);
        
        // FM synth state display =============================================
        tft_fillRoundRect(0, 170, 125, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,170);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        if (fm_on) sprintf(buffer, "FM Synth: on, %d", fm_pressed);
        else sprintf(buffer, "FM Synth: off, %d", fm_pressed);
        tft_writeString(buffer);
        
        // Sustain state display ==============================================
        tft_fillRoundRect(0, 180, 125, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,180);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        if (sustain) sprintf(buffer, "Sustain: on, %d", sus_pressed);
        else sprintf(buffer, "Sustain: off, %d", sus_pressed);
        tft_writeString(buffer);
        
        
        // mod_param print ====================================================
        tft_fillRoundRect(0, 190, 125, 10, 1, ILI9340_BLACK);
        switch (mod_param) {
            case MOD_FM:
                tft_setCursor(1,190);
                tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
                sprintf(buffer, "Modify FM Synthesis");
                tft_writeString(buffer);
                break;
            case MOD_FLANGER:
                tft_setCursor(1,190);
                tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
                sprintf(buffer, "Modify Flanger");
                tft_writeString(buffer);
                break;
            case MOD_ATK:
                tft_setCursor(1,190);
                tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
                sprintf(buffer, "Modify Attack Time");
                tft_writeString(buffer);
                break;
            case MOD_DECAY:
                tft_setCursor(1,190);
                tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
                sprintf(buffer, "Modify Decay Time: %d");
                tft_writeString(buffer);
                break;
            default:
                // do something 
                tft_setCursor(1,190);
                tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
                sprintf(buffer, "mod_param error");
                tft_writeString(buffer);
                break;      
        }
        // Cycle Button Display ===============================================
        tft_fillRoundRect(0, 200, 125, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,200);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        sprintf(buffer, "cycle_pressed: %d", cycle_pressed);
        tft_writeString(buffer);
        // Enter Button Display ===============================================
        tft_fillRoundRect(0, 210, 125, 10, 1, ILI9340_BLACK);
        tft_setCursor(1,210);
        tft_setTextColor(ILI9340_YELLOW);  tft_setTextSize(1);
        sprintf(buffer, "enter button: %d", enter_pressed);
        tft_writeString(buffer);
        }
    PT_END(pt);
}

static PT_THREAD (protothread_read_mux(struct pt *pt ))
{
    PT_BEGIN(pt);
    char buffer[256];
    
    // UI Analog Mux Ports ====================================================
    mPORTBSetPinsDigitalOut(BIT_7);   // A
    mPORTBSetPinsDigitalOut(BIT_10);  // B
    mPORTBSetPinsDigitalOut(BIT_13);  // C
    
    EnablePullUpB(BIT_0);
    mPORTBSetPinsDigitalIn(BIT_8); // read mux   
    
    while (1) {
        // flanger Toggle =====================================================
        //ABC = 000;
        mPORTBClearBits(BIT_7 | BIT_10 | BIT_13);
        //yield necessary otherwise you use the select mask from select signal
        PT_YIELD_TIME_msec(10);
        flange_pressed = mPORTBReadBits(BIT_8);
        // FM Synth Toggle ====================================================
        // ABC = 100
        mPORTBClearBits(BIT_10 | BIT_13);
        mPORTBSetBits(BIT_7);
        PT_YIELD_TIME_msec(10);
        fm_pressed = mPORTBReadBits(BIT_8);
        // Sustain Toggle =====================================================
        // ABC = 010
        mPORTBClearBits(BIT_7 | BIT_13);
        mPORTBSetBits(BIT_10);
        PT_YIELD_TIME_msec(10);
        sus_pressed = mPORTBReadBits(BIT_8);
        // Tone Stack Switch ==================================================
        // ABC = 110
        mPORTBClearBits(BIT_13);
        mPORTBSetBits(BIT_7 | BIT_10);
        PT_YIELD_TIME_msec(10);
        analog_noise_on = mPORTBReadBits(BIT_8); 
        // Cycle Button =======================================================
        // ABC = 001 
        mPORTBClearBits(BIT_7 | BIT_10);
        mPORTBSetBits(BIT_13);
        PT_YIELD_TIME_msec(10);
		cycle_pressed = mPORTBReadBits(BIT_8);
        // Enter Button =======================================================
        // ABC = 101
        mPORTBClearBits(BIT_10);
        mPORTBSetBits(BIT_7 | BIT_13);
        PT_YIELD_TIME_msec(10);
		enter_pressed = mPORTBReadBits(BIT_8);
        // Repeat Button ======================================================
        // ABC = 011
        mPORTBClearBits(BIT_7);
        mPORTBSetBits(BIT_10 | BIT_13);
        PT_YIELD_TIME_msec(10);
		repeat_pressed = mPORTBReadBits(BIT_8);
    }
    PT_END(pt);
}

/* Configures Analog-to-Digital Converter */
void adc_config(void)
{
    CloseADC10(); // ensure the ADC is off before setting the configuration
    // define setup parameters for OpenADC10
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON 
    // define setup parameters for OpenADC10
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE \
            | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF \
            | ADC_ALT_INPUT_ON
    // Define setup parameters for OpenADC10
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy 
    // define setup parameters for OpenADC10
    #define PARAM4 ENABLE_AN1_ANA | ENABLE_AN5_ANA 
    // define setup parameters for OpenADC10
    #define PARAM5 SKIP_SCAN_ALL 
    // configure to sample AN5 and AN1 on MUX A and B
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 \
            | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );
    // configure ADC 
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); 

    EnableADC10(); // Enable the ADC
}

/* Configures peripherals, timing, interrups, and schedules+cofigures threads*/
void main(void)
{
    // Timing and ISR =========================================================
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // 400 is 100 ksamples/sec at 30 MHz clock
    // 200 is 200 ksamples/sec
    // increased to 572 from 200 because was leading to incorrect sine freq
    //changing from 143
    //#define ISR_PERIOD 
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_4, 508);   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); 
    
    // SPI + DAC setup ========================================================
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    
    // config adc 
    adc_config();
    
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | 
            SPI_OPEN_CKE_REV , spiClkDiv);
   
    ANSELA = 0; ANSELB = 0; CM1CON = 0; CM2CON = 0;
    
    // config threads =========================================================
    PT_setup();

    // setup system wide interrupts  ==========================================
    INTEnableSystemMultiVectoredInt();
    
    // TFT ====================================================================
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    //240x320 vertical display
    tft_setRotation(1); // Use tft_setRotation(1) for 320x240
    
    // build the sine lookup table
    // scaled to produce values between 0 and 4096
    
    // Sine Tables ============================================================
    int i;
    for (i = 0; i < SINE_TABLE_SIZE; i++){
        sin_table[i] =  
                float2fix16(2047*sin((float)i*6.283/(float)SINE_TABLE_SIZE));
    }
   
    int j;
    for (j=0; j<KEYPRESS_SIZE; j++) {
       keypresses[j] = 0;
       keypress_ID[j] = 0;
    }
   
    int k;
    for (k=0; k<MAX_FLANGER_SIZE; k++) {
        flange_buffer[k] = 0;
    }
    
    
    // PT INIT ================================================================
    PT_INIT(&pt_read_button);
    PT_INIT(&pt_read_inputs);
    PT_INIT(&pt_freq_tune);
    PT_INIT(&pt_read_repeat);
    PT_INIT(&pt_repeat_buttons);
    PT_INIT(&pt_cycle_button);
    PT_INIT(&pt_enter_button);
    PT_INIT(&pt_ui);
    PT_INIT(&pt_ui_print);
    
    PT_INIT(&pt_read_mux);
    // scheduling loop 
    while(1) {
        if (!repeat_mode_on){
            PT_SCHEDULE(protothread_read_button(&pt_read_button));
        }
        else {
            PT_SCHEDULE(protothread_repeat_buttons(&pt_repeat_buttons));
        }
        PT_SCHEDULE(protothread_read_inputs(&pt_read_inputs));
        PT_SCHEDULE(protothread_freq_tune(&pt_freq_tune));
        PT_SCHEDULE(protothread_read_repeat(&pt_read_repeat));
        PT_SCHEDULE(protothread_cycle_button(&pt_cycle_button));
        PT_SCHEDULE(protothread_enter_button(&pt_enter_button));
        PT_SCHEDULE(protothread_read_mux(&pt_read_mux));
        PT_SCHEDULE(protothread_ui(&pt_ui));
        PT_SCHEDULE(protothread_ui_print(&pt_ui_print));
    }    
}  // main