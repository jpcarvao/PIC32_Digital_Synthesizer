/* main.c
 * Joao Pedro Carvao
 * Albert Chu 
 * Francois Mertil
 *
 * Source code for our final project in ECE 4760 
 * 
*/
#include "config.h"
#include "pt_cornell_1_2.h"
#include "tft_master.h"
#include "tft_gfx.h"
#include <math.h>

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define Fs 70000.0  // 70kHz
#define two32 4294967296.0 // 2^32 
#define frequency 440 
#define NUM_KEYS 2

// === 16:16 fixed point macros ==========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)


volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock

// === thread structures ============================================
static struct pt pt_read_button, pt_read_inputs, pt_read_repeat, pt_freq_tune, pt_repeat_buttons;  

// DDS sine table
#define SINE_TABLE_SIZE 256
volatile fix16 sin_table[SINE_TABLE_SIZE];

//== Timer 2 interrupt handler ===========================================
// actual scaled DAC 
volatile short DAC_data;

// A4, C#4, and F#4 
volatile int frequencies[NUM_KEYS] = { 440, 554 };  // actual frequencies 
volatile int frequencies_set[NUM_KEYS] = {440, 554 };  // for freq modulation
volatile int frequencies_FM[NUM_KEYS] = {440*4,554*4};
// the DDS units:
//volatile unsigned int phase_accum_main = 0, phase_incr_main = frequency*two32/Fs;
volatile unsigned int phase_accum_main[NUM_KEYS] = {0,0};
volatile unsigned int phase_incr_main[NUM_KEYS];

volatile unsigned int phase_accum_FM[NUM_KEYS] = {0,0};
volatile unsigned int phase_incr_FM[NUM_KEYS];

//volatile int modulation_constant=0; 
volatile int ramp_done = 0;
volatile int ramp_counter = 0;
volatile int ramp_flag=0;
volatile int ramp_flag_in=0;
volatile int button_pressed[NUM_KEYS] = {0, 0};
volatile int button_pressed_in[NUM_KEYS] = {0, 0};
volatile int num_keys_pressed;

#define KEYPRESS_SIZE 256
volatile int keypresses[KEYPRESS_SIZE];
volatile int keypress_ID[KEYPRESS_SIZE];
volatile int keypress_count = 0;

volatile int repeat_mode_on = 0;
volatile int valid_size;
volatile float tempo;

#define MAX_FLANGER_SIZE 2048
volatile short flange_buffer[MAX_FLANGER_SIZE];
#define DELAY_RAMP_PERIOD 1000
volatile unsigned int current_flanger_delay = 2000;
volatile int flange_counter = 0;
volatile int delay_counter = 0;
volatile int flange_flag=-1;
volatile short delay_signal;
volatile int delay_on=0;


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
/*
E.g.
EnablePullDownB( BIT_7 | BIT_8 | BIT_9);
*/


#define ONE_SECOND 1000  // yield macro 


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
    static int temp;
    static int temp2;
    static int delay_index;
    
    for (i=0;i<NUM_KEYS;i++)
    {
        temp = phase_accum_FM[i]>>24;
   		if (button_pressed_in[i] || ramp_flag==-1) {
            phase_accum_FM[i] += phase_incr_FM[i];
       	    //phase_accum_main[i] += phase_incr_main[i] + ((unsigned int)sin_table[phase_accum_FM[i]>>24]);
            phase_accum_main[i] += phase_incr_main[i] + multfix16(float2fix16(0.5),sin_table[phase_accum_FM[i]>>24]);
            temp2 = phase_accum_main[i]>>24;
		    DAC_data += fix2int16(sin_table[temp2]);
		    num_keys_pressed++;
    	}
   }

    if (num_keys_pressed){
        DAC_data = DAC_data/num_keys_pressed;
    }
//    if (!ramp_done) {
//        ramp_flag=ramp_flag_in;
//    }
//    else {
//        ramp_flag=0;
//    }
    ramp_counter += ramp_flag;
    if (ramp_counter <= 0) {
        ramp_counter=0;
        ramp_flag=0;
    }
    if (ramp_counter>=511)
    {
        ramp_counter=511;
        ramp_flag=0;
    }
    
    //phase_accum_flange += phase_incr_flange;
    //current_flanger_delay = fix2int16(sin_table[phase_accum_flange>>24])+2048;
    //current_flanger_delay = current_flanger_delay>>1;
    
    delay_counter++;
    if (delay_counter==DELAY_RAMP_PERIOD) {
        current_flanger_delay+=flange_flag;
        delay_counter=0;
    }
    if (current_flanger_delay==0)
    {
        flange_flag=1;
    }
    if (current_flanger_delay>=MAX_FLANGER_SIZE)
    {
        flange_flag=-1; 
    }
    
    flange_counter++;
    if (flange_counter>MAX_FLANGER_SIZE) {
        flange_counter=0;
        delay_on=1;
    }
    flange_buffer[flange_counter]=(ramp_counter*DAC_data)>>9;
    delay_index=0;
    if (delay_on){
        delay_index = flange_counter - current_flanger_delay;
        if (delay_index<0) {
            delay_index+=MAX_FLANGER_SIZE;
        }
    }
    delay_signal = flange_buffer[delay_index];
    
    //DAC_data = (DAC_data+delay_signal);
    //DAC_data = delay_signal;
    
    // === Channel A =============
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // write to spi2 
    WriteSPI2( DAC_config_chan_A | ((ramp_counter*(DAC_data+delay_signal))>>10)+2048);
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
}  // end ISR TIMER2

static PT_THREAD (protothread_read_inputs(struct pt *pt))
{
	PT_BEGIN(pt);
	static int none_pressed;
	static int none_pressed_old;
    static int pressed_old[NUM_KEYS];
	while (1) {
		PT_YIELD_TIME_msec(5);
		none_pressed_old = none_pressed;
		none_pressed = 1;
        int i;
		for (i=0; i<NUM_KEYS; i++) {
            pressed_old[i] = button_pressed_in[i];
            button_pressed_in[i] = button_pressed[i];
			if (button_pressed_in[i]) {
				none_pressed=0;
				//ramp up if nothing was pressed before and now something is pressed, indicating a new sound 
				if (none_pressed_old) {
					ramp_flag=1;
				}
                
                if (!pressed_old[i]) {
                    //record time of press/release and which key was pressed/released
                    if (!repeat_mode_on) {
                        keypresses[keypress_count]=PT_GET_TIME();
                        keypress_ID[keypress_count]=i;
                        keypress_count++;
                    }
                }

			}
			else {                
                if (pressed_old[i]) {
                    //record time of press/release and which key was pressed/released
                    if (!repeat_mode_on) {
                        keypresses[keypress_count]=PT_GET_TIME();
                        keypress_ID[keypress_count]=i;
                        keypress_count++;
                    }
                }

			}
		}
		if (none_pressed) {
            ramp_flag=-1;
		}

	}
	PT_END(pt);
}

static PT_THREAD(protothread_read_repeat(struct pt *pt))
{
    PT_BEGIN(pt);
    static int state=0;
    while (1) {
        PT_YIELD_TIME_msec(30);
        if (mPORTBReadBits(BIT_8)) {
            repeat_mode_on=1;
            if (!state) {
                keypresses[keypress_count] = PT_GET_TIME();
                keypress_ID[keypress_count] = -1; 
                valid_size = keypress_count;
            }
            state=1;
        }
        else 
        {
            repeat_mode_on=0;
            state=0;
        }
    }
    PT_END(pt);
}

static PT_THREAD (protothread_read_button(struct pt *pt))
{
    PT_BEGIN(pt);
    static int pressed[NUM_KEYS]; 
    
    mPORTBSetPinsDigitalIn(BIT_3);
    mPORTBSetPinsDigitalIn(BIT_8);
    mPORTBSetPinsDigitalIn(BIT_7);  
    static int i;
    while(1) {
        PT_YIELD_TIME_msec(30);
        pressed[0] = mPORTBReadBits(BIT_3);
        pressed[1] = mPORTBReadBits(BIT_7);

        
            
        for (i=0;i<NUM_KEYS;i++) {
            if (pressed[i]) {
                button_pressed[i]=1;
            }
            else {
                button_pressed[i]=0;
            }
//            if (!state[i]) {
//                if (pressed[i])
//                {
//                    state[i]=1;
//                    //button_pressed[i]=1;
//                    //ramp_flag=1;
//                }
//            }
//            else {
//                if (!pressed[i])
//                {
//                    state[i]=0;
//                    //button_pressed[i]=0;
//                    //ramp_flag=-1;
//                }
//            }
        }
//        tft_fillRoundRect(0, 50, 400, 40, 1, ILI9340_BLACK);
//        tft_setCursor(0,50);
//        tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
//        sprintf(buffer, "%d\n", ramp_flag);
//        tft_writeString(buffer);
//        tft_fillRoundRect(0, 100, 400, 40, 1, ILI9340_BLACK);
//        tft_setCursor(0,100);
//        tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
//        sprintf(buffer, "%d\n", button_pressed[1]);
//        tft_writeString(buffer);
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
        for (j=0; j<NUM_KEYS; j++) {
            button_pressed[j]=0;
        }
        PT_YIELD_TIME_msec(keypresses[0]);
        for (i=0; i<valid_size; i++) {
            yield_length = keypresses[i+1]-keypresses[i];
            if (!button_pressed[keypress_ID[i]]) {
                button_pressed[keypress_ID[i]] = 1;
            }
            else {
                button_pressed[keypress_ID[i]] = 0;
            }
            PT_YIELD_TIME_msec(((int)yield_length*tempo));
//            if (i==1) {
//            tft_fillRoundRect(0, 50, 400, 60, 1, ILI9340_BLACK);
//            tft_setCursor(0,50);
//            tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
//            sprintf(buffer, "%d", yield_length);
//            tft_writeString(buffer);
//            }
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
    static float scale2;
    static int counter = 0;  // used to decrease rate of printing
    while(1) {
        PT_YIELD_TIME_msec(5); 
        // adc_val of 502 is 0 for freq modulation
        static int j;
        adc_val = ReadADC10(0);
        scale = ((float)(1.5*(adc_val-502))/1024.0)+1;
        scale2 = ((float)adc_val/1024.0);
        tempo = scale2*2;
        for (j = 0; j < NUM_KEYS; j++) {
            //frequencies[j] = ((int) frequencies_set[j] * scale);
            frequencies[j] = ((int) frequencies_set[j] * 1);
            phase_incr_main[j]  = frequencies[j]*two32/Fs;
            frequencies_FM[j] = 3*frequencies[j];
            phase_incr_FM[j]  = frequencies_FM[j]*two32/Fs;
        }
        // print every 500 ms -- will be removed later anyway 
        if (counter%50) {  
            tft_fillRoundRect(0, 90, 400, 60, 1, ILI9340_BLACK);
            tft_setCursor(0,90);
            tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
            sprintf(buffer, "%d", current_flanger_delay);
            //sprintf(buffer, "%d %d %d %d %d %d", keypresses[0], keypresses[1], keypresses[2], keypresses[3], keypresses[4], keypresses[5]);
            tft_writeString(buffer);
        }
        counter++;
    }
    PT_END(pt);
}

void adc_config(void)
{
    CloseADC10();  // ensure the ADC is off before setting the configuration
    // define setup parameters for OpenADC10
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF \
            | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2
    // set AN11 and  as analog inputs
    #define PARAM4  ENABLE_AN1_ANA  
    // do not assign channels to scan
    #define PARAM5  SKIP_SCAN_ALL
    // configure to sample AN11 
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11 );
    // configure ADC using the parameters defined above
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); 

    EnableADC10(); // Enable the ADC
}


void main(void)
{
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // 400 is 100 ksamples/sec at 30 MHz clock
    // 200 is 200 ksamples/sec
    // increased to 572 from 200 because was leading to incorrect sine freq
    //changing from 143
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_4, 143);   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); 
    
    /// SPI setup ////////////////////////////////
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
    
    // build the sine lookup table
    // scaled to produce values between 0 and 4096
    
   int i;
   for (i = 0; i < SINE_TABLE_SIZE; i++){
         sin_table[i] =  float2fix16(2047*sin((float)i*6.283/(float)SINE_TABLE_SIZE));
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
    
    // PT INIT
    PT_INIT(&pt_read_button);
    PT_INIT(&pt_read_inputs);
    PT_INIT(&pt_freq_tune);
    PT_INIT(&pt_read_repeat);
    PT_INIT(&pt_repeat_buttons);
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
    }    
}  // main