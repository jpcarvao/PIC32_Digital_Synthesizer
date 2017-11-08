/* main.c
 * Joao Pedro Carvao
 * Albert Chu 
 * Francois Mertil
 *
 * Source code for our final project in ECE 4760 
 * 
*/
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define Fs 70000.0  // 70kHz
#define two32 4294967296.0 // 2^32 
#define frequency 440 
#define NUM_KEYS 2

#include "config.h"
#include "pt_cornell_1_2.h"
#include "tft_master.h"
#include "tft_gfx.h"
#include <math.h>

volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock

// === thread structures ============================================
static struct pt pt_read_button, pt_read_inputs, pt_freq_tune;  
// DDS sine table
#define SINE_TABLE_SIZE 256
volatile int sin_table[SINE_TABLE_SIZE];

//== Timer 2 interrupt handler ===========================================
// actual scaled DAC 
volatile int DAC_data;

volatile int frequencies[NUM_KEYS] = { 440, 554 };
volatile int frequencies_set[NUM_KEYS] = { 440, 554 };
// the DDS units:
//volatile unsigned int phase_accum_main = 0, phase_incr_main = frequency*two32/Fs;
volatile unsigned int phase_accum_main[NUM_KEYS]={0,0};
volatile unsigned int phase_incr_main[NUM_KEYS];

//volatile int modulation_constant=0;
volatile int test_mode_on = 0; 
volatile int ramp_done = 0;

volatile int ramp_counter = 0;
volatile int ramp_flag=0;
volatile int ramp_flag_in=0;
volatile int button_pressed[NUM_KEYS]= { 0, 0 };
volatile int button_pressed_in[NUM_KEYS] = {0, 0};
volatile int num_keys_pressed;

/* *** Keypad Macros *** */
// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
/*
E.g.
EnablePullDownB( BIT_7 | BIT_8 | BIT_9);
*/

/* Auxiliary Function definitions */


void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // 74 cycles to get to this point from timer event
    mT2ClearIntFlag();
    // main DDS phase
    DAC_data = 0;
    num_keys_pressed = 0;
    //sine_index = phase_accum_main>>24 ;
    int i;
    static int temp;
    for (i=0;i<NUM_KEYS;i++)
    {
   		if (button_pressed_in[i] || ramp_flag==-1) {
       	    phase_accum_main[i] += phase_incr_main[i];
            temp = phase_accum_main[i]>>24;
		    num_keys_pressed++;
		    DAC_data += sin_table[temp];
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
    // === Channel A =============
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // write to spi2 
    WriteSPI2( DAC_config_chan_A | ((ramp_counter*DAC_data)>>9)+2048);
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
    mPORTBSetBits(BIT_4); // end transaction
} // end ISR TIMER2

static PT_THREAD (protothread_read_inputs(struct pt *pt))
{
	PT_BEGIN(pt);
	static int none_pressed;
	static int none_pressed_old;
	while (1) {
		PT_YIELD_TIME_msec(5);
		none_pressed_old = none_pressed;
		none_pressed = 1;
        int i;
		for (i=0; i<NUM_KEYS; i++)
		{
            button_pressed_in[i] = button_pressed[i];
			if (button_pressed[i])
			{
				none_pressed=0;
				//ramp up if nothing was pressed before and now something is pressed, indicating a new sound 
				if (none_pressed_old)
				{
					ramp_flag=1;
				}
				//NEED CODE THAT KEEPS TRACK OF WHEN BUTTONS ARE PRESSED FOR TIMING 

			}
			else 
			{
                int y;
                y=2;
				//NEED CODE THAT KEEPS TRACK OF WHEN BUTTONS ARE RELEASED FOR TIMING 

			}
		}
		if (none_pressed) {
            ramp_flag=-1;
		}

	}
	PT_END(pt);
}

static PT_THREAD (protothread_read_button(struct pt *pt))
{
    PT_BEGIN(pt);
    static int pressed[NUM_KEYS]; 
    char buffer[256];
    mPORTBSetPinsDigitalIn(BIT_3);
    mPORTBSetPinsDigitalIn(BIT_7);
    #define ONE_SECOND 1000  
    static int state[NUM_KEYS];
    int i;
    for (i=0;i<NUM_KEYS;i++)
    {
        state[i] = 0;
    }
    while(1) {
        PT_YIELD_TIME_msec(30);
        pressed[0] = mPORTBReadBits(BIT_3);
        pressed[1] = mPORTBReadBits(BIT_7);
        for (i=0;i<NUM_KEYS;i++)
        {
            if (!state[i]) {
                if (pressed[i])
                {
                    state[i]=1;
                    button_pressed[i]=1;
                    //ramp_flag=1;
                }
            }
            else {
                if (!pressed[i])
                {
                    state[i]=0;
                    button_pressed[i]=0;
                    //ramp_flag=-1;
                }
            }
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


static PT_THREAD (protothread_freq_tune(struct pt *pt))
{
    PT_BEGIN(pt);
    char buffer[256];
    static float scale;
    static int adc_val;
    while(1) {
        PT_YIELD_TIME_msec(10);
        //frequency = ReadADC10(0);
        // 502 = 0
        static int j;
        adc_val = ReadADC10(0);
        scale = ((float)(1.5*(adc_val-502))/1024.0)+0.5;;
        for (j = 0; j < NUM_KEYS; j++) {
            frequencies[j] = ((int) frequencies_set[j] * scale);
            phase_incr_main[j]  = frequencies[j]*two32/Fs;
        }
        
        //phase_incr_main = frequency*two32/Fs;
        tft_fillRoundRect(0, 50, 400, 40, 1, ILI9340_BLACK);
        tft_setCursor(0,70);
        tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
        sprintf(buffer, "ADC raw: %d\n", ReadADC10(0));
        sprintf(buffer, "frequency: %d\n", frequencies[0]);
        tft_writeString(buffer);
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
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    
    // build the sine lookup table
    // scaled to produce values between 0 and 4096
    
    int i;
    for (i = 0; i < SINE_TABLE_SIZE; i++) 
        sin_table[i] = (int)(2047*sin((float)i*6.283/(float)SINE_TABLE_SIZE));
    
    
    // PT INIT
    PT_INIT(&pt_read_button);
    PT_INIT(&pt_read_inputs);
    PT_INIT(&pt_freq_tune);
    // scheduling loop 
    while(1) {
        PT_SCHEDULE(protothread_read_button(&pt_read_button));
        PT_SCHEDULE(protothread_read_inputs(&pt_read_inputs));
        PT_SCHEDULE(protothread_freq_tune(&pt_freq_tune));
    }    
}  // main