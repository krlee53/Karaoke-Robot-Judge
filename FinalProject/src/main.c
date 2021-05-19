/*
 * main.c
 * gtz4, klj92
 * 
 * This program is used for testing things that will eventually go into the 
 * final main.c program
 */

////////////////////////////////////
// clock AND protoThreads configure
#include "config_1_3_2.h"
// threading library
#include "pt_cornell_1_3_2_python.h"
#include <math.h>

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"

// midi stuff
#include "midi_lookup.h"
#include "saveme.h"
#include "saveme_vocal.h"
#include "youbelongwithme.h"
#include "youbelongwithme_vocal.h"
#include "loststars.h"
#include "loststars_vocal.h"

//== Timer 2 interrupt handler (for DAC) ======================================
// direct digital synthesis of sine wave
#define two32 4294967296.0 // 2^32 
#define Fs 40000
#define WAIT {}
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000
//
volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for DAC!!
// the DDS units:
volatile unsigned int phase_accum_main1, phase_accum_main2, phase_accum_main3;
volatile unsigned _Accum amplitude1 = 0, amplitude2 = 0, amplitude3 = 0;
volatile unsigned int fcalc = two32/Fs;
// DDS sine table
#define sine_table_size 256
volatile _Accum sin_table[sine_table_size] ;
// the dds state controlled by python interface
volatile int dds_state = 1;
// the voltage specifed from python
volatile float V_data = 0;
// sine, sq, tri
volatile char wave_type = 0 ;

volatile unsigned int time;
volatile unsigned int note;
volatile int quit = 0;

// midi stuff
volatile int row = 0;
volatile int midi_count = 0;
volatile int duration = 0;
volatile int scaler = 0;

// ISR timing variables
volatile _Accum clicks_per_sec;
volatile int new_tempo = 0; // determine if tempo is being changed
volatile _Accum samples_per_click = 36;
volatile int sample_count = 0; // max value = samples_per_sec 

// extra DAC stuff
volatile _Accum data1, data2, data3;
volatile int duration1, duration2, duration3;
volatile _Accum scaler1, scaler2, scaler3;
volatile int phase_inc1, phase_inc2, phase_inc3;

//====== Timer 4 interrupt (for ADC) ====================================
volatile _Accum channel4;	// conversion result as read from result buffer

// Array sizes
#define nSamp 512
#define nPixels 256

// FFT
#define N_WAVE          512    /* size of FFT 512 */
#define LOG2_N_WAVE     9     /* log2(N_WAVE) 0 */

volatile int ADC_count = 0;
volatile _Accum omega[512];
_Accum fr[512];
_Accum fi[512];
_Accum amplitudes[512];
_Accum max_amp;
int max_freq = 0;

// volatiles for the stuff used in the ISR
volatile unsigned _Accum DAC_value; // Vref output
volatile _Accum channel4;	// conversion result as read from result buffer

_Accum v_in[nSamp] ;

_Accum Sinewave[N_WAVE]; // a table of sines for the FFT
_Accum window[N_WAVE]; // a table of window values for the FFT
_Accum fr[N_WAVE], fi[N_WAVE];
int pixels[nPixels] ;
int column = 50;
int freq_scale = 8; // 2*2000Hz/512 rounded, this scaling covers most singing ranges
unsigned int (* song_p)[3];
unsigned int (* vocal_p)[2];
unsigned int size;
unsigned _Accum time_sig;
unsigned int vocal_tempo;
unsigned int vocal_size;
unsigned int header;
unsigned int vocal_start;

volatile int sampling = 0;
volatile int start = 0;
_Accum prescaler = 1;
_Accum decrement = 1;

int mel_col = 0;
int mel_row = 0;
_Accum mel_dur = 0;
int mel_note = 0;

int pan = 75000;
int tilt = 66000;
char direction = 1;
char wrong = 0;
volatile int robot_count = 0;
char breath_count = 0;

int wrong_count = 0;
int total_count = 0;

char pause = 0;

//volatile int rampup1 = 0, rampup2 = 0, rampup3 = 0;
//volatile int rampdown1 = 0, rampdown2 = 0, rampdown3 = 0;
//volatile int midi1 = 0, midi2 = 0, midi3 = 0;

// timer 4 interrupt, DAC
void __ISR(_TIMER_4_VECTOR, ipl3) Timer4Handler(void)
{
    // maximum 3 notes at once?
    
    // you MUST clear the ISR flag
    mT4ClearIntFlag();
    if (start == 1){
        if (row < size){
            // check midi time 
            time = *(*(song_p + row) + 0);
            
            if (midi_count == vocal_start){
                sampling = 1;
                column = 0;
            }

            if(sample_count >= (int)samples_per_click){ // go to next click 
                if (row > 0){ // not at very beginning of file
                    if(*(*(song_p + row - 1) + 0) != time){ // if previous row does not have same time
                        // just continue incrementing
                        midi_count++;
                        sample_count = 0;
                    } // otherwise stay on same click
                } // otherwise at beginning of file
            }
            else sample_count++;

            if (time == midi_count){
                note = *(*(song_p + row) + 1);
                duration = *(*(song_p + row) + 2);
                if (note > 127){
                    // recalculate tempo
                    clicks_per_sec = (_Accum)(1000000.0/(float)note)*time_sig;
                    // bpm (quarter notes/min) = 60E6(us/min)/tempo(us/quarter note)
                    // bpm(quarter notes/min)/60(s/min) = (quarter notes/s)
                    // (quarter notes/s)*(time sig (clicks/quarter note)) = clicks/s
                    samples_per_click = (_Accum)((480/header)*2000)/clicks_per_sec;
                    // (2000 samples/s)/(clicks/s) = samples/click
                }
                else{ // not tempo
                    // check to see which has the lowest amplitude
                    if (amplitude1 == 0) { // use DDS unit 1
                        phase_inc1 = (int)midi_lookup[note-21]*fcalc;
                        duration1 = duration+midi_count;
                        scaler1 = 1/(_Accum)duration;
                        amplitude1 = 1;
                    }
                    else if (amplitude2 == 0) { // use DDS unit 2
                        phase_inc2 = (int)midi_lookup[note-21]*fcalc;
                        duration2 = duration+midi_count;
                        scaler2 = 1/(_Accum)duration;
                        amplitude2 = 1;
                    }
                    else { // use DDS unit 3
                        phase_inc3 = (int)midi_lookup[note-21]*fcalc;
                        duration3 = duration+midi_count;
                        scaler3 = 1/(_Accum)duration;
                        amplitude3 = 1;
                    }
                }
                row++;
            }
        }

        phase_accum_main1 += phase_inc1;
        phase_accum_main2 += phase_inc2;
        phase_accum_main3 += phase_inc3;

        // scale the dds units
        if (amplitude1 > 0) amplitude1 = (_Accum)(duration1-midi_count)*scaler1;
        if (amplitude2 > 0) amplitude2 = (_Accum)(duration2-midi_count)*scaler2;
        if (amplitude3 > 0) amplitude3 = (_Accum)(duration3-midi_count)*scaler3;

        // amplitude and DAC data calculations 
        data1 = sin_table[phase_accum_main1>>24]*amplitude1;
        data2 = sin_table[phase_accum_main2>>24]*amplitude2;
        data3 = sin_table[phase_accum_main3>>24]*amplitude3;

        DAC_data = (int)(data1 + data2 + data3);

        if (row >= size){  
            if(sample_count >= (int)samples_per_click){ // go to next click 
                midi_count++;
                sample_count = 0;
            }
            else sample_count++;
            if (amplitude1 == 0 && amplitude2 == 0 && amplitude3 == 0) quit = 1;
        }

        // === DAC Channel A =============
        // wait for possible port expander transactions to complete
        // CS low to start transaction
         mPORTBClearBits(BIT_4); // start transaction
        // write to spi2 
        if (1)
            WriteSPI2( DAC_config_chan_A | ((DAC_data + 2048) & 0xfff));
        while (SPI2STATbits.SPIBUSY) WAIT; // wait for end of transaction
         // CS high
        mPORTBSetBits(BIT_4) ; // end transaction
       //  
    }
}

// timer 5 interrupt, ADC
void __ISR(_TIMER_5_VECTOR, ipl4) Timer5Handler(void)
{
    // clear the interrupt flag
    mT5ClearIntFlag();
    // read the ADC
    // read the first buffer position
    channel4 = (_Accum)ReadADC10(0);   // read the result of channel 4 conversion from the idle buffer
    AcquireADC10(); // not needed if ADC_AUTO_SAMPLING_ON below
    
    if (ADC_count < 512){
        // scale the sample and put it in array
        omega[ADC_count] = window[ADC_count]*channel4;
        ADC_count++;
    }
}

// === timer 23, robot interrupt =====================
void __ISR(_TIMER_3_VECTOR, ipl5) Timer3Handler(void)
{
    mT3ClearIntFlag();
    SetDCOC3PWM(pan);
    SetDCOC4PWM(tilt);
    
    robot_count++;
}

//=== song done =========================
static PT_THREAD (protothread_quit(struct pt *pt))
{
    PT_BEGIN(pt);
    
    while (1){
        PT_YIELD_UNTIL(pt, quit == 1);
        INTDisableInterrupts();
        // clear display
        tft_fillScreen(ILI9340_BLACK);
        // reset variables
        row = 0;
        midi_count = 0;
        sample_count = 0;
        column = 0;
        mel_dur = 0;
        mel_row = 0;
        sampling = 0;
        ADC_count = 0;
        amplitude1 = 0;
        amplitude2 = 0;
        amplitude3 = 0;
        breath_count = 0;
        total_count = 0;
        wrong_count = 0;
        wrong = 0;
        pan = 75000;
        tilt = 66000;
        pause = 0;
        
    }
    
    PT_END(pt);
}

//=== FFT ==============================================================
void FFTfix(_Accum fr[], _Accum fi[], int m)
//Adapted from code by:
//Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
//fr[n],fi[n] are real,imaginary arrays, INPUT AND RESULT.
//size of data = 2**m
// This routine does foward transform only
{
    int mr,nn,i,j,L,k,istep, n;
    _Accum qr,qi,tr,ti,wr,wi;

    mr = 0;
    n = 1<<m;   //number of points
    nn = n - 1;

    /* decimation in time - re-order data */
    for(m=1; m<=nn; ++m)
    {
        L = n;
        do L >>= 1; while(mr+L > nn);
        mr = (mr & (L-1)) + L;
        if(mr <= m) continue;
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
    }

    L = 1;
    k = LOG2_N_WAVE-1;
    while(L < n)
    {
        istep = L << 1;
        for(m=0; m<L; ++m)
        {
            j = m << k;
            wr =  Sinewave[j+N_WAVE/4];
            wi = -Sinewave[j];

            for(i=m; i<n; i+=istep)
            {
                j = i + L;
                tr = (wr*fr[j]) - (wi*fi[j]);
                ti = (wr*fi[j]) + (wi*fr[j]);
                qr = fr[i] >> 1;
                qi = fi[i] >> 1;
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            }
        }
        --k;
        L = istep;
    }
}

// === display thread ======================================================
static PT_THREAD (protothread_display(struct pt *pt))
{
    PT_BEGIN(pt);
    
    static int draw_length = 5;
    static _Accum mel_scale;
    
    while (1){
        // wait until sample array is full
        PT_YIELD_UNTIL(pt, (ADC_count >= 512 && quit == 0 && sampling == 1));
        // disable interrupt
        DisableIntT5;
        // copy scaled samples into fr
        static int jj;
        for (jj = 0; jj < 512; jj++){
            fr[jj] = omega[jj];
        }
        // signal ISR to refill array
        ADC_count = 0;
        // enable interrupt
        EnableIntT5;

        // populate fi with zeros
        static int ii;
        for (ii = 0; ii < 512; ii++){
            fi[ii] = 0;
        }
        
        // do FFT
        FFTfix(fr, fi, LOG2_N_WAVE);
        // get magnitude and log
        // The magnitude of the FFT is approximated as: 
        //   |amplitude|=max(|Re|,|Im|)+0.4*min(|Re|,|Im|). 
        // This approximation is accurate within about 4% rms.
        // https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
        // alpha max beta min algorithm
        static int kk;
        for (kk = 16; kk < nPixels; kk++) {  
            // get the approx magnitude
            // reuse fr to hold magnitude
            fr[kk] = max(abs(fr[kk]), abs(fi[kk])) + 
                    (min(abs(fr[kk]), abs(fi[kk]))*(_Accum)0.4); 

            // find frequency with max amplitude            
            if (fr[kk] > max_amp){
                max_amp = fr[kk];
                max_freq = kk;
            }
        } // end for

        // Reset column location to beginning once it reaches edge of screen
        if (column > 320){
            column = 0;
        }
        
        mel_scale = (_Accum)prescaler*(_Accum)draw_length/(clicks_per_sec*20.48);
        
        // draw melody line
        if (mel_dur <= 0){
            mel_dur = (_Accum)(*(*(vocal_p + mel_row)+1))*mel_scale; 
            mel_note = *(*(vocal_p + mel_row));
            if (mel_note < 127 && mel_note > 0){
                mel_note = (int)(midi_lookup[mel_note - 21]);
            }
            mel_row++;
        }
        // draw gaps
        tft_fillRect(column, 0, 3*draw_length, 240, ILI9340_BLACK);
        if (mel_note > 0){
            tft_fillRect(column, -((int)(mel_note*0.128))+230, draw_length, 5, ILI9340_RED);
        }
        mel_dur = mel_dur - decrement;
        
        // taking breaths or singing
        if (max_amp == 0) {
            max_freq = 0;
            breath_count++;
        }
        else {
            // Display vocal line on TFT
            tft_fillRect(column, -max_freq+230, draw_length, 5, ILI9340_WHITE);
            breath_count = 0;
        }
        
        total_count++;
        
        // if not sing for too long or out of tune, then add to wrong count
        if (abs(mel_note - max_freq) > 200 || breath_count > 10){
            wrong_count++;
        }
        
        // wrong, make robot turn away
        if (wrong_count > 30){
            wrong = 1;
            robot_count = 0;
        }
        
        // reset wrong variables
        if (total_count > 60){
            wrong_count = 0;
            total_count = 0;
        }
        
        // shift over column
        column = column + draw_length;
        
        // reset max amplitude to zero
        max_amp = 0;
        
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
}

// === robot thread ===========================================================
static PT_THREAD (protothread_robot(struct pt *pt))
{
    PT_BEGIN(pt);
    
    while(1){
        PT_YIELD_UNTIL(pt, sampling == 1);
        // if wrong, make robot turn away 
        if (wrong == 1 || breath_count > 10){
            pan = 30000;
            tilt = 90000;
            if (robot_count > 75){
                wrong = 0;
            }
        }
        else {
            // robot bops head 
            static int ii;
            pan = 75000;
            if (tilt < 75000 && direction){
                tilt++;
            } else {
                tilt--;
                direction = 0;
                if (tilt <= 60000){
                    direction = 1;
                }
            }
        }
    }
    
    PT_END(pt);
}

// === outputs from python handler =============================================
// signals from the python handler thread to other threads
// These will be used with the prototreads PT_YIELD_UNTIL(pt, condition);
// to act as semaphores to the processing threads
char new_string = 0;
char new_button = 0;
char new_list = 0 ;
// identifiers and values of controls
// curent button
char button_id, button_value ;
// current listbox
int list_id, list_value ; 
// current string
char receive_string[64];

// === string input thread =====================================================
// process text from python
static PT_THREAD (protothread_python_string(struct pt *pt))
{
    PT_BEGIN(pt);
    static int dds_freq;
    // 
    while(1){
        // wait for a new string from Python
        PT_YIELD_UNTIL(pt, new_string==1);
        new_string = 0;
        // parse frequency command
        if (receive_string[0] == 'f'){
           
        }
        //
        else if (receive_string[0] == 'v'){
            
        }
        //
        else if (receive_string[0] == 'h'){
           
        }
        //
        else {
                 
        }
    } // END WHILE(1)   
    PT_END(pt);  
} // thread python_string

// === Buttons thread ==========================================================
// process buttons from Python for clear LCD and blink the on-board LED
static PT_THREAD (protothread_buttons(struct pt *pt))
{
    PT_BEGIN(pt);
    // set up LED port A0 to blink
    mPORTAClearBits(BIT_0 );	//Clear bits to ensure light is off.
    mPORTASetPinsDigitalOut(BIT_0);    //Set port as output
    while(1){
        PT_YIELD_UNTIL(pt, new_button==1);
        // clear flag
        new_button = 0; 
        // Button one -- start
        if (button_id==1 && button_value==1){
            start = 1;
            quit = 0;
            INTEnableInterrupts();
        }
        // Button 2 -- pause/resume
        if (button_id==2 && button_value==1){
            if (pause == 0){
                pause = 1;
                INTDisableInterrupts();
            } else {
                pause = 0;
                INTEnableInterrupts();
            }
        }
        // Button 3 -- quit
        if (button_id==3 && button_value==1){
            quit = 1;
            start = 0;
        }
    } // END WHILE(1)   
    PT_END(pt);  
} // thread blink

// ===  listbox thread =========================================================
// process listbox from Python to set DDS waveform
static PT_THREAD (protothread_listbox(struct pt *pt))
{
    PT_BEGIN(pt);
    while(1){
        PT_YIELD_UNTIL(pt, new_list==1);
        // clear flag
        new_list = 0; 
        if (list_id == 1){ // save me 
            if (list_value == 0){
                song_p = saveme;
                vocal_p = saveme_vocal;
                time_sig = saveme_time_sig;
                size = saveme_size;
                vocal_tempo = saveme_vocal_tempo;
                vocal_size = saveme_vocal_size;
                header = saveme_header;
                vocal_start = saveme_start_vocal;
                prescaler = 0.1;
                decrement = 0.1;
            } 
            else if (list_value == 1){ // you belong with me 
                song_p = ybwm;
                vocal_p = ybwm_vocal;
                time_sig = ybwm_time_sig;
                size = ybwm_size;
                vocal_tempo = ybwm_vocal_tempo;
                vocal_size = ybwm_vocal_size;
                header = ybwm_header;
                vocal_start = ybwm_start_vocal;
                prescaler = 2;
                decrement = 0.5;
            }
            else if (list_value == 2){ // lost stars
                song_p = loststars;
                vocal_p = loststars_vocal;
                time_sig = loststars_time_sig;
                size = loststars_size;
                vocal_tempo = loststars_vocal_tempo;
                vocal_size = loststars_vocal_size;
                header = loststars_header;
                vocal_start = loststars_start_vocal;
                prescaler = 0.1;
                decrement = 0.1;
            }
        }
    } // END WHILE(1)   
    PT_END(pt);  
} // thread listbox

// === Python serial thread ====================================================
// you should not need to change this thread UNLESS you add new control types
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    static char junk;
    //   
    //
    while(1){
        // There is no YIELD in this loop because there are
        // YIELDS in the spawned threads that determine the 
        // execution rate while WAITING for machine input
        // =============================================
        // NOTE!! -- to use serial spawned functions
        // you MUST edit config_1_3_2 to
        // (1) uncomment the line -- #define use_uart_serial
        // (2) SET the baud rate to match the PC terminal
        // =============================================
        
        // now wait for machine input from python
        // Terminate on the usual <enter key>
        PT_terminate_char = '\r' ; 
        PT_terminate_count = 0 ; 
        PT_terminate_time = 0 ;
        // note that there will NO visual feedback using the following function
        PT_SPAWN(pt, &pt_input, PT_GetMachineBuffer(&pt_input) );
        
        // Parse the string from Python
        // There can be button and string events
        
        // pushbutton
        if (PT_term_buffer[0]=='b'){
            // signal the button thread
            new_button = 1;
            // subtracting '0' converts ascii to binary for 1 character
            button_id = (PT_term_buffer[1] - '0')*10 + (PT_term_buffer[2] - '0');
            button_value = PT_term_buffer[3] - '0';
        }
        
        // listbox
        if (PT_term_buffer[0]=='l'){
            new_list = 1;
            list_id = PT_term_buffer[2] - '0' ;
            list_value = PT_term_buffer[3] - '0';
            //printf("%d %d", list_id, list_value);
        }
        
        // string from python input line
        if (PT_term_buffer[0]=='$'){
            // signal parsing thread
            new_string = 1;
            // output to thread which parses the string
            // while striping off the '$'
            strcpy(receive_string, PT_term_buffer+1);
        }                                  
    } // END WHILE(1)   
    PT_END(pt);  
} // thread blink


// === Main  ======================================================

void main(void) {

  // === set up DAC on big board ========
  // timer interrupt //////////////////////////
    // Set up timer3 on,  interrupts, internal clock, prescalar 1, toggle rate
    #define TIMEOUT3 (40000000/Fs) // clock rate / sample rate
//    #define TIMEOUT (40000000/297)
    // 2000 is 20 ksamp/sec
    OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_1, TIMEOUT3);

    // set up the timer interrupt with a priority of 3
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_3);
    mT4ClearIntFlag(); // and clear the interrupt flag

    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 2);
  // === end DAC setup =========
    
  // === set up PWM ==========
    #define robot_timeout (40000000/50)
    OpenTimer23(T2_ON | T2_SOURCE_INT | T2_PS_1_1, robot_timeout);
    ConfigIntTimer23(T3_INT_ON | T3_INT_PRIOR_5);
    mT3ClearIntFlag();
    
    OpenOC3(OC_ON | OC_TIMER_MODE32 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 60000, 60000);
    OpenOC4(OC_ON | OC_TIMER_MODE32 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 60000, 60000);
    
    PPSOutput(4, RPA3, OC3);
    PPSOutput(3, RPA2, OC4);
  // === end PWM setup ========
    
  // === set up ADC on big board =====
    // timer 2 setup for adc trigger  ==============================================
    // Set up timer2 on,  no interrupts, internal clock, prescalar 1, compare-value
    // This timer generates the time base for each ADC sample. 
    // works at ??? Hz
    #define sample_rate 4000 // 5kHz
    // 40 MHz PB clock rate
    #define timer_match 40000000/sample_rate
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_1, timer_match);
    
    // set up the timer interrupt with a priority of 2
    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2);
    mT5ClearIntFlag(); // and clear the interrupt flag
    
    // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration 
    
    // define setup parameters for OpenADC10
    // Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_OFF //

    // define setup parameters for OpenADC10
    // ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    //
    // Define setup parameters for OpenADC10
    // for a 40 MHz PB clock rate
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy should work at 40 MHz.
    // ADC_SAMPLE_TIME_6 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_6 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_5| ADC_CONV_CLK_Tcy2
    
    // define setup parameters for OpenADC10
    // set AN11 and  as analog inputs
    #define PARAM4	ENABLE_AN11_ANA // pin 24

    // define setup parameters for OpenADC10
    // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL
    
     // use ground as neg ref for A | use AN11 for input A     
    // configure to sample AN11 
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11); 
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5); // configure ADC using the parameters defined above
    
    EnableADC10(); // Enable the ADC
    
  // === build the sine lookup table =======
   // scaled to produce values between 0 and 4096
   int ii;
   for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = (_Accum)(512*sin((float)ii*6.283/(float)sine_table_size));
    }
   
   // populate Sinewave and Hann window arrays
  for (ii = 0; ii < N_WAVE; ii++) {
      Sinewave[ii] = (_Accum)(sin(6.283 * ((float) ii) / N_WAVE)*0.5);
      window[ii] = (_Accum)(0.5 * (1.0 - cos(6.283 * ((float) ii) / (N_WAVE - 1))));
  }
 
    // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
  // === TFT setup ============================
  // init the display in main since more than one thread uses it.
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(1); // Use tft_setRotation(1) for 320x240
  
  // === config threads ========================
  PT_setup();
  
  // === identify the threads to the scheduler =====
  // add the thread function pointers to be scheduled
  // --- Two parameters: function_name and rate. ---
  // rate=0 fastest, rate=1 half, rate=2 quarter, rate=3 eighth, rate=4 sixteenth,
  // rate=5 or greater DISABLE thread!
  
  pt_add(protothread_display, 0);
  pt_add(protothread_quit, 0);
  pt_add(protothread_buttons, 0);
  pt_add(protothread_listbox, 0);
  pt_add(protothread_python_string, 0);
  pt_add(protothread_serial, 0);
  pt_add(protothread_robot, 0);
  
  // === initalize the scheduler ====================
  PT_INIT(&pt_sched) ;
  // >>> CHOOSE the scheduler method: <<<
  // (1)
  // SCHED_ROUND_ROBIN just cycles thru all defined threads
  //pt_sched_method = SCHED_ROUND_ROBIN ;
  
  // NOTE the controller must run in SCHED_ROUND_ROBIN mode
  // ALSO note that the scheduler is modified to cpy a char
  // from uart1 to uart2 for the controller
  
  pt_sched_method = SCHED_ROUND_ROBIN ;
  
  // === scheduler thread =======================
  // scheduler never exits
  PT_SCHEDULE(protothread_sched(&pt_sched));
  // ============================================
  
} // main
// === end  ======================================================

