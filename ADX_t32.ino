/*
 *   ADX-UNO with Teensy 3.2 cpu
 *      One USB cable provides Audio, Power, CAT control
 *      
 *      Wiring 1 - 1 with UNO wiring
 *      Added RX audio on A2 pin from the ADX-UNO board ( 100 ohm in series )
 *      An op-amp was added for more audio gain.  4.7k in, 100k feedback.
 *      Teensy A2 has the recommended circuit from Audio Library except ground was used instead of AGND
 *      Vin from USB on Teensy is 5 volts, wired to UNO proto card Vin, 5 volts : giving ADX-UNO 5 volts on the PA FETS.
 *      
 *      Audio Library provides audio on USB cable, the Teensy looks like a 16 bit soundcard.
 *      Clock 0 is tx, Clock 1 is rx, using PLLB.   Clock 2 is using PLLA ( init at 8.999 and unused )
 *      
 *      Tap buttons changes mode.
 *      Double tap buttons changes band.
 *      Long press ->  <-  buttons, changes the calibrate value and saves after a 2 minute delay
 *      Long press Tx button is tune mode.
 *      Tap Tx button to cancel tx inhibit on band change.
 *      
 */

#include <Audio.h>
//#include <Wire.h>
#include <i2c_t3.h>
//#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>
#include "Zero_Cross.h"
#include "si5351_init_20m.h"
#include <EEPROM.h>

#define stage(c) Serial.write(c)

 // LED's
 #define TX_LED 13
 #define FT8_LED 12
 #define FT4_LED 11
 #define JS8_LED 10
 #define WSPR_LED 9

 // control
 #define RX_EN 8     // high for RX enable, signal marked on schematic as RXSW

 // switches
 #define SW_DN 2
 #define SW_UP 3
 #define SW_TX 4


#define SI5351 0x60     // I2C address  
 //  starting addresses of phase lock loop registers
#define PLLA 26
#define PLLB 34

 /* switch states */
#define NOTACTIVE 0
#define ARM 1
#define DTDELAY 2
#define FINI 3
#define TAP  4
#define DTAP 5
#define LONGPRESS 6
#define DBOUNCE 60

int sw_state[3] = {NOTACTIVE,NOTACTIVE,NOTACTIVE};   /* state of the switches */


struct BAND {
   int led;       // LED pattern for binary band indication
   int divider;   // si5351 fixed divider per band
   unsigned long freq[4];  // frequencies for ft8,ft4,js8,wspr 
};

#define NUM_BANDS 4
#define NUM_MODES 4

struct BAND bands[NUM_BANDS] = {
 //{8, 208 , 3573000,   3575000,   3578000,   3568600},      // 80
   {4, 104 , 7074000,   7047500,   7078000,   7038600},      // 40
   {3,  74 , 10136000, 10140000,  10130000,  10138700},      // 30
   {2,  54 , 14074000, 14080000,  14078000,  14095600},      // 20
 //{15, 42,  18100000, 18104000,  18104000,  18104600},      // 17  
   {13, 36,  21074000, 21140000,  21078000,  21094600}      // 15
 //{1 , 26,  28074000, 28180000,  28078000,  28124600}       // 10
};

int mode_led[NUM_MODES] = { FT8_LED, FT4_LED, JS8_LED, WSPR_LED };
int band = 2;           // band 2 is 20 meters with the above table, 0 = 40, 1 = 30, 2 = 20, 3 = 15
int mode = 0;           // ft8
int transmitting;
int tx_inhibit = 1;     // no transmit until user verifies filter change, tap tx ( on band change )
int led_update;         // flag to reset a timer from outside the led display code
uint32_t freq;
uint32_t i_tone = 1500;   // transmit tone, it should be somewhere 300 to 2900 hz, will not remain as 1500
float    f_tone;          // fractional part of tx tone  
long cal;
long cal_write_pending;



// GUItool: begin automatically generated code
AudioInputUSB            usb2;           //xy=190,154
AudioInputAnalog         adc1;           //xy=192,224
//AudioFilterBiquad        Tx_Filter;        //xy=358,157   // distorts on full output from wsjt-x
//AudioFilterBiquad        Rx_Filter;        //xy=358,221
AudioAnalyzePeak         peak1;          //xy=496,94
AudioOutputUSB           usb1;           //xy=527,226
//AudioMixer4              Zero_Cross_Det;         //xy=528,154
AudioZeroCross1          Zero_Cross_Det;         //xy=528,154
//AudioConnection          patchCord1(usb2, 0, Tx_Filter, 0);
//AudioConnection          patchCord2(adc1, Rx_Filter);
//AudioConnection          patchCord3(Tx_Filter, 0, Zero_Cross_Det, 0);
AudioConnection          patchCord3(usb2, 0, Zero_Cross_Det, 0);
AudioConnection          patchCord4(usb2, peak1);
AudioConnection          patchCord5(adc1, 0, usb1, 0);
AudioConnection          patchCord6(adc1, 0, usb1, 1);
// GUItool: end automatically generated code


void setup() {

   Wire.begin();

   pinMode( TX_LED, OUTPUT );
   pinMode( FT8_LED, OUTPUT );
   pinMode( FT4_LED, OUTPUT );
   pinMode( JS8_LED, OUTPUT );
   pinMode( WSPR_LED, OUTPUT );
   pinMode( RX_EN, OUTPUT );
   pinMode( SW_DN, INPUT );
   pinMode( SW_UP, INPUT );
   pinMode( SW_TX, INPUT );

   EEPROM.get(0,cal);
   if( cal > 10000 || cal < -10000 ) cal = 0;    // out of range
   if( cal == -1 ) cal = 0;                      // blank eeprom value

//0.54119610  butterworth cascade Q values for two sections biquads
//1.3065630
   AudioNoInterrupts();
   AudioMemory(40);
   // tx filter seems overdriven when wsjt is at full power
  // Tx_Filter.setHighpass( 0, 300, 0.54119610 );    // keep any noise out of the zero cross detector
  // Tx_Filter.setHighpass( 1, 300, 1.3065630 );
  // Tx_Filter.setLowpass( 2, 2900, 0.54119610 );
  // Tx_Filter.setLowpass( 3, 2900, 1.3065630 );
   
  // Rx_Filter.setHighpass( 0, 300, 0.54119610 );    // receive bandwidth
  // Rx_Filter.setHighpass( 1, 300, 1.3065630 );
  // Rx_Filter.setLowpass( 2, 2900, 0.54119610 );
  // Rx_Filter.setLowpass( 3, 2900, 1.3065630 );
   AudioInterrupts();
   
   Serial.begin(1200);
   si5351_init();
   delay(10);
   // set some divider registers that will never change
   for(int i = 0; i < 3; ++i ){
     i2cd(SI5351,42+8*i,0);
     i2cd(SI5351,43+8*i,1);
     i2cd(SI5351,47+8*i,0);
     i2cd(SI5351,48+8*i,0);
     i2cd(SI5351,49+8*i,0);
   }
   freq = bands[band].freq[mode];
   si_pll_x( PLLB, freq, bands[band].divider, 0.0 );    // clock 0 and 1 on same pll
   si_load_divider( bands[band].divider, 0, 0 );
   si_load_divider( bands[band].divider, 1, 1 );        // reset pll for clocks
   delay( 10 );
   i2cd( SI5351, 177, 0xA0 );         // another PLLA PLLB soft reset needed?
   i2cd( SI5351, 3, 0xff ^ 2 );       // enable rx clock, clock 1
     //i2cd( SI5351, 3, 0xff ^ 7 );     // !!! all clocks on, testing  

   digitalWrite( RX_EN, HIGH );


}

// what needs to change to enter tx mode
void tx(){

  if( tx_inhibit ) return;
  i2cd( SI5351, 3, 0xff );         // clocks off
  digitalWrite( RX_EN, LOW );      // rx disable switch
  transmitting = 1;
  Zero_Cross_Det.setmode( 1 );
  si_pll_x(PLLB, freq + i_tone, bands[band].divider, 0.0 );      // tune and start with prev tone
  i2cd( SI5351, 3, 0xff ^ 1 );     // clock 0 on, tx on tones detected
  
}

// what needs to change to enter rx mode
void rx(){

   i2cd( SI5351, 3, 0xff );      // turn off clocks
   transmitting = 0;
   Zero_Cross_Det.setmode( 0 );
   band_change3();               // back to rx clock frequency
   i2cd( SI5351, 3, 0xff ^ 2 );  // rx clock on
   digitalWrite( RX_EN, HIGH );  // rx enabled
  
}

void loop() {
static unsigned long tm;
static int db;  

  led_control();    // the LED's control themselves depending upon mode, band, etc.
  radio_control();

  // 1ms routines
  if( tm != millis() ){
     tm = millis();
     button_state();    // switches latch on until they are processed
     mode_change();
     band_change();
     tx_button();
     vox_check();
     send_tone();
     ++db;              // debug timer

     if( cal_write_pending){
        if( --cal_write_pending == 0 ) save_cal();
     }
  }

  if( db == 10 ){     // timed debug prints
    db = 0;
    //if( Zero_Cross_Det.available() ) Serial.println( Zero_Cross_Det.read() - 1500 );   // !!! debug consumes avail data
  }

}

#define VOX_MIN 0.2
#define VOX_MAX 0.3

void vox_check(){
static float val;

   if( peak1.available() == 0 ) return;
   val += peak1.read();
   val /= 2.0;

   if( transmitting == 0 && val > VOX_MAX ) tx();
   if( transmitting && val < VOX_MIN && sw_state[1] != LONGPRESS ) rx();
   
}

void send_tone(){
float val;
   if( transmitting == 0 || sw_state[1] == LONGPRESS ) return;     // not active
   // check if update i_tone f_tone available and send it
   if( Zero_Cross_Det.available() ){
       val = Zero_Cross_Det.read();
       i_tone = (uint32_t)val;                     // integer part
       f_tone = val - (float)i_tone;               // fractional part
       si_pll_x(PLLB, freq + i_tone, bands[band].divider, f_tone );      // set new frequency 
   }
}

void tx_button(){

   if( sw_state[1] < TAP ) return;
   if( sw_state[1] == TAP && tx_inhibit ){
      tx_inhibit = 0;
   }
   
   if( sw_state[1] == LONGPRESS ){           // Tune mode
      if( digitalRead( SW_TX ) == LOW ){     // Tune mode still active
         if( transmitting == 0 ) tx();       // start transmitting
         return;                             // stall switch state in LONGPRESS mode, skip changing state to FINI        
      }
      else  rx();                            // finished
   }
   
   // double tap unused for this switch
   sw_state[1] = FINI;                       // done processing this switch, tap,double tap, long press
}

void mode_change(){

    if( sw_state[0] == TAP ){        // buttons work in reverse of how I set up the data struct, fixed here
        mode_change2( 1 );
        sw_state[0] = FINI;
    }
    if( sw_state[2] == TAP ){
        mode_change2( -1 );
        sw_state[2] = FINI;
    }

    // DTAP handled in another function
    // LONGPRESS changes cal, moves waterfall signal in direction of button arrow
    if( sw_state[0] == LONGPRESS ) adj_cal( -10L ), sw_state[0] = FINI;
    if( sw_state[2] == LONGPRESS ) adj_cal( 10L ), sw_state[2] = FINI;
}

void save_cal(){            // save the calibrate value

    EEPROM.put(0, cal );  
}

void adj_cal( long val ){

   cal += val;
   cal_write_pending = 120000;    // two minutes wait before eeprom write
   band_change3();                // effect the change for the user
   Serial.print("Calibrate value ");
   Serial.println(cal);
}

void mode_change2( int val ){
int tmp;
  
    tmp = mode + val;
    if( tmp < 0 || tmp >= NUM_MODES ) return;
    mode = tmp;

    // freq = bands[band].freq[mode];   redundant
    band_change3();
    led_update = 1;
}

void band_change(){
    if( sw_state[0] == DTAP ){
        band_change2( -1 );
        sw_state[0] = FINI;
    }
    if( sw_state[2] == DTAP ){
        band_change2 ( 1 );
        sw_state[2] = FINI;
    }
}

void band_change2( int val ){
int tmp;

     tmp = band + val;
     if( tmp < 0 || tmp >= NUM_BANDS ) return;
     tx_inhibit = 1;                            // no transmit until user says correct lowpass filter in place
     band = tmp;                                // by tapping tx button
     
    // freq = bands[band].freq[mode];
     led_update = 1;
     band_change3();
}

void band_change3(){

   if( transmitting ) return;
   freq = bands[band].freq[mode];
   si_pll_x( PLLB, freq, bands[band].divider, 0.0 );    // clock 0 and 1 on same pll
   si_load_divider( bands[band].divider, 0, 0 );
   si_load_divider( bands[band].divider, 1, 1 );        // reset pll for clocks 
   delay( 10 );
   i2cd( SI5351, 177, 0xA0 );         // another PLLA PLLB soft reset needed?
 
}


int read_buttons(){
int val;

   val = 0;
   if( digitalRead( SW_DN ) == LOW ) val |= 1;
   if( digitalRead( SW_TX ) == LOW ) val |= 2;
   if( digitalRead( SW_UP ) == LOW ) val |= 4;
   return val;
}

// read switches, need level detect on TX switch or latch on the transmitter
void button_state(){ /* state machine running at 1ms rate */
int sw,st,i;
static int press_,nopress;

      sw = read_buttons();                 // only one button detected at a time    
      if( sw ) ++press_, nopress = 0;
      else ++nopress, press_= 0;
      
      /* switch state machine */
      for( i= 0; i < 3; ++i ){
         st= sw_state[i];      /* temp the array value to save typing */

         if( st == NOTACTIVE && (sw & 0x1) && press_ >= DBOUNCE ) st= ARM;
         if( st == FINI && nopress >= DBOUNCE ) st = NOTACTIVE;   /* reset state */

         /* double tap detect */
         if( st == ARM && nopress >= DBOUNCE/2 )     st= DTDELAY;
         if( st == ARM && (sw & 0x1 ) && press_ >= 10*DBOUNCE )  st= LONGPRESS; 
         if( st == DTDELAY && nopress >= 4*DBOUNCE ) st= TAP;
         if( st == DTDELAY && ( sw & 0x1 ) && press_ >= DBOUNCE )   st= DTAP;
         
         sw_state[i]= st;      
         sw >>= 1;   /* next switch */
      }        
}


void led_control(){       // LED's just take care of themselves, blink out the band or show mode otherwise
static unsigned long tm;  // tx LED flashes tx_inhibit after band change
static int state;
static int tx_led;
int i;

   if( led_update == 0 && millis() - tm < 200 ) return;
   tm = millis();
   if( led_update ) led_update = 0, state = 0;         // reset the timer when something has changed
   ++state;

//Serial.print( sw_state[0] );  Serial.write(' ');
//Serial.print( sw_state[1] );  Serial.write(' ');
//Serial.println( sw_state[2] );

   switch( state ){
      case 1:  case 3: case 5:         // blink band on
         if( bands[band].led & 1 ) digitalWrite( FT8_LED, HIGH );
         if( bands[band].led & 2 ) digitalWrite( FT4_LED, HIGH );
         if( bands[band].led & 4 ) digitalWrite( JS8_LED, HIGH );
         if( bands[band].led & 8 ) digitalWrite( WSPR_LED, HIGH );
      break;
      case 2:  case 4: case 6:         // blink band off
         for( i = 0; i < NUM_MODES; ++i ) digitalWrite( mode_led[i], LOW );
      break;
      case 7:                          // mode on
         digitalWrite( mode_led[mode], HIGH );
      break;
      case 32:  digitalWrite( mode_led[mode], LOW );  state = 0;  break;
   }

   // TX_LED control
   if( tx_inhibit ){  // user must manually check for proper lowpass filter, tap tx switch to end flashing
      if( state > 19 ) digitalWrite( TX_LED, digitalRead( TX_LED ) ^ 1 );
      tx_led = 1;         // so code below turns LED off when done
   }
   else{
      if( transmitting && tx_led == 0 ) tx_led = 1, digitalWrite( TX_LED, HIGH );
      if( transmitting == 0 && tx_led ) tx_led = 0, digitalWrite( TX_LED, LOW );
   }
}


/*****************************************************************************************/
// TenTec Argonaut V CAT emulation

//int un_stage(){    /* send a char on serial */
//char c;

//   if( stg_in == stg_out ) return 0;
//   c = stg_buf[stg_out++];
//   stg_out &= ( STQUESIZE - 1);
//   Serial.write(c);
//   return 1;
//}

#define CMDLEN 20
char command[CMDLEN];
uint8_t vfo = 'A';

void radio_control() {
static int expect_len = 0;
static int len = 0;
static char cmd;

char c;
int done;

    if (Serial.available() == 0) return;
    
    done = 0;
    while( Serial.available() ){
       c = Serial.read();
       command[len] = c;
       if(++len >= CMDLEN ) len= 0;  /* something wrong */
       if( len == 1 ) cmd = c;       /* first char */
       /* sync ok ? */
       if( cmd == '?' || cmd == '*' || cmd == '#' );  /* ok */
       else{
          len= 0;
          return;
       }
       if( len == 2  && cmd == '*' ) expect_len = lookup_len(c);    /* for binary data on the link */       
       if( (expect_len == 0 &&  c == '\r') || (len == expect_len) ){
         done = 1;
         break;   
       }
    }
    
    if( done == 0 ) return;  /* command not complete yet */
        
    if( cmd == '?' ){
      get_cmd();
     // operate_mode = CAT_MODE;            // switch modes on query cat command
     // if( wwvb_quiet < 2 ) ++wwvb_quiet;  // only one CAT command enables wwvb logging, 2nd or more turns it off
     // mode_display();
    }
    if( cmd == '*' )  set_cmd();
    if( cmd == '#' ){
        pnd_cmd(); 
       // if( wwvb_quiet < 2 ) ++wwvb_quiet;  // allow FRAME mode and the serial logging at the same time
    }

 /* prepare for next command */
   len = expect_len= 0;
   stage('G');       /* they are all good commands */
   stage('\r');

}

int lookup_len(char cmd2){     /* just need the length of the command */
int len;

   
   switch(cmd2){     /* get length of argument */
    case 'X': len = 0; break;
    case 'A':
    case 'B': len = 4; break;
    case 'E':
    case 'P':
    case 'M': len = 2; break;
    default:  len = 1; break ;
   }
   
   return len+3;     /* add in *A and cr on the end */
}

void set_cmd(){
char cmd2;
unsigned long val4;

   cmd2 = command[1];
   switch(cmd2){
    case 'X':   stage_str("RADIO START"); stage('\r'); break; 
    case 'O':   /* split */ 
    break;
    case 'A':   // set frequency
    case 'B':
       val4 = get_long();
       cat_qsy(val4);  
    break;
    case 'E':
       if( command[2] == 'V' ) vfo = command[3];
    break;
    case 'W':    /* bandwidth */
    break;
    case 'K':    /* keying speed */
    break;
    case 'T':    /* added tuning rate as a command */
    break;
    case 'M':
      // int i = command[2] - '0';          // FM will map to DIGI
      // mode_change(i);
      // status_display();
    break;       
   }  /* end switch */   
}

void cat_qsy( unsigned long val ){
int i,j;

  if( transmitting ) return;

  if( abs((long)freq - (long)val ) > 500000 ) tx_inhibit = 1;   // band change
  freq = val;
  
  // see if can find freq in our tables
  for( i = 0; i < NUM_BANDS; ++i ){
    for( j = 0; j < NUM_MODES; ++j ){
       if( val == bands[i].freq[j] ) band = i, mode = j;
    }
  }

  // frequency changed, mode change or band change handle same
  // may fail if moved frequency out of band and not one of the frequencies in the table
  // as the divider may not be in range of the PLL.  Don't think will try to correct
  // as it is supposed to be a 4 mode radio.

   si_pll_x( PLLB, freq, bands[band].divider, 0.0 );    // clock 0 and 1 on same pll
   si_load_divider( bands[band].divider, 0, 0 );
   si_load_divider( bands[band].divider, 1, 1 );        // reset pll for clocks 
   delay( 10 );
   i2cd( SI5351, 177, 0xA0 );         // another PLLA PLLB soft reset needed?
  
}

void get_cmd(){
char cmd2;
long arg;
int len; // i;

   cmd2 = command[1];   
   stage(cmd2);
   switch(cmd2){
    case 'A':     // get frequency
    case 'B': 
      arg = freq;
      stage_long(arg);
    break;
    case 'V':   /* version */
      stage_str("ER 1010-516");
    break;
    case 'W':          /* receive bandwidth */
       stage(30);
    break;
    case 'M':          /* mode. 11 is USB USB  ( 3 is CW ) vfo A, vfo B */
     // i = mode;
     // if( i > 4 ) i &= 3;                          // report UDSB, LDSB as USB, LSB.  DIGI reports as FM
     // i = i + '0';
      stage(3); stage(3);
    break;
    case 'O':          /* split */   
       stage(0);
    break;
    case 'P':         /*  passband slider */
       stage_int( 3000 );
    break;
    case 'T':         /* added tuning rate command */
    break;   
    case 'E':         /* vfo mode */
      stage('V');
      stage(vfo);
    break;
    case 'S':         /* signal strength */
       stage(7);
       stage(0);
    break;
    case 'C':      // transmitting status 
       stage(0);
       if( transmitting ) stage(1);
       else stage(0);
    break;
    case 'K':   /* wpm on noise blanker slider */
       stage( 15 - 10 );
    break;   
    default:           /* send zeros for unimplemented commands */
       len= lookup_len(cmd2) - 3;
       while( len-- ) stage(0);  
    break;    
   }
  
   stage('\r');  
}


void stage_str( String st ){
unsigned int i;
char c;

  for( i = 0; i < st.length(); ++i ){
     c = st.charAt( i );
     stage(c);
  }    
}

void stage_long( long val ){
unsigned char c;
   
   c = val >> 24;
   stage(c);
   c = val >> 16;
   stage(c);
   c = val >> 8;
   stage(c);
   c = val;
   stage(c);
}


unsigned long get_long(){
union{
  unsigned long v;
  unsigned char ch[4];
}val;
int i;

  for( i = 0; i < 4; ++i) val.ch[i] = command[5-i]; // or i+2 for other endian
  return val.v;
}

void stage_int( int val ){
unsigned char c;
   c = val >> 8;
   stage(c);
   c = val;
   stage(c);
}

void stage_num( int val ){   /* send number in ascii */
char buf[35];
char c;
int i;

   itoa( val, buf, 10 );
   i= 0;
   while( (c = buf[i++]) ) stage(c);  
}

void pnd_cmd(){
char cmd2;
   
   cmd2 = command[1];      // !!! using vox or CAT for tx on/off
   switch(cmd2){
    // case '0':  rx();  break;    // enter rx mode
    // case '1':  tx();  break;    // TX
   }

}

/********************* end Argo V CAT ******************************/


// some of my early SI5251 code, inits all the registers from a table
// load the table produced by clock builder into the clock chip
void si5351_init(){
 
  int i;
  
  i2cd( SI5351, 3, 0xff );  // disable outputs
  for( i = 16; i < 24; ++i ) i2cd( SI5351, i, 0x80 );  // power down output drivers
  
  for( i= 15; i <= 92; i++ ){                        // load table values 
    i2cd( SI5351, i, si_clk_initial_values[ 2*i + 1 ]); 
  }
  for( i= 149; i <= 170; i++ ){
    i2cd( SI5351, i, si_clk_initial_values[ 2*i + 1 ]);
  }
  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset
  // i2cd( SI5351, 3, 0xff ^ 0x5 );     // don't enable this prelim setup, CLK0 and CLK2 outputs

  // table assigns clock 0 to PLLB
  // assign clock 1 to PLLB with 2ma drive
  i2cd( SI5351, 17, 0x6c );
  // disable clock 2
  i2cd( SI5351, 18, 0x80 );    // table has 0x4f, assigned to PLLA, so that is the setting if want a cal signal
}


//    I2C handler for blind writes  
//    multiple writes indirect via pointer to char array
//void i2ci( unsigned char addr, unsigned char reg, unsigned char *p, int count ){

//    Wire.beginTransmission( addr );
//    Wire.send( reg );                      // starting register or 1st data if no registers in device
//    while( count-- )  Wire.send( *p++ );
//    Wire.endTransmission();
//}
//    single write direct
void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){

    Wire.beginTransmission( addr );
    Wire.send( (int) reg );                    // register or 1st data byte if no registers in device
    Wire.send( (int) dat );
    Wire.endTransmission();
}

// general idea of this code from some code written by Hans Summers
// modified to remove redundant division, and merge two functions into one
// load a new frequency into PLL A or B 
// the output divider is fixed per band in use and not recalculated
void  si_pll_x(unsigned char pll, uint64_t freq, int out_divider, float fract ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t clock_freq = (uint64_t)(25001740L + cal);

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;


//  for fractional part, just add fraction * out_divider to the pll_freq
//  as a full increment of one will add full value of out_divider to pll_freq
//  only works for positive frequency changes

   c = 1000000;     // max 1048575
   pll_freq = freq * (uint64_t)out_divider;
   fract = fract * (float)out_divider;
   if( fract > 0.0 ) pll_freq += (uint64_t)fract;
   a = pll_freq / (clock_freq);
   r = pll_freq - a * (clock_freq);
   b = ( c * r ) / (clock_freq);
   bc128 =  (128 * r)/ (clock_freq);
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ) P2 = 0;        // ? avoid negative numbers 
   P3 = c;

   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
   
 //  i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset  
}


// init has loaded other registers with the clock builder values to allow this simplified code to work
void si_load_divider( int val, int clk , int rst){
 
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, (val >> 16 ) & 3 );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xA0 );         // PLLA PLLB soft reset needed?
}
