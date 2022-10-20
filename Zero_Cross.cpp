/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// find the tone of the transmit audio stream by detecting the zero cross point

#include <Arduino.h>
#include "Zero_Cross.h"
#include "utility/dspinst.h"


 // static int32_t val1, val2;


void AudioZeroCross1::update(void){ 

    audio_block_t *blk1;
    int16_t *dat1;
    int i;
    static int counts;               // full counts of samples
    static int last;                 // previous audio sample value
    static float fract;              // part of zero cross amount between samples

    // receiving, do nothing
    if( mode == 0 ){
      	blk1 = receiveReadOnly(0);
        if( blk1 ) release( blk1 );
        return;
    }

    // transmitting
    blk1 = receiveReadOnly(0);
    if( blk1 == 0  ){
       //release( blk1 );
       return;
    }
    
    dat1 = blk1->data;
    
    for( i = 0; i < AUDIO_BLOCK_SAMPLES; i++ ){

        ++counts;
        if( last <= 0 && *dat1 > 0 ){
         //  tone_ = (float)counts / 44117.0f ;           // the period
         //  tone_ += fract;                              // add in the missing piece from last calculation
         //  float spread = (float)( *dat1 - last );
         //  fract = ( (float)(*dat1)/spread ) / 44117.0f;
         //  tone_ -= fract;                              // subract the amount we went past zero cross this time
         //  tone_ = 1.0f / tone_;
         
         // same algorithm as above except values are stored as counts instead of time, then converted at the end to freq
           tone_ = (float)counts + fract;               // add the amount past zero last cycle
           float spread = (float)( *dat1 - last );      // last is always negative here
           fract = ( (float)(*dat1)/spread );
           tone_ -= fract;                              // subract the amount we went past zero cross this time
           tone_ = 44117.0f / tone_;          
           
           avail = 1;                                   // new data to return
           counts = 0;
        }
        last = *dat1++;
        
        //dat1 += 1;                                              
    }
    release( blk1 );
}
