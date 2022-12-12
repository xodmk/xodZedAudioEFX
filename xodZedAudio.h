/*------------------------------------------------------------------------------------------------*/
/* ___::((xodZedAudio.h))::___

   ___::((JIROBATA Programming Industries))::___
   ___::((ODMK:2018:2022))::___
   ___::((created by e.schei))___

	Purpose: ARM software for xodZedAudio
	Device: zedboard - xc7z020clg484-1
	Tools: Vivado IPI / Vitis IDE 2021.1

	Revision History: May  16, 2017 - initial
	Revision History: June 9,  2018 - odmkSoCFX2 initial
	Revision History: Sept 13, 2018 - odmkSoCFX2 revision 1
	Revision History: Jan  28, 2022 - xodZedAudioMain initial
*/

/*------------------------------------------------------------------------------------------------*/

/*---%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%---*

Zynq xc7z020 Bare-metal ARM software
for system configuration and parameter control of odmkSoCFX2 effects processor

Current Peripheral setup:

4x Pmod Rotary Encoders
1x PmodOLEDrgb 96x64 pixel RGB display
1x Pmod BTN0 (4 button module)

1x USB-Uart (optional)

8x onboard switches (zedboard)
5x onboard buttons (zedboard)


*---%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%---*/

/*------------------------------------------------------------------------------------------------*/

#ifndef __XODZEDAUDIO_H__
#define __XODZEDAUDIO_H__

const double Fclk = 100000000;			// 100 MHz (This clk should match the actual HW clock)
const double Fs = 48000;				// audio sample rate

const float pi = 3.14159265359;

const float clkFs = 100000000.0;		// DDS Frequency


// OLEDrgb_DrawBitmap function delay (approximate! - needs accurate tuning)
// *empirically measured tempo lag + included usleep(5000) func call
const unsigned long oledTempoLag = 50000 + 5000;

// Tunings:
const float mstrVolScale = 0.23;
const u16 maxVolVal = 32767; 					// ap_uint<15> delay_t;

const unsigned int gainWidth = 16;				// gainWidth > 12

// led patterns
const int ledFlash1[] = {24, 36, 66, 129};		// center out LED strobe pattern


/*------------------------------------------------------------------------------------------------*/
// oledRGB

uint8_t rgbUserFont[] = {
	0x00, 0x04, 0x02, 0x1F, 0x02, 0x04, 0x00, 0x00,	// 0x00
	0x0E, 0x1F, 0x15, 0x1F, 0x17, 0x10, 0x1F, 0x0E,	// 0x01
	0x00, 0x1F, 0x11, 0x00, 0x00, 0x11, 0x1F, 0x00,	// 0x02
	0x00, 0x0A, 0x15, 0x11, 0x0A, 0x04, 0x00, 0x00,	// 0x03
    0x07, 0x0C, 0xFA, 0x2F, 0x2F, 0xFA, 0x0C, 0x07  // 0x04
}; // this table defines 5 user characters, although only one is used


/*------------------------------------------------------------------------------------------------*/

struct switch_t {
	bool bypass_sw1;
	bool multiChan_sw2;
	bool wavShaperOn_sw3;
	bool selectCross_sw4;
	bool selectHouse_sw5;
	bool selectSSB_sw6;
	bool lfoUpDn_sw7;
	bool lfoSync_sw8;
};

/*------------------------------------------------------------------------------------------------*/


typedef struct {
	u32 word_0;
	u32 word_1;
	u32 word_2;
} ddlLengthCtrl_t;


//union ddlLengthCtrl_u {
//	u32 swValue;
//	struct sws_type switches;
//};

typedef union {
	u32 ddlLenArr[MAXCHANNELS];
	ddlLengthCtrl_t ddlLenStruct;
} ddlLengthCtrl_u;



/////////////////////////////////////////////////
// ** From HLS - xodEFX_top input Ctrl **
//
// typedef ap_ufixed<16, 1> delayCtrl_t;
// ddlCtrlAll_t ddlCtrlAll
//
// struct ddlCtrlAll_t {
//  	delayCtrl_t wetMix;			=> ddlWetMixFloat
//  	delayCtrl_t feedbackMix;	=> ddlFBGainFloat
//  	delayCtrl_t outputGain;		=> volCtrlDB (?)
// };
//
//ddlLength4CH_t* ddlLengthCtrl
//
// struct ddlLength4CH_t {
//  	delayLen_t ddlLengthCH1;
//  	delayLen_t ddlLengthCH2;
//  	delayLen_t ddlLengthCH3;
//  	delayLen_t ddlLengthCH4;
// };
/////////////////////////////////////////////////


/*------------------------------------------------------------------------------------------------*/
// *---end---*

#endif


