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


#include "xxodefx_top.h"


/*------------------------------------------------------------------------------------------------*/
/* (((User Definitions))) */

#define MAXCHANNELS 4
#define MAXDELAY 16384
#define LFOCHANNELS 1

/*------------------------------------------------------------------------------------------------*/
/* (((Definitions))) */

#define bool u8
#define true 1
#define false 0

//#define printf xil_printf							/* smaller, optimised printf !!?doesn't allow for %d variables?!?*/

#define BTN_CH_MASK		XGPIO_IR_CH1_MASK
#define BTN0_CH_MASK	XGPIO_IR_CH1_MASK
#define SWS_CH_MASK		XGPIO_IR_CH2_MASK
#define TIMER_INT		XGPIO_IR_CH1_MASK
#define ROTARY_INT		XGPIO_IR_CH1_MASK

//#define LED 0xC3									/* Initial LED value - XX0000XX */
#define LED 0x01									/* Initial LED value - XX0000XX */
#define LED_DELAY		15600000					/* Software delay length */
#define LED_CHANNEL		1							/* GPIO port for LEDs */
#define SW_CHANNEL		2							/* GPIO port for SWs */

/*------------------------------------------------------------------------------------------------*/
/* Device IDs */

//#define Snoop Control Unit (interrupt controller)
#define INTC_DEVICE_ID 				XPAR_PS7_SCUGIC_0_DEVICE_ID

//#define Timer
//#define TMRCTR_TIMER_ID				XPAR_AXI_TIMER_0_DEVICE_ID
//#define INTC_TIMER_INTERRUPT_ID	  XPAR_FABRIC_AXI_TIMER_0_INTERRUPT_INTR

//#define timer0 DEVICE_ID
//#define GPIO_TIMER0_ID 				XPAR_AXIGPIO_TIMER0_DEVICE_ID

//#define AXICTRL_DEVICE_ID (built-in)
#define AXI_CTRL_ID  				XPAR_XODEFX_TOP_DEVICE_ID

//#define BTNS_DEVICE_ID (built-in)
#define GPIO_BTNS_ID  				XPAR_AXIGPIO_BTNS_DEVICE_ID
#define INTC_BTNS_INTERRUPT_ID 		XPAR_FABRIC_AXIGPIO_BTNS_IP2INTC_IRPT_INTR

//#define BTN0_DEVICE_ID (Pmod)
//#define GPIO_BTN0x1_ID  			XPAR_AXIGPIO13_BTN0_DEVICE_ID
//#define INTC_BTN0x1_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO13_BTN0_IP2INTC_IRPT_INTR


//#define ROTARY1_DEVICE_ID
#define GPIO_ROTARY1_ID				XPAR_AXIGPIO_ROT1_DEVICE_ID
#define INTC_ROTARY1_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO_ROT1_IP2INTC_IRPT_INTR

//#define ROTARY2_DEVICE_ID
#define GPIO_ROTARY2_ID				XPAR_AXIGPIO_ROT2_DEVICE_ID
#define INTC_ROTARY2_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO_ROT2_IP2INTC_IRPT_INTR

//#define ROTARY3_DEVICE_ID
#define GPIO_ROTARY3_ID				XPAR_AXIGPIO_ROT3_DEVICE_ID
#define INTC_ROTARY3_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO_ROT3_IP2INTC_IRPT_INTR

//#define ROTARY4_DEVICE_ID
#define GPIO_ROTARY4_ID				XPAR_AXIGPIO_ROT4_DEVICE_ID
#define INTC_ROTARY4_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO_ROT4_IP2INTC_IRPT_INTR

//#define LEDS_SWS_DEVICE_ID
// DUAL: ?LSB[7:0]=LED, MSB[7:0]=SW
#define GPIO_LED_SW_ID  			XPAR_AXIGPIO_LEDSW_DEVICE_ID

//#define DDLCTRL_DEVICE_ID
#define GPIO_DDLCTRL_ID 			XPAR_AXIGPIO_DDLCTRL_DEVICE_ID

//#define VOLCTRL_DEVICE_ID
#define GPIO_VOLWSGAIN_ID 			XPAR_AXIGPIO_VOLWSGAIN_DEVICE_ID

//#define OSC4T Freq2 & PWM DEVICE_ID (dual channel)
// DUAL: LSB[27:0]=DDS FREQ2, MSB[27:0]=PWM FREQ
//#define GPIO_OSCF2_PWM_ID 			XPAR_AXIGPIO7_FREQ2PWM_DEVICE_ID

//#define AUDIOMUX_DEVICE_ID
//#define GPIO_AUDIOMUX_ID 			XPAR_AXIGPIO9_MUXCTRL_DEVICE_ID

//#define MHL_ALPHA_K_DEVICE_ID
// DUAL: LSB[27:0]=ALPHA (G - cutoff), MSB[27:0]=K (resonance)
//#define GPIO_MHL_ALPHA_K_ID 		XPAR_AXIGPIO10_ALPHAK_DEVICE_ID

//#define MHL_PARAM1_DEVICE_ID
// DUAL: LSB[23:0]=ALPHA0, MSB[23:0]=Beta1
//#define GPIO_MHL_PARAM1_ID 			XPAR_AXIGPIO11_FPARAMS1_DEVICE_ID

//#define MHL_PARAM2_DEVICE_ID
// DUAL: LSB[23:0]=Beta2, MSB[23:0]=Beta3
//#define GPIO_MHL_PARAM2_ID 			XPAR_AXIGPIO12_FPARAMS2_DEVICE_ID

//#define SEMALPHA_DEVICE_ID
// DUAL: LSB[23:0]=alpha0, MSB[23:0]=alpha
//#define GPIO_SEMALPHA_ID 			XPAR_AXIGPIO16_SEMALPHA_DEVICE_ID

//#define SEMRHO_DEVICE_ID
//#define GPIO_SEMRHO_ID 				XPAR_AXIGPIO17_SEMRHO_DEVICE_ID


/*------------------------------------------------------------------------------------------------*/


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


struct ddlLengthCtrl_t {
	bool ddlCenterLock;
	double ddlLengthBase;
	double ddlTilt;
};


//typedef struct {
//	u32 word_0;
//	u32 word_1;
//	u32 word_2;
//} ddlLengthCtrl_t;

struct ddlLength4CHAXI_t {
	u32 ddlLengthCH1;
	u32 ddlLengthCH2;
	u32 ddlLengthCH3;
};

typedef union {
	u32 ddlLenArr[3];		// 3 x 24bits for 4CH delay
	XXodefx_top_Ddllengthctrl ddlLen4CHAXI;
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


// *-----------------------------------------------------------------------------------* //
///// Single Side-band Modulator types /////////////////////

//typedef ap_fixed<dataWidth, 2> ssbData_t;
//typedef ap_fixed<dataWidth+8, 6> ssbData_t;
//typedef ap_fixed<dataWidth + extTop + extBottom, 2 + extTop> ssbData_t;

// ** same as data_t

struct ssbModFreqCtrl_t {
	bool ssbCenterLock;
	double ssbFreqBase;
	double ssbTilt;
};

struct ddsFreq4CHAXI_t {
	u32 ddsFreqCtrlCH1;
	u32 ddsFreqCtrlCH2;
	u32 ddsFreqCtrlCH3;
	u32 ddsFreqCtrlCH4;
};


//typedef struct {
//	u32 word_0;
//	u32 word_1;
//	u32 word_2;
//} ddlLengthCtrl_t;


union ssbModFreqCtrl_u {
	u32 ssbFreqArr[4];		// 3 x 24bits for 4CH delay
	XXodefx_top_Ssbmodfreqctrl ssbFreq4CHAXI;
};


union sws_u {
	u32 swValue;
	struct switch_t switches;
};

// *-----------------------------------------------------------------------------------* //
///// DDS Typedefs /////////////////////

// ** copied from xodSSBMod.h

//const double Fs = 100000000;	// FPGA clock frequency

const int ddsDataWidth = 24;

// *** limitation for 32 bit UI frequency control ***
// For Fclk = 100MHz internal clock step - swap 100MHz Fclk for 48000 Fs when calculating step (SW func)
// (-1499 * (2**36 / 48000) = -2146051992  =>  2**31 - 1 = 2147483647 (1500 overflows 32 bits)
// +/-1499 Hz = max/min freq for 32 bit CTRL UI
// (or - FIXIT - implement DDS freq equation -> requires 'power of 2' in FPGA)
const int ddsControlWidth = 32;
const int ddsPhaseWidth = 36;


// *-----------------------------------------------------------------------------------* //


/*------------------------------------------------------------------------------------------------*/
// *---end---*

#endif


