/*------------------------------------------------------------------------------------------------*/
/* ___::((xodZedAudioMain.c))::___

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


HLS Function call:
void xodEFX_test(const std::string &efxTestName, const std::string &outDir, const int simLength,
		                switch_t switches, lfoCtrlAll_t lfoCTRL, ddlLengthCtrl_t ddlLengthCTRL,
						ddlCtrlAll_t ddlCTRL, ssbModFreqCtrl_t ssbModFreqCTRL, wavshaperCtrl_t wavShaperGain,
						AudioFile<float>* wavMain, AudioFile<float>* wavSub,
						stereoData_t x_in[], stereoData_t y_out[])



const std::string &efxTestName 		-
const std::string &outDir 			-
const int simLength 				-
switch_t switches 					-
lfoCtrlAll_t lfoCTRL 				-
ddlLengthCtrl_t ddlLengthCTRL 		-
ddlCtrlAll_t ddlCTRL 				-
ssbModFreqCtrl_t ssbModFreqCTRL 	-
wavshaperCtrl_t wavShaperGain 		-
AudioFile<float>* wavMain 			-
AudioFile<float>* wavSub 			-
stereoData_t x_in[] 				-
stereoData_t y_out[] 				-




*---%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%---*/

/*------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "platform.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xscugic.h"
//#include "xtmrctr.h"
#include "xil_exception.h"
#include "xstatus.h"
#include "sleep.h"
#include "xil_types.h"
#include "xil_cache.h"
#include "xparameters.h"
//#include "xil_printf.h"
#include "xxodefx_top.h"
#include "xodZedAudio.h"

#include "oledrgb_pattern1.h"
#include "PmodOLEDrgb.h"


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
/* instantiate objects */

XScuGic INTCInst;
//XTmrCtr TIMERInst;
PmodOLEDrgb oledrgb;
XGpio LEDSWSInst;
XGpio BTNInst;
//XGpio BTN0x1Inst;
XGpio AUDIOMUXInst;
XGpio ROTARY1Inst;
XGpio ROTARY2Inst;
XGpio ROTARY3Inst;
XGpio ROTARY4Inst;
XGpio DDLCTRLInst;
XGpio VOLWSGAINInst;

//XGpio OSCF2PWMStepInst;
//XGpio MHLALPHAKInst;
//XGpio MHLCTRL1Inst;
//XGpio MHLCTRL2Inst;
//XGpio SEMCTRL1Inst;
//XGpio SEMCTRL2Inst;

XXodefx_top XodAXICtrl;


/*------------------------------------------------------------------------------------------------*/
/* prototype EFX controls */


const std::string efxTestName = "mumra_was_here";
// const std::string &outDir 			-


///// DDL controls /////
const float ddlWetDryScale = 0.023;
//const u32 ddlLengthScale = 56<<8;
const u32 ddlLengthScaleBase = 128;				// default base scale (pre-scaled scaler)
const float ddlFbGainScale = 0.023;
const unsigned int ddlCtrlBwidth = 16;			// ap_ufixed<16,8>
const u32 maxDlyLength = 65535<<8; 				// ap_uint<16,8> delay_t;


///// DDS controls /////
const int ddsPhaseWidth = 36;


// // waveshaper Gain ctrl: ap_ufixed<16,10>
// //Atan exponent" - range {0.1 - 1024.0}, initial value = 1.0
const float wsGainScale = 0.46;

//const int oscFreqScale = 560;
//const int lfoFreqScale = 1;


//const int mooghlCutoffScale = 111;
//const float filtVolScale = 2.7;
//const float mooghlResScale = 0.007;

//const int semCutoffScale = 111;
//const float semQScale = 0.007;


/*------------------------------------------------------------------------------------------------*/
/* (((Static Variables))) */

static int INT_CH_MASK = XGPIO_IR_CH1_MASK;
static int INTC_X_INTERRUPT_ID = INTC_BTNS_INTERRUPT_ID;	// arbitrarily defaults to button interrupt ID

static float BPM = 133.0;	// Default BPM
static unsigned long uSECPERBEAT = 451128;		// 1000000*60/BPM - default initial value assuming 133 bpm

// system state variables
static int sysmode = 0;							// 0=>OLED info ; 1=>OLED imgRotate ; 2=>?
//static u8 CH1_outMux_prev = 0;
static bool trigOnce = 1;
static int ledState = 0;						// used for LED manipulation function

// button variables
static int btnValue;
//static int btn0x1_value;

// rotary encoder variables
static u8 fader1MuxSel = 0;
static u8 fader2MuxSel = 1;
static u8 fader3MuxSel = 2;
static u8 fader4MuxSel = 3;
static float fader1Lvl = 0;
static float fader2Lvl = 0;
static float fader3Lvl = 0;
static float fader4Lvl = 0;
static u8 rotary1_AB = 0;
static u8 rotary2_AB = 0;
static u8 rotary3_AB = 0;
static u8 rotary4_AB = 0;


static float volCtrlDB = 25.2;		// Attenuation amount: 0 DB = full volume, -140 DB ~ -inf
static u32 volCtrlVal = 3601;		// 16b exponential audio out volume control val { 0 - 32767 }, default = 16422 (-6 DB)

static float delayBase = 777;
static float freqBase = 56.6;

static float ddlTilt = 0;
static float ssbTilt = 0;

static bool ddlCenterLock = 1;
static bool ssbCenterLock = 1;

//static union ddlLengthCtrl_u ddlLenCtrlUnion;


//typedef struct {
//	    u32 word_0;
//	    u32 word_1;
//	    u32 word_2;
//	} XXodefx_top_Ddllengthctrl;
static XXodefx_top_Ddllengthctrl* ddlLenCtrlAxiBus;

//		typedef struct {
//		    u32 word_0;
//		    u32 word_1;
//		    u32 word_2;
//		    u32 word_3;
//		} XXodefx_top_Ssbmodfreqctrl;
static XXodefx_top_Ssbmodfreqctrl* ssbModPhaseAxiBus;

/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
// define DDL CTRL parameters


static float ddlWetMixFloat = 0.5;
static float ddlFBGainFloat = 0;



//static u32 ddlLengthShift;
static u32 delayLengthTmp = 450688;		// range = 0 - ap_ufixed<16,8> (ex. 1760.5 << 8 = 450688)

static u32 ddlLengthScale = 1;
static u32 ddlTiltScale = 0.1;

// DDL CH1
struct DDLCTRL1_type {
	u16 wetMix: 16;						// range = 0.0:1.0 <-> ap_ufixed<16, 1> delayCtrl_t
	u16 feedbackGain: 16;				// range = 0.0:1.0 <-> ap_ufixed<16, 1> delayCtrl_t
};

union DDLCTRL1_u {
	u32 DDLCTRL_CH1Val;
	struct DDLCTRL1_type DDLCTRL1;
};

static union DDLCTRL1_u DDLCTRL1Union;


// DDL CH2
struct DDLCTRL2_type {
	u32 delayLength: 24;			// range = 0 - ap_ufixed<16,8> (ex. 1760.5 << 8 = 450688)
};

union DDLCTRL2_u {
	u32 DDLCTRL_CH2Val;
	struct DDLCTRL2_type DDLCTRL2;
};

static union DDLCTRL2_u DDLCTRL2Union;

/*------------------------------------------------------------------------------------------------*/

//static float mooghlCutoff = 24860.00;			// modify to Hz values (currently raw value)
//static float mooghlRes = 0.0;
//
//static float semCutoff = 24860.00;			    // modify to Hz values (currently raw value)
//static float semQ = 1.0;						// Q range {0.5:10}
//static float semQscaled = 0.0;

static float wsGainFloat = 1.7;
static u32 wsGainVal = 1.7 * 64;	// waveshaper Gain ctrl: ap_ufixed<16,10> - range { 0.25 - 1023 }

/*------------------------------------------------------------------------------------------------*/


union sws_u {
	u32 swValue;
	struct switch_t switches;
};

static union sws_u swsUnion;

// CH1_outMux: {bank0_sw1&2&3} = CH1 output audio MUX sel
// 0 = Dry, 1 = DDL, 2 = SSB, 3 = Filter, 4 = WaveShaper

// CH2_outMux: {bank0_sw4&5} = CH2 output audio MUX sel
// 0 = DDL, 1 = SSB, 2 = WaveShaper, 3 = Filter

// wsInMuxSel: {bank0_sw6&7} = Waveshaper Input Mux sel
// 0 = Dry, 1 = Filter, 2 = DDL, 3 = SSB

// ddlInMuxSel: {bank1_sw1&2} = DDL Input Mux sel
// 0 = Dry, 1 = Filter, 2 = WaveShaper, 3 = SSB

// ddlFBSrcSel: {bank1_sw3} = DDL feedback Source Mux Sel - sw7
// 0 = SSB, 1 = Filter

// ddlExtFBSel: {bank1_sw4} = DDL Internal/External Feedback sel - sw6
// 0 = Internal FB, 1 = External FB

// filterInMux: {bank1_sw5&6} = Filter Input Mux sel
// 0 = Dry, 1 = DDL, 2 = WaveShaper, 3 = DDLFBOut

// ssbInSel: {bank1_sw7} = SSB Input Mux sel
// 0 = Dry, 1 = Filter

// flt6Mux: {???} = Filter Output Mux6 sel
// 0 = moog-HL, 1 = SEM LP, 2 = SEM BP, 3 = SEM HP, 4 = SEM LP

// flt4Mux: {???} = Filter Output Mux4 sel
// 0 = moog-HL, 1 = SEM LP, 2 = SEM BP, 3 = SEM HP

// audioSrcMux: {right BTN} = input audio MUX sel
// 0 = sin, 1 = saw, 2 = sqr, 3 = pwm, 4 = external

/*------------------------------------------------------------------------------------------------*/
// Audio Mux Control [0:21]

//struct auMux_type {
//
//	// bank 0 : sw8 == 0		// IPI block name										  	bit-position
//	unsigned CH1_outMux: 3;		// Audio Source Mux CH1 (CH1_OutMux6)			- sw1&2&3 	- 0,1,2
//	unsigned CH2_outMux: 2;		// Audio Output Mux CH2 (CH2_OutMux4)			- sw4&5   	- 3,4
//	unsigned wsInMux: 2;		// Waveshaper Input Mux (filterInMux4Stereo)	- sw6&7   	- 5,6
//
//	// bank 1 : sw8 == 1
//	unsigned ddlInMux: 2;		// Delay Input Mux (ddlInMux4Stereo)			- sw1&2   	- 7,8
//	unsigned ddlFBSrcSel: 1;	// DDL FB Input Sel (ddlFBMux2Stereo)			- sw3     	- 9
//	unsigned ddlExtFBSel: 1;	// DDL FB Int/Ext Sel (selectExtFB)		        - sw4     	- 10
//	unsigned fltInMux: 2;		// Filter Input Mux (ddlFBMux2Stereo)			- sw5&6   	- 11,12
//	unsigned ssbInSel: 1;		// SSB Input Sel (ssbMux2Stereo) 				- sw7	  	- 13
//
//	unsigned flt6Mux: 3;		// Filter Output Mux6 (xlslice_filterMux6Sel)   - null    	- 14,15,16
//	unsigned flt4Mux: 2;		// Filter Output Mux4 (xlslice_filterMux4Sel)	- null    	- 17, 18
//
//	unsigned audioSrcMux: 3; 	// Audio Source Mux (audioSrcMux5Stereo)		- right BTN - 19,20,21
//};
//
//union auMux_u {
//	u32 audioMuxSel;
//	struct auMux_type audioSelect;
//};
//
//static union auMux_u auMuxUnion;


/*------------------------------------------------------------------------------------------------*/
//  __OSC4T__
//
// oscOutFreq = clkFs * oscStep / pow(2, hirezPhaseAccWidth)
// oscOutFreq = 100000000 * oscStep / pow(2, 36);
//
// osc1Step = osc1OutFreq * pow(2, hirezPhaseAccWidth) / clkFs;
// osc1Step = (int)osc1OutFreq * pow(2, 36) / 100000000;
//
// 560 Hz osc output 384829 (hex: 0x00005DF3D)
// 560 / 2 : 192414 (hex: 0x00002EF9E)

// //static int oscFreq1StepVal = 192414;
//static int oscFreq1StepVal = 60836;		// ~ 88.53 Hz - (??check calc??)
//static int oscFreq2StepVal = 317;		// ~ 118.09 Hz - (??check calc??)
//static int oscPWMStepVal = 3848;		// ~ 5.6 Hz
//
//const int hirezPhaseAccWidth = 36;					// high resolution phase accumulator width
//const int lorezPhaseAccWidth = 28;					// low resolution phase accumulator width
//
//static float oscFreq1;
//static float oscFreq2;

/* OSC4T USER INPUTS

ctrlBus_lfoType_V	[1:0]
ctrlBus_syncSw_V	[0:0]
ctrlBus_fMuxSel_V	[1:0]
ctrlBus_pwMuxSel_V	[1:0]
ctrlBus_upDn1_V		[0:0]
ctrlBus_upDn2_V		[0:0]

freq1_V 			[35:0]
freq2_V				[27:0]
fm_V				[27:0]
pwm_V				[27:0]
fmVol_V				[11:0]

*/

/*------------------------------------------------------------------------------------------------*/
//  __SSB - Single Side-Band Modulator__

static int ssbSetStepVal = 11000093;	// SSM Modulation frequency


/*------------------------------------------------------------------------------------------------*/
// define struct for Moog Half-Ladder Filter parameters

// ap_fixed<24,3>

//struct moogHLParam_type {
//	int K;
//	int alpha0;
//	int alpha;
//	int beta1;
//	int beta2;
//	int beta3;
//};
//
//struct moogHLParam_type mooghlParam;


/*------------------------------------------------------------------------------------------------*/
// define struct for SEM State Variable Filter parameters


//struct semSvfParam_type {
//	float alpha0;
//	float alpha;
//	float rho;
//};
//
//struct semSvfParam_type semlParam;


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
// PROTOTYPE FUNCTIONS

//int run_audio(void);
static void BTN_Intr_Handler(void *baseaddr_p);
//static void BTN0x1_Intr_Handler(void *baseaddr_p);
static void rotary_select(u8 *rotary_AB, u8 *faderMuxSel, float *faderLvl);
static void ROTARY1_Intr_Handler(void *InstancePtr);
static void ROTARY2_Intr_Handler(void *InstancePtr);
static void ROTARY3_Intr_Handler(void *InstancePtr);
static void ROTARY4_Intr_Handler(void *InstancePtr);
static int InterruptSystemSetup(XScuGic *XScuGicInstancePtr, XGpio *GpioInstancePtr, int INT_CH_MASK);
int IntcGpioInitFunction(u16 DeviceId, XGpio *GpioInstancePtr, Xil_ExceptionHandler X_Intr_Handler, int INT_X_ID, int INT_CH_MASK);
//void processSWT(void);
void selectScene(u8 scnSel);
//void sceneUpdate(struct auMux_type audioScnSel);
void odmkZynqLED1(unsigned long uSECPERBEAT);
void ftoa(char *buffer, float d, int precision);
void odmkInfoScreen(void *oledrgbPtr, float bpm);
void odmkAudioMuxScreen(void *oledrgbPtr, int muxSel);
void odmkFaderMuxScreen(void *oledrgbPtr, u8 faderSel, u8 faderNum);
void faderVal2Screen(void *oledrgbPtr, u8 faderSel, float faderVal, u8 faderNum);
//void setFcAndRes_ARM(float sampleRate, float cutoff, float resonance, struct moogHLParam_type *fParamSW);
void volCtrlDB_update(void *VOLCTRLPtr, u8 rot_AB);
//void osc1StepCtrl_update(void *OSCF1Ptr, u8 rot_AB);
//void osc2StepCtrl_update(void *OSCF2Ptr, u8 rot_AB);
//void pwmStepCtrl_update(void *PWMCTRLPtr, u8 rot_AB);

//void mooghlCutCtrl_update(void *MHLALPHAKPtr, void *MHLCTRL1Ptr, void *MHLCTRL2Ptr, u8 rot_AB);
//void mooghlResCtrl_update(void *MHLALPHAKPtr, void *MHLCTRL1Ptr, void *MHLCTRL2Ptr, u8 rot_AB);
//void semCutCtrl_update(void *SEMCTRL1Ptr, void *SEMCTRL2Ptr, u8 rot_AB);
//void semQCtrl_update(void *SEMCTRL1Ptr, void *SEMCTRL2Ptr, u8 rot_AB);

// called inside ddl update functions
void ddlxnLengthCtrl(const bool centerLock, const float delayBase,
		             const float tilt, ddllengthctrl_t ddlLengthBus);

// Rotary Encoder updates delay length base -> generates delay length values -> updates DDL AXI-bus
// pass XodAXICtrl object ptr
void ddlxnLength_update(void *DDLAXICTRLPtr, const bool centerLock, float *delayBase,
					    const float tilt, u8 rot_AB);

// Rotary Encoder updates frequency tilt -> generates phaseStep values -> updates SSB AXI-bus
// pass XodAXICtrl object ptr
void ddlxnTilt_update(void *DDLAXICTRLPtr, const bool centerLock, const float freqBase,
		              float *tilt, u8 rot_AB);


// called inside ssb update functions
void ssbModFreqCtrl(const bool centerLock, const float freqBase,
		            const float tilt, u32 phaseStepArray[MAXCHANNELS]);

// Rotary Encoder updates frequency base -> generates phaseStep values -> updates SSB AXI-bus
// pass XodAXICtrl object ptr
void ssbModFreq_update(void *SSBAXICTRLPtr, const bool centerLock, float *freqBase,
					   const float tilt, u8 rot_AB);

// Rotary Encoder updates frequency tilt -> generates phaseStep values -> updates SSB AXI-bus
// pass XodAXICtrl object ptr
void ssbModTilt_update(void *SSBAXICTRLPtr, const bool centerLock, const float freqBase,
		               float *tilt, u8 rot_AB);


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
// Multi-Channel Control functions

// __ddlxnLengthCtrl__
//
// USEAGE:
// ddlxnLengthCtrl<delayLen_t, channels>(ap_uint<1> centerLock, double delayBase, double tilt, T ddlLengthArray[channels]);
// ddlxnLengthCtrl<delayLen_t, 4>(1, 566.6, 56, ddlLengthArray);

//void ddlxnLengthCtrl(const bool centerLock, const float delayBase,
//		             const float tilt, float ddlLengthArray[MAXCHANNELS])
void ddlxnLengthCtrl(const bool centerLock, const float delayBase,
		             const float tilt, ddllengthctrl_t ddlLengthBus)
{
	int channels = (int)MAXCHANNELS;
	// centerLock: locks two center step values to base frequency (else both sides shift from base freq!)
	assert(channels % 2 == 0);
	assert(channels <= 8);

	//float ddlLengthArray[MAXCHANNELS] { 0 };


	if (centerLock) {
		for (int c = 0; c < channels; c++) {
			if ((c == channels / 2 - 1) || (c == channels / 2)) {
				ddlLengthArray[c] = delayBase;
			} else if (c < channels / 2 - 1) {
				ddlLengthArray[c] = delayBase - (( (channels / 2 - 1) - c) * tilt);
			} else {
				ddlLengthArray[c] = delayBase + ((c - channels/2) * tilt);
			}
		}

	} else {
		for (int c = 0; c < channels; c++) {
			if (c < channels / 2) {
				ddlLengthArray[c] = delayBase - ((channels/2 - c) * tilt);
			} else {
				ddlLengthArray[c] = delayBase + ((c - channels/2 + 1) * tilt);
			}
		}

	}

	for (int c = 0; c < channels; c++) {

		ddlLenCtrlUnion

	}
}


// Rotary Encoder updates delay length base -> generates delay length values -> updates DDL AXI-bus
// pass XodAXICtrl object ptr
void ddlxnLength_update(void *DDLAXICTRLPtr, const bool centerLock, float *delayBase,
					    const float tilt, u8 rot_AB)
{
	float ddlLenArray[MAXCHANNELS];

	if(rot_AB == 0) {
		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);ssbFreqScale
		*delayBase -= ddlLengthScale;

	}
	else if(rot_AB == 2) {
		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
		*delayBase += ddlLengthScale;

	}
	else {

	}

	ddlxnLengthCtrl(centerLock, *delayBase, tilt, ddlLenArray);

	XXodefx_top_Set_ddlLengthCtrl(DDLAXICTRLPtr, ddlLenArray);

}

// Rotary Encoder updates frequency tilt -> generates phaseStep values -> updates SSB AXI-bus
// pass XodAXICtrl object ptr
void ddlxnTilt_update(void *DDLAXICTRLPtr, const bool centerLock, const float delayBase,
		              float *tilt, u8 rot_AB)
{
	float ddlLenArray[MAXCHANNELS];

	if(rot_AB == 0) {
		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);ssbFreqScale
		*tilt -= ddlTiltScale;

	}
	else if(rot_AB == 2) {
		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
		*tilt += ddlTiltScale;

	}
	else {

	}

	ddlxnLengthCtrl(centerLock, delayBase, *tilt, ddlLenArray);

	XXodefx_top_Set_ddlLengthCtrl(DDLAXICTRLPtr, ddlLenArray);
}



// __ssbModFreqCtrl__
//
// USEAGE:
// ssbModFreqCtrl<ddsCtrl_t, channels, phaseWidth>
//					(ap_uint<1> centerLock, double freqBase, double tilt, T phaseStepArray[channels]);
// ssbModFreqCtrl<ddsCtrl_t, 4, phaseWidth>(1, 566.6, 56, phaseStepArr);
//
// +/-1499 Hz max/min freq ssbOsc CTRL UI (ddsCtrl_t = 32bits)
// (-1499 * (2**36 / 48000) = -2146051992  =>  2**31 - 1 = 2147483647 (1500 overflows 32 bits)

void ssbModFreqCtrl(const bool centerLock, const float freqBase,
		            const float tilt, u32 phaseStepArray[MAXCHANNELS])
{
	int channels = (int)MAXCHANNELS;
	// centerLock: locks two center step values to base frequency (else both sides shift from base freq!)
	assert(channels % 2 == 0);
	assert(channels <= 8);
	u32 ddsOutFreqInit[MAXCHANNELS];

	if (centerLock) {
		for (int c = 0; c < channels; c++) {
			if ((c == channels / 2 - 1) || (c == channels / 2)) {
				ddsOutFreqInit[c] = freqBase;
			} else if (c < channels / 2 - 1) {
				ddsOutFreqInit[c] = freqBase - (( (channels / 2 - 1) - c) * tilt);
			} else {
				ddsOutFreqInit[c] = freqBase + ((c - channels/2) * tilt);
			}
		}

	} else {
		for (int c = 0; c < channels; c++) {
			if (c < channels / 2) {
				ddsOutFreqInit[c] = freqBase - ((channels/2 - c) * tilt);
			} else {
				ddsOutFreqInit[c] = freqBase + ((c - channels/2 + 1) * tilt);
			}
		}

	}

	for (int c = 0; c < channels; c++) {
		// phaseStepArray[c] = static_cast<T>(ddsOutFreqInit[c] * pow(2, phaseWidth) / Fclk);
		phaseStepArray[c] = (u32)(ddsOutFreqInit[c] * pow(2, ddsPhaseWidth) / Fs);
	}
}


// Rotary Encoder updates frequency base -> generates phaseStep values -> updates SSB AXI-bus
// pass XodAXICtrl object ptr
//void ssbModFreq_update(void *SSBAXICTRLPtr, const bool centerLock, double *freqBase,
//					   const double tilt, u8 rot_AB, XXodefx_top_Ssbmodfreqctrl *ssbModPhaseSteps)
//{
//
////	//		typedef struct {
////	//		    u32 word_0;
////	//		    u32 word_1;
////	//		    u32 word_2;
////	//		    u32 word_3;
////	//		} XXodefx_top_Ssbmodfreqctrl;
////	XXodefx_top_Ssbmodfreqctrl ssbModPhaseStep;
//
//	// Update SSB Modulator phase accumulator value (SSB modulation Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);ssbFreqScale
//		ssbSetStepVal -= ssbFreqScale;
//
//		//if (ssbSetStepVal < 0) ssbSetStepVal = 0;
//		//XGpio_DiscreteWrite(SSBCTRLPtr, 1, ssbSetStepVal);
//
//
//		XXodefx_top_Set_ssbModFreqCtrl(SSBAXICTRLPtr, *ssbModPhaseSteps);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
//		ssbSetStepVal += ssbFreqScale;
//
//		//XGpio_DiscreteWrite(SSBCTRLPtr, 1, ssbSetStepVal);
//	}
//	else {
//
//	}
//}

// Rotary Encoder updates frequency tilt -> generates phaseStep values -> updates SSB AXI-bus
// pass XodAXICtrl object ptr
void ssbModTilt_update(void *SSBAXICTRLPtr, const bool centerLock, const float freqBase,
					   float *tilt, u8 rot_AB)
{
	XXodefx_top_Ssbmodfreqctrl ssbModDataPacked { 0 };

	if(rot_AB == 0) {
		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);ssbFreqScale
		tilt -= ssbFreqScale;

	}
	else if(rot_AB == 2) {
		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
		tilt += ssbFreqScale;

	}
	else {

	}
	//void ssbModFreqCtrl(const bool centerLock, const double freqBase,
	//		            const double tilt, ddsCtrl_t phaseStepArray[MAXCHANNELS])
	ssbModFreqCtrl(centerLock, freqBase, tilt, phaseStepArray);

//	typedef struct {
//	    u32 word_0;
//	    u32 word_1;
//	    u32 word_2;
//	    u32 word_3;
//	} XXodefx_top_Ssbmodfreqctrl;

	XXodefx_top_Set_ssbModFreqCtrl(SSBAXICTRLPtr, XXodefx_top_Ssbmodfreqctrl Data);

}

// ** FIXIT **
//----------------------------------------------------
// update SSB-OSC Step Ctrl parameter
// - input ptr to SSB Inst, rotary controller AB
//----------------------------------------------------
//void ssbStepCtrl_update(void *SSBCTRLPtr, u8 rot_AB) {
//	// Update SSB Modulator phase accumulator value (SSB modulation Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);ssbFreqScale
//		ssbSetStepVal -= ssbFreqScale;
//		//if (ssbSetStepVal < 0) ssbSetStepVal = 0;
//		XGpio_DiscreteWrite(SSBCTRLPtr, 1, ssbSetStepVal);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
//		ssbSetStepVal += ssbFreqScale;
//		XGpio_DiscreteWrite(SSBCTRLPtr, 1, ssbSetStepVal);
//	}
//	else {
//
//	}
//}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
// Parameter Update FUNCTIONS


//----------------------------------------------------
// update Volume Ctrl parameter
// - input ptr to VOLWSGAINInst, rotary controller AB
//----------------------------------------------------
void volCtrlDB_update(void *VOLCTRLPtr, u8 rot_AB) {

	printf("Master Vol Ctrl: volCtrlDB_update, rot_AB = %i\n\r", rot_AB);

	// Audio Output volume control {0 dB MAX : -140 dB MIN}
	// convert dB to 16 bit unsigned integer values {0:65536}
	if(rot_AB == 0) {
		volCtrlDB += mstrVolScale;
		if (volCtrlDB >= 140.0) {
			volCtrlDB = 140.0;		// -140 DB, Min Volume
		}
		volCtrlVal = (u32)maxVolVal * (pow((double)10.0, -volCtrlDB/20));
		printf("Rotary Enc -Left- rotary1_AB = %d,  volCtrlDB = %i\n", rotary1_AB, volCtrlVal);
		XGpio_DiscreteWrite(VOLCTRLPtr, 1, volCtrlVal);
	}
	else if(rot_AB == 2) {
		volCtrlDB -= mstrVolScale;
		if (volCtrlDB <= 0.0) {
			volCtrlDB = 0.0;		// 0 DB, Max Volume
		}
		volCtrlVal = (u32)maxVolVal * (pow((double)10.0, -volCtrlDB/20));
		printf("Rotary Enc -Right- rotary1_AB = %d\n", rotary1_AB);
		XGpio_DiscreteWrite(VOLCTRLPtr, 1, volCtrlVal);
	}
	else {

	}

}


//----------------------------------------------------
// update Waveshaper1 Gain Ctrl parameter
// - input ptr to VOLWSCTRLInst, rotary controller AB
//----------------------------------------------------
void wsGainCtrl_update(void *WSCTRLPtr, u8 rot_AB) {

	if(rot_AB == 0) {
		//printf("Rotary Enc -Left- rotary1_AB = %d\n",rotary1_AB);
		wsGainFloat-=wsGainScale;
		if (wsGainFloat < 1.0) {	// fuckd up at gains lower than 1!?!
			wsGainFloat = 1.0;
		}
		wsGainVal = (u32)(wsGainFloat * 64.0);

		XGpio_DiscreteWrite(WSCTRLPtr, 2, wsGainVal);
	}
	else if(rot_AB == 2) {
		//printf("Rotary Enc -Right- rotary1_AB = %d\n",rotary1_AB);
		wsGainFloat+=wsGainScale;
		if (wsGainFloat >= 1023.0) {
			wsGainFloat = 1023.0;
		}
		wsGainVal = (u32)(wsGainFloat * 64.0);
		XGpio_DiscreteWrite(WSCTRLPtr, 2, wsGainVal);
	}
	else {

	}

}


//----------------------------------------------------
// update OSC Step Ctrl parameter
// - input ptr to VOLWSCTRLInst, rotary controller AB
//----------------------------------------------------
//void osc1StepCtrl_update(void *OSCF1Ptr, u8 rot_AB) {
//	// Update DDS phase accumulator value (OSC output Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);
//		oscFreq1StepVal-=oscFreqScale;
//		XGpio_DiscreteWrite(OSCF1Ptr, 1, oscFreq1StepVal);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
//		oscFreq1StepVal+=oscFreqScale;
//		XGpio_DiscreteWrite(OSCF1Ptr, 1, oscFreq1StepVal);
//	}
//	else {
//
//	}
//}


//----------------------------------------------------
// update OSC1 Step Ctrl parameter
// - input ptr to VOLWSCTRLInst, rotary controller AB
//----------------------------------------------------
//void osc2StepCtrl_update(void *OSCF2Ptr, u8 rot_AB) {
//	// Update OSC4T freq2 phase accumulator value (LFO output Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary1_AB);
//		oscFreq2StepVal-=lfoFreqScale;
//		XGpio_DiscreteWrite(OSCF2Ptr, 1, oscFreq2StepVal);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary1_AB);
//		oscFreq2StepVal+=lfoFreqScale;
//		XGpio_DiscreteWrite(OSCF2Ptr, 1, oscFreq2StepVal);
//	}
//	else {
//
//	}
//}


//----------------------------------------------------
// update OSC1 Step Ctrl parameter
// - input ptr to OSC Inst, rotary controller AB
//----------------------------------------------------
//void pwmStepCtrl_update(void *PWMCTRLPtr, u8 rot_AB) {
//	// Update OSC4T PWM phase accumulator value (SSB modulation Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- (NoEvent) - rotary1_AB = %d\n\r",rotary1_AB);
//		oscPWMStepVal-=lfoFreqScale;
//		if (ssbSetStepVal < 0) ssbSetStepVal = 0;
//		XGpio_DiscreteWrite(PWMCTRLPtr, 2, oscPWMStepVal);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- (NoEvent) - rotary1_AB = %d\n\r",rotary1_AB);
//		oscPWMStepVal+=lfoFreqScale;
//		XGpio_DiscreteWrite(PWMCTRLPtr, 2, oscPWMStepVal);
//	}
//	else {
//
//	}
//}



//----------------------------------------------------
// Moog Half Ladder Filter Parameter update
//----------------------------------------------------
/*
cutoff = 500, res = 1.707
setFcAndRes_ARM: g = 0.0327366,	G = 0.0316989,	K = 1.707,	beta1 = -0.0287482,	beta2 = -0.906913,	beta3 = 1.9366,	alpha0 = 1.00161

cutoff = 8600, res = 1.0
setFcAndRes_ARM: g = 0.630954,	G = 0.386862,	K = 1,	beta1 = -0.0536727,	beta2 = -0.138739,	beta3 = 1.22628,	alpha0 = 1.03505
*/


//void setFcAndRes_ARM(float sampleRate, float cutoff, float resonance, struct moogHLParam_type *fParamSW)
//{
//	// prewarp for BZT
//	double wd = 2*pi*cutoff;
//	double T  = 1/(double)sampleRate;
//	double wa = (2/T)*tan(wd*T/2);
//	double g  = wa*T/2;
//
//	// G - the feedforward coeff in the VA One Pole
//	float G = g/(1.0 + g);
//
//	// the allpass G value
//	float GA = 2.0*G-1;
//
//	// calculate alpha0
//	// for 2nd order, K = 2 is max so limit it there
//	float K = resonance;
//	if(K > 2.0)
//		K = 2.0;
//
//	// parameter HLS type: ap_fixed<24,3>, therefore, scale by 2^21
//
////	fParamSW->beta1 = GA*G / (1.0 + g);
////	fParamSW->beta2 = GA / (1.0 + g);
////	fParamSW->beta3 = 2.0 / (1.0 + g);
////
////	fParamSW->K = K;
////	fParamSW->alpha = G;
////	fParamSW->alpha0 = 1.0 / (1.0 + K*GA*G*G);
//
////	printf("alpha = %f,  K = %f\n", fParamSW->alpha, fParamSW->K);
////	printf("alpha0 = %f,  beta1 = %f\n", fParamSW->alpha0, fParamSW->beta1);
////	printf("beta2 = %f,  beta3 = %f\n", fParamSW->beta2, fParamSW->beta3);
//
//	// scale for ap_fixed<3,24>
//	fParamSW->beta1 = (int)( (GA*G / (1.0 + g)) * pow(2, 21) );
//	fParamSW->beta2 = (int)( (GA / (1.0 + g)) * pow(2, 21) );
//	fParamSW->beta3 = (int)( (2.0 / (1.0 + g)) * pow(2, 21) );
//
//	fParamSW->K = (int)( K * pow(2, 21) );
//	fParamSW->alpha = (int)( G * pow(2, 21) );
//	fParamSW->alpha0 = (int)( (1.0 / (1.0 + K*GA*G*G)) * pow(2, 21) );
//
//}


//----------------------------------------------------
// update Moog Half-Ladder Cutoff Freq Ctrl parameter
// - input ptr to MHLALPHAK Inst, MHLCTRL1, MHLCTRL2, rotary controller AB
//----------------------------------------------------
//void mooghlCutCtrl_update(void *MHLALPHAKPtr, void *MHLCTRL1Ptr, void *MHLCTRL2Ptr, u8 rot_AB) {
//	// Update FLT Cutoff Freq & re-calculate parameters (FLT Cutoff Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n",rotary1_AB);
//		mooghlCutoff-=mooghlCutoffScale;
//		if (mooghlCutoff <= 0) {
//			mooghlCutoff = 0;
//		}
//		setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 1, mooghlParam.alpha);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 2, mooghlParam.K);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 1, mooghlParam.alpha0);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 2, mooghlParam.beta1);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 1, mooghlParam.beta2);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 2, mooghlParam.beta3);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n",rotary1_AB);
//		mooghlCutoff+=mooghlCutoffScale;
//		if (mooghlCutoff >= 30860) {
//			mooghlCutoff = 30860;
//		}
//		setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 1, mooghlParam.alpha);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 2, mooghlParam.K);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 1, mooghlParam.alpha0);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 2, mooghlParam.beta1);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 1, mooghlParam.beta2);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 2, mooghlParam.beta3);
//	}
//	else {
//
//	}
//}


//----------------------------------------------------
// update Moog Half-Ladder Resonance Ctrl parameter
// - input ptr to MHLALPHAK Inst, MHLCTRL1, MHLCTRL2, rotary controller AB
//----------------------------------------------------
//void mooghlResCtrl_update(void *MHLALPHAKPtr, void *MHLCTRL1Ptr, void *MHLCTRL2Ptr, u8 rot_AB) {
//	// Update FLT Cutoff Freq & re-calculate parameters (FLT Cutoff Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n",rotary1_AB);
//		mooghlRes-=mooghlResScale;
//		//mooghlRes-=0.003;
//		if (mooghlRes <= 0.0) {
//			mooghlRes = 0.0;
//		}
//		setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 1, mooghlParam.alpha);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 2, mooghlParam.K);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 1, mooghlParam.alpha0);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 2, mooghlParam.beta1);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 1, mooghlParam.beta2);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 2, mooghlParam.beta3);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n",rotary1_AB);
//		mooghlRes+=mooghlResScale;
//		//mooghlRes+=0.003;
//		if (mooghlRes >= 2.0) {
//			mooghlRes = 2.0;
//		}
//		setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 1, mooghlParam.alpha);
//		XGpio_DiscreteWrite(MHLALPHAKPtr, 2, mooghlParam.K);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 1, mooghlParam.alpha0);
//		XGpio_DiscreteWrite(MHLCTRL1Ptr, 2, mooghlParam.beta1);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 1, mooghlParam.beta2);
//		XGpio_DiscreteWrite(MHLCTRL2Ptr, 2, mooghlParam.beta3);
//	}
//	else {
//
//	}
//}


//----------------------------------------------------
// SEM SVF filter ARM software - alpha0, alpha, rho
//----------------------------------------------------

// decode the Q value; Q on UI is 1->10
//int odmkSEMSVF_setQ_ref(float Q)
//{
//	// this maps dQControl = 1->10 to Q = 0.5->25
//	//float Qscaled = (25.0 - 0.5) * (Q - 1.0) / (10.0 - 1.0) + 0.5;
//	//return Qscaled;
//	int Qscaled = (int)( ((25.0 - 0.5) * (Q - 1.0) / (10.0 - 1.0) + 0.5) * pow(2, 21) );
//	return Qscaled;
//}


// recalc the coeffs
//void odmkSEMSVF_update(float sampleRate, float cutoff, float Q, int alpha0, int alpha, int rho)
//{
//
//	// prewarp the cutoff- these are bilinear-transform filters
//	float wd = 2 * pi * cutoff;
//	float T  = 1 / sampleRate;
//	float wa = (2/T) * tan(wd * T/2);
//	float g  = wa*T/2;
//
//	// note R is the traditional analog damping factor
//	float R = 1.0 / (2.0 * Q);
//
//	// set the coeffs
////	alpha0 = 1.0 / (1.0 + 2.0 * R*g + g*g);
////	alpha = g;
////	rho = 2.0 * R + g;
//
//	alpha0 = (int)( (1.0 / (1.0 + 2.0 * R*g + g*g)) * pow(2, 21) );
//	alpha = (int)( g * pow(2, 21) );
//	rho = (int)( (2.0 * R + g) * pow(2, 21) );
//
//
//	//cout<<endl<<"TB odmkSEMSVF_update rho: "<<rho<<endl;
//
//}


//----------------------------------------------------
// update Moog Half-Ladder Cutoff Freq Ctrl parameter
// - input ptr to MHLALPHAK Inst, MHLCTRL1, MHLCTRL2, rotary controller AB
//----------------------------------------------------
//void semCutCtrl_update(void *SEMCTRL1Ptr, void *SEMCTRL2Ptr, u8 rot_AB) {
//	// Update FLT Cutoff Freq & re-calculate parameters (FLT Cutoff Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n",rotary1_AB);
//		semCutoff-=semCutoffScale;
//		if (semCutoff <= 0) {
//			semCutoff = 0;
//		}
//		// *-----------------------------------------------------------------------------------* //
//		///// generate alpha0, alpha, rho in testbench (ARM SW function) /////////////////////
//
//		semQscaled = odmkSEMSVF_setQ_ref(semQ);
//		odmkSEMSVF_update(fs, semCutoff, semQscaled, semlParam.alpha0, semlParam.alpha, semlParam.rho);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 1, semlParam.alpha0);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 2, semlParam.alpha);
//		XGpio_DiscreteWrite(SEMCTRL2Ptr, 1, semlParam.rho);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n",rotary1_AB);
//		semCutoff+=semCutoffScale;
//		if (semCutoff >= 30860) {
//			semCutoff = 30860;
//		}
//		semQscaled = odmkSEMSVF_setQ_ref(semQ);
//		odmkSEMSVF_update(fs, semCutoff, semQscaled, semlParam.alpha0, semlParam.alpha, semlParam.rho);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 1, semlParam.alpha0);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 2, semlParam.alpha);
//		XGpio_DiscreteWrite(SEMCTRL2Ptr, 1, semlParam.rho);
//	}
//	else {
//
//	}
//}


//----------------------------------------------------
// update Moog Half-Ladder Resonance Ctrl parameter
// - input ptr to MHLALPHAK Inst, MHLCTRL1, MHLCTRL2, rotary controller AB
//----------------------------------------------------
//void semQCtrl_update(void *SEMCTRL1Ptr, void *SEMCTRL2Ptr, u8 rot_AB) {
//	// Update FLT Cutoff Freq & re-calculate parameters (FLT Cutoff Frequency)
//	if(rot_AB == 0) {
//		//printf("Rotary Enc -Left- rotary1_AB = %d\n",rotary1_AB);
//		semQ-=semQScale;
//		//mooghlRes-=0.003;
//		if (semQ <= 0.0) {
//			semQ = 0.0;
//		}
//		// *-----------------------------------------------------------------------------------* //
//		///// generate alpha0, alpha, rho in testbench (ARM SW function) /////////////////////
//
//		semQscaled = odmkSEMSVF_setQ_ref(semQ);
//		odmkSEMSVF_update(fs, semCutoff, semQscaled, semlParam.alpha0, semlParam.alpha, semlParam.rho);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 1, semlParam.alpha0);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 2, semlParam.alpha);
//		XGpio_DiscreteWrite(SEMCTRL2Ptr, 1, semlParam.rho);
//	}
//	else if(rot_AB == 2) {
//		//printf("Rotary Enc -Right- rotary1_AB = %d\n",rotary1_AB);
//		semQ+=semQScale;
//		if (semQ >= 30860) {
//			semQ = 30860;
//		}
//		semQscaled = odmkSEMSVF_setQ_ref(semQ);
//		odmkSEMSVF_update(fs, semCutoff, semQscaled, semlParam.alpha0, semlParam.alpha, semlParam.rho);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 1, semlParam.alpha0);
//		XGpio_DiscreteWrite(SEMCTRL1Ptr, 2, semlParam.alpha);
//		XGpio_DiscreteWrite(SEMCTRL2Ptr, 1, semlParam.rho);
//	}
//	else {
//
//	}
//}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/


//----------------------------------------------------
// read and process switch states
//
//----------------------------------------------------

//void processSWT() {
//	/*------------------------------------------------------------------------------*/
//	// read and process switch states
//
//	swsUnion.swValue = XGpio_DiscreteRead(&LEDSWSInst, 2);
//
//	if (swsUnion.switches.sw8 == 0) {
//
//		// switch bank 0:
//
//		// update CH1_OutMux6 Mux Select
//		auMuxUnion.audioSelect.CH1_outMux = (u8)(7&swsUnion.swValue);
//		if (CH1_outMux_prev != 3 && auMuxUnion.audioSelect.CH1_outMux == 3) {
//			printf("Enabled Filter Gain Compensation\n");
//			printf("volCtrlDB = %f,   volCtrlVal = %d\n", volCtrlDB, (int)volCtrlVal);
//			//volCtrlDB+=filtVolScale;	// ?doesn't work?
//			volCtrlDB-=6.0;
//			if (volCtrlDB <= 0.0) {
//				volCtrlDB = 0.0;		// 0 DB, Max Volume
//			}
//			volCtrlVal = (u32)maxVolVal*(pow((float)10.0, -volCtrlDB/20));
//			XGpio_DiscreteWrite(&VOLWSCTRLInst, 1, volCtrlVal);
//			CH1_outMux_prev = auMuxUnion.audioSelect.CH1_outMux;
//		} else if(CH1_outMux_prev == 3 && auMuxUnion.audioSelect.CH1_outMux != 3) {
//			printf("Disabled Filter Gain Compensation\n");
//			printf("volCtrlDB = %f,   volCtrlVal = %d\n", volCtrlDB, (int)volCtrlVal);
//			//volCtrlDB-=filtVolScale;
//			volCtrlDB+=6.0 ;
//			if (volCtrlDB >= 140.0) {
//				volCtrlDB = 140.0;		// -140 DB, Min Volume
//			}
//			volCtrlVal = (u32)maxVolVal*(pow((float)10.0, -volCtrlDB/20));
//			XGpio_DiscreteWrite(&VOLWSCTRLInst, 1, volCtrlVal);
//			CH1_outMux_prev = auMuxUnion.audioSelect.CH1_outMux;
//		}
//
//
//		// update Waveshaper Input Mux Select - sw6&7
//		auMuxUnion.audioSelect.wsInMux = (u8)(96&swsUnion.swValue)>>5;
//		if (auMuxUnion.audioSelect.wsInMux == 0) {
//			printf("Waveshaper Input: Direct\n");
//		} else if (auMuxUnion.audioSelect.wsInMux == 1) {
//			printf("Waveshaper Input: Filter\n");
//		} else if (auMuxUnion.audioSelect.wsInMux == 2) {
//			printf("Waveshaper Input: DDL\n");
//		} else if (auMuxUnion.audioSelect.wsInMux == 3) {
//			printf("Waveshaper Input: SSB\n");
//		}
//
//	} else {
//
//		// switch bank 1:
//
//		// update DDL Data In Mux (ddlInMux2Stereo) - sw5
//		auMuxUnion.audioSelect.ddlInMux = (u8)(3&swsUnion.swValue);
//		if (auMuxUnion.audioSelect.ddlInMux == 0) {
//			printf("DDL Input: Source Direct\n");
//		} else if (auMuxUnion.audioSelect.ddlInMux == 1) {
//			printf("DDL Input: Filter\n");
//		} else if (auMuxUnion.audioSelect.ddlInMux == 2) {
//			printf("DDL Input: Waveshaper\n");
//		} else if (auMuxUnion.audioSelect.ddlInMux == 3) {
//			printf("DDL Input: SSB\n");
//		}
//
//		// update DDL feedbackIn Mux Select {CH1: SSB, CH2: Filter} - sw3
//		auMuxUnion.audioSelect.ddlFBSrcSel = swsUnion.switches.sw3;
//		if (auMuxUnion.audioSelect.ddlFBSrcSel == 1) {
//			printf("DDL feedback Input: Filter");
//		} else {
//			printf("DDL feedback Input: SSB Modulator");
//		}
//		// update DDL selectExtFB - sw4
//		auMuxUnion.audioSelect.ddlExtFBSel = swsUnion.switches.sw4;
//		printf("DDL selectExtFB = %d\n", auMuxUnion.audioSelect.ddlExtFBSel);
//
//		// update filter Data In Mux (filterInMux3Stereo) - sw5&sw6
//		auMuxUnion.audioSelect.fltInMux = (u8)(48&swsUnion.swValue)>>4;
//		if (auMuxUnion.audioSelect.fltInMux == 0) {
//			printf("Filter Input: Source Direct\n");
//		} else if (auMuxUnion.audioSelect.fltInMux == 1) {
//			printf("Filter Input: DDL\n");
//		} else if (auMuxUnion.audioSelect.fltInMux == 2) {
//			printf("Filter Input: Waveshaper\n");
//		} else if (auMuxUnion.audioSelect.fltInMux == 3) {
//			printf("Filter Input: DDL FB Out\n");
//		}
//
//		// update SSB - sw7
//		auMuxUnion.audioSelect.ssbInSel = swsUnion.switches.sw7;
//		if (auMuxUnion.audioSelect.ssbInSel == 1) {
//			printf("SSB Input: Source Direct\n");
//		} else {
//			printf("SSB Input: Filter\n\n");
//		}
//
//	}
//
//	XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
//
//}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
// INTERRUPT HANDLER FUNCTIONS


//----------------------------------------------------
// Button INTERRUPT HANDLER
// - button interrupt, performs
// -
// - Button Values returned when pressed:
// - center: 1, bottom: 2, left: 4, right: 8, top: 16
//----------------------------------------------------
void BTN_Intr_Handler(void *InstancePtr)
{

	// Disable GPIO interrupts
	XGpio_InterruptDisable(&BTNInst, BTN_CH_MASK);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(&BTNInst) & BTN_CH_MASK) != BTN_CH_MASK) {
		return;
	}

	usleep(50000);	//Wait 1/1000 seconds - button debounce

	btnValue = XGpio_DiscreteRead(&BTNInst, 1);

	// Center button pressed
	if(btnValue == 1) {
		// Reset to Top - print initial info / tempo - LED BPM flash
		printf("BTN[%d] Go State (update SWT)\n", btnValue);
		sysmode = 0;
		trigOnce = 1;

		// read and process switch states
		//processSWT();
	}

	// Lower Button pressed
	else if(btnValue == 2) {
		// Empty Function
		printf("BTN[%d] - Empty Function\n", btnValue);
	}

	// Left Button pressed
	else if(btnValue == 4) {
		// Empty Function
		printf("BTN[%d] - Empty Function\n", btnValue);
	}

	// Right Button pressed
	else if(btnValue == 8) {
		// Audio Mux Select - btn increments mux sel
		printf("BTN[%d] Audio Source Select\n", btnValue);
		//printf("audioSrcMux: %d\n", auMuxUnion.audioSelect.audioSrcMux);
		sysmode = 1;
		trigOnce = 1;

//		if (auMuxUnion.audioSelect.audioSrcMux == 4) {
//			auMuxUnion.audioSelect.audioSrcMux = 0;
//		} else {
//			auMuxUnion.audioSelect.audioSrcMux++;
//		}
//
//		printf("audioMuxSelect = %d\n", (int)auMuxUnion.audioMuxSel);
//
//		XGpio_DiscreteWrite( &AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);

	}

	// Top Button pressed
	else if(btnValue == 16) {
		// Empty Function
		printf("BTN[%d] - Empty Function\n", btnValue);
	}

	usleep(300000);	//Wait 1/1000 seconds - button debounce

    (void)XGpio_InterruptClear(&BTNInst, BTN_CH_MASK);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(&BTNInst, BTN_CH_MASK);

}


//----------------------------------------------------
// Pmod BTN INTERRUPT HANDLER
// - button interrupt, performs
// -
// - Button Values returned when pressed:
// - pBTN1, pBTN2, pBTN3, pBTN4
//----------------------------------------------------
//void BTN0x1_Intr_Handler(void *InstancePtr)
//{
//
//	// Disable GPIO interrupts
//	XGpio_InterruptDisable(InstancePtr, BTN0_CH_MASK);
//
//	// Ignore additional button presses
//	if ((XGpio_InterruptGetStatus(InstancePtr) & BTN0_CH_MASK) != BTN0_CH_MASK) {
//		return;
//	}
//
//	btn0x1_value = XGpio_DiscreteRead(InstancePtr, 1);
//
//	//printf("Pmod BTN0 - btn0x1_value = %d\n", btn0x1_value);
//
//	if(btn0x1_value == 1) {
//		// Fader1 Mux Sel - selects the target for fader1 control
//		printf("btn0x1[%d] - Fader1 Target Select\n", btn0x1_value);
//		sysmode = 2;
//		trigOnce = 1;
//
//		if (fader1MuxSel == 12) {
//			fader1MuxSel = 0;
//		} else {
//			fader1MuxSel++;
//		}
//	}
//	else if(btn0x1_value == 2) {
//		// Fader2 Mux Sel - selects the target for fader2 control
//		printf("btn0x1[%d] - Fader 2 Target Select\n", btn0x1_value);
//
//		sysmode = 3;
//		trigOnce = 1;
//
//		if (fader2MuxSel == 12) {
//			fader2MuxSel = 0;
//		} else {
//			fader2MuxSel++;
//		}
//
//	}
//	else if(btn0x1_value == 4) {
//		// Fader2 Mux Sel - selects the target for fader2 control
//		printf("btn0x1[%d] - Fader3 Target Select\n", btn0x1_value);
//		sysmode = 4;
//		trigOnce = 1;
//		if (fader3MuxSel == 12) {
//			fader3MuxSel = 0;
//		} else {
//			fader3MuxSel++;
//		}
//
//	}
//	else if(btn0x1_value == 8) {
//		// Audio Mux Select - btn increments mux sel
//		printf("BTN0x1[%d] - Fader4 Target Select\n", btn0x1_value);
//		sysmode = 5;
//		trigOnce = 1;
//		if (fader4MuxSel == 12) {
//			fader4MuxSel = 0;
//		} else {
//			fader4MuxSel++;
//		}
//
//	}
//	// null case
//	else {
//		// - null -
//		printf("btn0x1[%d] - null - \n", btn0x1_value);
//	}
//
//	usleep(300000);		// Wait 1/? seconds - button debounce
//						// prevents release-triggered interrupt
//
//    (void)XGpio_InterruptClear(InstancePtr, BTN0_CH_MASK);
//
//    // Enable GPIO interrupts
//    XGpio_InterruptEnable(InstancePtr, BTN0_CH_MASK);
//
//}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------

//faderMuxSel == 0: VOLWSGAIN -  (Master Volume)


void rotary_select(u8 *rotary_AB, u8 *faderMuxSel, float *faderLvl)
{

	if (*faderMuxSel == 0) {
		printf("Master Vol Ctrl: Fader MuxSel = %d\n\r", faderMuxSel);
		volCtrlDB_update(&VOLWSGAINInst, *rotary_AB);
		*faderLvl = (float)volCtrlDB;

	} else if (*faderMuxSel == 1) {

		//printf("DDL Length Ctrl: Fader2 MuxSel = %d\n\r",fader2MuxSel);
		// DDL Length control {0 samples MIN : ? samples MAX}

		//int delayLengthTmp2 = delayLengthTmp;
		//ddlLengthShift = (int)log(delayLengthTmp2 >> 8);
		//ddlLengthScale = ddlLengthScaleBase << ddlLengthShift;

		if(*rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary1_AB = %d\n\r",rotary2_AB);

			delayBase -= ddlLengthScale;
			if (delayBase < 0) delayBase = 0;
			if (delayBase > maxDlyLength) delayBase = maxDlyLength;

//			float delayLengthTmp = delayBase;
//			delayLengthTmp -= ddlLengthScale;
//			if (delayLengthTmp < 0 || delayLengthTmp > maxDlyLength) delayLengthTmp = 0;
//			delayBase = delayLengthTmp;
			ddlxnLength_update(&XodAXICtrl, centerLock, &delayBase, cv   tilt, 0);


		} else if(*rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary1_AB = %d\n\r",rotary2_AB);

			delayBase += ddlLengthScale;
			if (delayBase < 0) delayBase = 0;
			if (delayBase > maxDlyLength) delayBase = maxDlyLength;

//			DDLCTRL2Union.DDLCTRL2.delayLength = delayLengthTmp;
//			printf("delayLength = %d\n", (int)DDLCTRL2Union.DDLCTRL2.delayLength);
//			XGpio_DiscreteWrite(&DDLCTRLInst, 2, DDLCTRL2Union.DDLCTRL_CH2Val);
			ddlxnLength_update(&XodAXICtrl, centerLock, &delayBase, cv   tilt, 2);

		}
		else {
		}
		*faderLvl = (float)(delayLengthTmp>>16);		// shift 8 MSB, 8 frac

	} else if (*faderMuxSel == 2) {
		//printf("DDL Wet/Dry Ctrl: Fader2 MuxSel = %d\n\r",fader2MuxSel);
		// DDL Wet/Dry 15bit {0 = 100% Dry : 32767 100% Wet}
		if(*rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary2_AB = %d\n", rotary2_AB);
			ddlWetMixFloat -= ddlWetDryScale;
			if (ddlWetMixFloat <= 0.0) ddlWetMixFloat = 0.0;		// -140 DB, Min Volume
			DDLCTRL1Union.DDLCTRL1.wetMix = (u16)(ddlWetMixFloat * (1 << (ddlCtrlBwidth-1)));
			//printf("ddlWetMixFloat = %f,  wetMix = %d\n", ddlWetMixFloat, (int)DDLCTRL1Union.DDLCTRL1.wetMix);
			XGpio_DiscreteWrite(&DDLCTRLInst, 1, DDLCTRL1Union.DDLCTRL_CH1Val);
		}
		else if(*rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary2_AB = %d\n", rotary2_AB);
			ddlWetMixFloat += ddlWetDryScale;
			if (ddlWetMixFloat >= 1.0) ddlWetMixFloat = 1.0;
			DDLCTRL1Union.DDLCTRL1.wetMix = (u16)(ddlWetMixFloat * (1 << (ddlCtrlBwidth-1)));
			//printf("ddlWetMixFloat = %f,  wetMix = %d\n", ddlWetMixFloat, (int)DDLCTRL1Union.DDLCTRL1.wetMix);
			XGpio_DiscreteWrite(&DDLCTRLInst, 1, DDLCTRL1Union.DDLCTRL_CH1Val);
		}
		else {
		}
		*faderLvl = ddlWetMixFloat*100;

	} else if (*faderMuxSel == 3) {

		//printf("DDL feedback Gain Ctrl: Fader2 MuxSel = %d\n\r",fader2MuxSel);
		// DDL feedback Gain control {0 samples MIN : ? samples MAX}
		if(*rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary1_AB = %d\n\r", rotary2_AB);
			ddlFBGainFloat -= ddlFbGainScale;
			if (ddlFBGainFloat < 0) ddlFBGainFloat = 0;
			DDLCTRL1Union.DDLCTRL1.feedbackGain = (u16)(ddlFBGainFloat * (1 << (ddlCtrlBwidth-1)));
			printf("ddlFBGainFloat = %f,   feedbackGain = %d\n", ddlFBGainFloat, (int)DDLCTRL1Union.DDLCTRL1.feedbackGain);
			XGpio_DiscreteWrite(&DDLCTRLInst, 1, DDLCTRL1Union.DDLCTRL_CH1Val);
		}
		else if(*rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary1_AB = %d\n\r", rotary2_AB);
			ddlFBGainFloat += ddlFbGainScale;
			if (ddlFBGainFloat >= 1.0) {
				ddlFBGainFloat = 1.0;
			}
			DDLCTRL1Union.DDLCTRL1.feedbackGain = (u16)(ddlFBGainFloat * (1 << (ddlCtrlBwidth-1)));
			printf("ddlFBGainFloat = %f,   feedbackGain = %d\n", ddlFBGainFloat, (int)DDLCTRL1Union.DDLCTRL1.feedbackGain);
			XGpio_DiscreteWrite(&DDLCTRLInst, 1, DDLCTRL1Union.DDLCTRL_CH1Val);
		}
		else {
		}
		*faderLvl = ddlFBGainFloat * 100;    // (???)

	} else if (*faderMuxSel == 4) {
		//printf("SSB Freq Ctrl: Fader MuxSel = %d\n\r",);
		ssbStepCtrl_update(&SSBMODFreq1Inst, *rotary_AB);
		*faderLvl = (float)ssbSetStepVal;


	} else if (*faderMuxSel == 6) {
		//printf("Waveshaper1 Gain Ctrl: Fader MuxSel = %d\n\r", fader1MuxSel);
		wsGainCtrl_update(&VOLWSGAINInst, *rotary_AB);
		*faderLvl = wsGainFloat;


//	} else if (*faderMuxSel == 2) {
//		//printf("FLT Cutoff Freq Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
//		mooghlCutCtrl_update(&MHLALPHAKInst, &MHLCTRL1Inst, &MHLCTRL2Inst, *rotary_AB);
//		*faderLvl = mooghlCutoff;
//
//	} else if (*faderMuxSel == 3) {
//		//printf("FLT Resonance Gain Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
//		mooghlResCtrl_update(&MHLALPHAKInst, &MHLCTRL1Inst, &MHLCTRL2Inst, *rotary_AB);
//		*faderLvl = mooghlRes;
//
//	} else if (*faderMuxSel == 4) {
//		//printf("FLT Cutoff Freq Ctrl: Fader MuxSel = %d\n\r",);
//		semCutCtrl_update(&SEMCTRL1Inst, &SEMCTRL2Inst, *rotary_AB);
//		*faderLvl = semCutoff;
//
//	} else if (*faderMuxSel == 5) {
//		//printf("FLT Resonance Gain Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
//		semQCtrl_update(&SEMCTRL1Inst, &SEMCTRL2Inst, *rotary_AB);
//		*faderLvl = semQ;
//
//
//
//	} else if (*faderMuxSel == 10) {
//		//printf("OSC Freq Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
//		osc1StepCtrl_update(&OSCF1SSBStepInst, *rotary_AB);
//		*faderLvl = (float)oscFreq1StepVal;
//
//	} else if (*faderMuxSel == 11) {
//		//printf("LFO Freq Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
//		osc2StepCtrl_update(&OSCF2PWMStepInst, *rotary_AB);
//		*faderLvl = (float)oscFreq2StepVal;
//
//	} else if (*faderMuxSel == 12) {
//		//printf("PWM Freq Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
//		pwmStepCtrl_update(&OSCF2PWMStepInst, *rotary_AB);
//		*faderLvl = (float)oscPWMStepVal;

	} else {
		//printf("Master Vol Ctrl: Fader MuxSel = %d\n\r",fader1MuxSel);
		volCtrlDB_update(&VOLWSGAINInst, *rotary_AB);
		*faderLvl = (float)volCtrlDB;
	}

}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER CH1
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------
void ROTARY1_Intr_Handler(void *InstancePtr)
{

	//printf("ROTARY1_Intr_Handler: entered\n\r");

	// Disable GPIO interrupts
	XGpio_InterruptDisable(InstancePtr, ROTARY_INT);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(InstancePtr) & ROTARY_INT) != ROTARY_INT) {
		return;
	}

	rotary1_AB = XGpio_DiscreteRead(InstancePtr, 1);

	rotary_select(&rotary1_AB, &fader1MuxSel, &fader1Lvl);

	sysmode = 6;
	trigOnce = 1;

	//printf("ROTARY1_Intr_Handler: sysmode = 6\n\r");

    (void)XGpio_InterruptClear(InstancePtr, ROTARY_INT);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(InstancePtr, ROTARY_INT);

}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER CH2
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------
void ROTARY2_Intr_Handler(void *InstancePtr)
{

	//printf("ROTARY2_Intr_Handler: entered\n\r");

	// Disable GPIO interrupts
	XGpio_InterruptDisable(InstancePtr, ROTARY_INT);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(InstancePtr) & ROTARY_INT) != ROTARY_INT) {
		return;
	}

	rotary2_AB = XGpio_DiscreteRead(InstancePtr, 1);

	rotary_select(&rotary2_AB, &fader2MuxSel, &fader2Lvl);

	sysmode = 7;
	trigOnce = 1;

    (void)XGpio_InterruptClear(InstancePtr, ROTARY_INT);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(InstancePtr, ROTARY_INT);

}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER CH3
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------
void ROTARY3_Intr_Handler(void *InstancePtr)
{

	// Disable GPIO interrupts
	XGpio_InterruptDisable(InstancePtr, ROTARY_INT);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(InstancePtr) & ROTARY_INT) != ROTARY_INT) {
		return;
	}

	rotary3_AB = XGpio_DiscreteRead(InstancePtr, 1);

	rotary3_AB &= 3;

	rotary_select(&rotary3_AB, &fader3MuxSel, &fader3Lvl);

	sysmode = 8;
	trigOnce = 1;

    (void)XGpio_InterruptClear(InstancePtr, ROTARY_INT);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(InstancePtr, ROTARY_INT);

}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER CH4
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------
void ROTARY4_Intr_Handler(void *InstancePtr)
{

	// Disable GPIO interrupts
	XGpio_InterruptDisable(InstancePtr, ROTARY_INT);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(InstancePtr) & ROTARY_INT) != ROTARY_INT) {
		return;
	}

	rotary4_AB = XGpio_DiscreteRead(InstancePtr, 1);

	rotary4_AB &= 3;

	rotary_select(&rotary4_AB, &fader4MuxSel, &fader4Lvl);

	sysmode = 9;
	trigOnce = 1;

    (void)XGpio_InterruptClear(InstancePtr, ROTARY_INT);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(InstancePtr, ROTARY_INT);

}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

//int flywheel_read(rotaryDin) {
//	static const double speed = 15.0; // Lower values change faster
//	timespec t1;                   // Time now
//	double diff, rate;             // Difference, rate
//	int rotaryOut, m = 1;                  // Return r, m multiple
//
//	get_time(t1);                   // Now
//	diff = timediff(t0,t1);         // Diff in seconds
//
//	if ( rotaryDin != 0 ) {                // Got a click event
//	lastr = rotaryDin;             // Save the event type
//	ival = ( ival * 0.75 + diff * 1.25 ) / 2.0 * 1.10;
//	if ( ival < 1.0 && ival > 0.00001 ) {
//		rate = 1.0 / ival;
//	} else rate = 0.0;
//	t0 = t1;
//	if ( speed > 0.0 )
//		m = rate / speed;
//	} else if ( diff > ival && ival >= 0.000001 ) {
//		rate = 1.0 / ival;      // Compute a rate
//		if ( rate > 15.0 ) {
//			if ( speed > 0.0 )
//				m = rate / speed;
//			ival *= 1.2;    // Increase interval
//			t0 = t1;
//			rotaryOut = lastr;      // Return last r
//		}
//	}
//	return m > 0 ? rotaryOut * m : rotaryOut;
//}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/


//----------------------------------------------------
// - ODMK LED Manipulation
//----------------------------------------------------
void odmkZynqLED1(unsigned long uSECPERBEAT)
{

	//int Delay;
	static int ledD;
	ledD = ledFlash1[ledState];

	/* Write output to the LEDs. */
	XGpio_DiscreteWrite(&LEDSWSInst, LED_CHANNEL, ledD);

	/* Wait a small amount of time so that the LED blinking is visible. */
	//for (Delay = 0; Delay < CYCLESPERBEAT; Delay++);
	usleep(uSECPERBEAT);	// Wait 1 beat

	if (ledState == 3) {
		ledState = 0;
	} else {
		ledState++;
	}

}


/*------------------------------------------------------------------------------------------------*/
// - OLED Screen Functions


//----------------------------------------------------
// - convert float to character array
//----------------------------------------------------
void ftoa(char *buffer, float d, int precision) {

	long wholePart = (long) d;

	// Deposit the whole part of the number.
	itoa(wholePart,buffer,10);

	// Now work on the fraction if we need one.
	if (precision > 0) {

		// locate the end of the string and insert a decimal point.
		char *endOfString = buffer;
		while (*endOfString != '\0') endOfString++;
		*endOfString++ = '.';

		// work on the fraction, be sure to turn any negative values positive.
		if (d < 0) {
			d *= -1;
			wholePart *= -1;
		}

		float fraction = d - wholePart;
		while (precision > 0) {

			// Multiple by ten and pull out the digit.
			fraction *= 10;
			wholePart = (long) fraction;
			*endOfString++ = '0' + wholePart;

			// Update the fraction and move on to the next digit.
			fraction -= wholePart;
			precision--;
		}

		// Terminate the string.
		*endOfString = '\0';
	}

}


//----------------------------------------------------
// - convert float to character array
//----------------------------------------------------
void odmkInfoScreen(void *oledrgbPtr, float bpm)
{
	char ch;
	char str2[5] = " bpm";
	char bpmStr[8];

	ftoa(bpmStr, bpm, 1);

	strcat(bpmStr, str2);

	/* output to screen */
	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(255, 0, 0)); // blue font
	OLEDrgb_SetCursor(oledrgbPtr, 2, 3);
	OLEDrgb_PutString(oledrgbPtr, "*XODMK*");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44)); // blue font
	OLEDrgb_SetCursor(oledrgbPtr, 1, 5);
	OLEDrgb_PutString(oledrgbPtr, bpmStr);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 7);
	OLEDrgb_PutString(oledrgbPtr, "CHROMSPHNX");

}


//----------------------------------------------------
// - display Audio Source Mux output on OLEDRGB
//----------------------------------------------------
void odmkAudioMuxScreen(void *oledrgbPtr, int muxSel)
{
	char ch;

	/* output to screen */
	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");
	OLEDrgb_SetCursor(oledrgbPtr, 2, 4);
	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(255, 0, 0)); // blue font
	OLEDrgb_PutString(oledrgbPtr, "*XODMK*");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 200, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 6);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
	switch(muxSel) {
		case 0:
			printf("AudioMux screen: <<OSC SIN>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SIN>>");
			break;

		case 1:
			printf("AudioMux screen: <<OSC SAW>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SAW>>");
			break;

		case 2:
			printf("AudioMux screen: <<OSC SQR>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SQR>>");
			break;

		case 3:
			printf("AudioMux screen: <<OSC PWM>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC PWM>>");
			break;

		case 4:
			printf("AudioMux screen: <<EXTERN>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<EXTERN>>");
			break;

		case 8:
			printf("AudioMux screen: <<DDL SIN>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SIN>>");
			break;

		case 9:
			printf("AudioMux screen: <<DDL SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SAW>>");
			break;

		case 10:
			printf("AudioMux screen: <<DDL SQR>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SQR>>");
			break;

		case 11:
			printf("AudioMux screen: <<DDL PWM>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL PWM>>");
			break;

		case 12:
			printf("AudioMux screen: <<DDL EXT>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL EXT>>");
			break;

		case 16:
			printf("AudioMux screen: <<SSB SIN>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SIN>>");
			break;

		case 17:
			printf("AudioMux screen: <<SSB SAW>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SAW>>");
			break;

		case 18:
			printf("AudioMux screen: <<SSB SQR>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SQR>>");
			break;

		case 19:
			printf("AudioMux screen: <<SSB PWM>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB PWM>>");
			break;

		case 20:
			printf("AudioMux screen: <<SSB EXT>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB EXT>>");
			break;

		case 24:
			printf("AudioMux screen: <<FLT SAW>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT SAW>>");
			break;

		case 25:
			printf("AudioMux screen: <<FLT SQR>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT SQR>>");
			break;

		case 26:
			printf("AudioMux screen: <<FLT PWM>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT PWM>>");
			break;

		case 27:
			printf("AudioMux screen: <<FLT EXT>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT EXT>>");
			break;

		case 28:
			printf("AudioMux screen: <<FLT EXT>> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT EXT>>");
			break;

		default:
			printf("AudioMux screen: <DEFAULT> MuxSel = %d\n\r", muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<DEFAULT>");
			break;

	}
}


//----------------------------------------------------
// - display Fader Target on OLEDRGB
//----------------------------------------------------
void odmkFaderMuxScreen(void *oledrgbPtr, u8 faderSel, u8 faderNum)
{
	char ch;

	/* output to screen */
	char FADER_0ScrnStr[12]  = "<MSTR VOL>";
	char FADER_1ScrnStr[12]  = "<SSB FREQ>";
	char FADER_2ScrnStr[12]  = "<MOOG CUT>";
	char FADER_3ScrnStr[12]  = "<MOOG RES>";
	char FADER_4ScrnStr[12]  = "<SEM CUT>";
	char FADER_5ScrnStr[12]  = "<SEM Q>";
	char FADER_6ScrnStr[12]  = "<WS GAIN>";
	char FADER_7ScrnStr[12]  = "<DDL MIX>";
	char FADER_8ScrnStr[12]  = "<DDL LEN>";
	char FADER_9ScrnStr[12]  = "<DDL FB_G>";
	char FADER_10ScrnStr[12] = "<OSC1 FRQ>";
	char FADER_11ScrnStr[12] = "<OSC2 FRQ>";
	char FADER_12ScrnStr[12] = "<PWM FREQ>";

	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");
	OLEDrgb_SetCursor(oledrgbPtr, 2, 4);
	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(255, 0, 0)); // blue font
	switch(faderNum) {
		case 0:
			OLEDrgb_PutString(oledrgbPtr, "FADER 1");
			break;

		case 1:
			OLEDrgb_PutString(oledrgbPtr, "FADER 2");
			break;

		case 2:
			OLEDrgb_PutString(oledrgbPtr, "FADER 3");
			break;

		case 3:
			OLEDrgb_PutString(oledrgbPtr, "FADER 3");
			break;

		default:
			break;
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 200, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 6);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
	switch(faderSel) {
		case 0:
			printf("FaderMux screen: <<MSTR VOL>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_0ScrnStr);
			break;

		case 1:
			printf("FaderMux screen: <<SSB FREQ>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_1ScrnStr);
			break;

		case 2:
			printf("FaderMux screen: <<MOOG CUT>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_2ScrnStr);
			break;

		case 3:
			printf("FaderMux screen: <<MOOG RES>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_3ScrnStr);
			break;

		case 4:
			printf("FaderMux screen: <<SEM CUT>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_4ScrnStr);
			break;

		case 5:
			printf("FaderMux screen: <<SEM Q>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_5ScrnStr);
			break;

		case 6:
			printf("FaderMux screen: <<WS GAIN>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_6ScrnStr);
			break;

		case 7:
			printf("FaderMux screen: <<DDL MIX>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_7ScrnStr);
			break;

		case 8:
			printf("FaderMux screen: <<DDL LEN>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_8ScrnStr);
			break;

		case 9:
			printf("FaderMux screen: <<DDL FB_G>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_9ScrnStr);
			break;

		case 10:
			printf("FaderMux screen: <<OSC1 FRQ>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_10ScrnStr);
			break;

		case 11:
			printf("FaderMux screen: <<OSC2 FRQ>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_11ScrnStr);
			break;

		case 12:
			printf("FaderMux screen: <<OSC FREQ>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_12ScrnStr);
			break;

		default:
			printf("FaderMux screen: <<MSTR VOL>> MuxSel = %d\n\r", faderSel);
			OLEDrgb_PutString(oledrgbPtr, FADER_0ScrnStr);
			break;

	}
}


void faderScrnPrint(void *oledrgbPtr, char *faderSelStr, float faderVal) {

	char faderValStr[12];
	ftoa(faderValStr, faderVal, 1);

	//OLEDrgb_SetCursor(oledrgbPtr, 2, 4);
	OLEDrgb_SetCursor(oledrgbPtr, 1, 4);
	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(255,  0, 0)); // blue font
	OLEDrgb_PutString(oledrgbPtr, faderSelStr);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 200, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 6);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
	OLEDrgb_PutString(oledrgbPtr, faderValStr);

}


void faderVal2Screen(void *oledrgbPtr, u8 faderSel, float faderVal, u8 faderNum)
{

	//temp
	printf("faderVal2Screen: faderSel = %d,  faderNum = %d\n\r", (int)faderSel, (int)faderNum);

	char ch;

	char faderNumStr[2];
	ftoa(faderNumStr, (float)faderNum, 1);

	char FADER_Num[12] = "FADER ";
	strcat(FADER_Num, faderNumStr);

	char FADER_0ScrnStr[12]  = "<MSTR VOL>";
	char FADER_7ScrnStr[12]  = "<DDL MIX>";
	char FADER_8ScrnStr[12]  = "<DDL LEN>";
	char FADER_9ScrnStr[12]  = "<DDL FB_G>";
	char FADER_1ScrnStr[12]  = "<SSB FREQ>";
	char FADER_2ScrnStr[12]  = "<MOOG CUT>";
	char FADER_3ScrnStr[12]  = "<MOOG RES>";
	char FADER_4ScrnStr[12]  = "<SEM CUT>";
	char FADER_5ScrnStr[12]  = "<SEM Q>";
	char FADER_6ScrnStr[12]  = "<WS GAIN>";
	char FADER_10ScrnStr[12] = "<OSC1 FRQ>";
	char FADER_11ScrnStr[12] = "<OSC2 FRQ>";
	char FADER_12ScrnStr[12] = "<PWM FREQ>";


	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, FADER_Num);

	// 0: MSTRVOL, 1: DDLLENGTH, 2: DDLMIX, 3: DDLFBGAIN, 4: SSBMODFreq1

	switch(faderSel) {
		case 0:
			//printf("Fader1Mux screen: <MSTR VOL> MuxSel = %d\n\r", fader1Sel);
			faderScrnPrint(oledrgbPtr, FADER_0ScrnStr, faderVal);
			break;

		case 1:
			//printf("Fader1Mux screen: <DDL LEN> MuxSel = %d\n\r", fader1Sel);
			faderScrnPrint(oledrgbPtr, FADER_8ScrnStr, faderVal);
			break;

		case 2:
			//printf("Fader1Mux screen: <DDL MIX> MuxSel = %d\n\r", fader1Sel);
			faderScrnPrint(oledrgbPtr, FADER_7ScrnStr, faderVal);
			break;

		case 3:
			//printf("Fader1Mux screen: <DDL FB_G> MuxSel = %d\n\r", fader1Sel);
			faderScrnPrint(oledrgbPtr, FADER_9ScrnStr, faderVal);
			break;

		case 4:
			//printf("Fader1Mux screen: <SSB FREQ> MuxSel = %d\n\r", fader1Sel);
			faderScrnPrint(oledrgbPtr, FADER_1ScrnStr, faderVal);
			break;

//		case 5:
//			//printf("Fader1Mux screen: <<PWM FREQ>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_5ScrnStr, faderVal);
//			break;
//
//		case 6:
//			//printf("Fader1Mux screen: <<SSB FREQ>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_6ScrnStr, faderVal);
//			break;
//
//		case 7:
//			//printf("Fader1Mux screen: <<FLT CUT>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_7ScrnStr, faderVal);
//			break;
//
//		case 8:
//			//printf("Fader1Mux screen: <<FLT RES>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_8ScrnStr, faderVal);
//			break;
//
//		case 9:
//			//printf("Fader1Mux screen: <<FLT CUT>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_2ScrnStr, faderVal);
//			break;
//
//		case 10:
//			//printf("Fader1Mux screen: <<FLT RES>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_10ScrnStr, faderVal);
//			break;
//
//		case 11:
//			//printf("Fader1Mux screen: <<WS GAIN>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_11ScrnStr, faderVal);
//			break;
//
//		case 12:
//			//printf("Fader1Mux screen: <<WS GAIN>> MuxSel = %d\n\r", fader1Sel);
//			faderScrnPrint(oledrgbPtr, FADER_12ScrnStr, faderVal);
//			break;

		default:
			//printf("Fader1Mux screen: <MSTR VOL> MuxSel = %d\n\r", fader1Sel);
			faderScrnPrint(oledrgbPtr, FADER_0ScrnStr, faderVal);
			break;

	}
}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

//----------------------------------------------------
// INITIAL SETUP FUNCTIONS
//----------------------------------------------------

void OLEDrgbInitialize()
{
	OLEDrgb_begin(&oledrgb, XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR, XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR);
}


int InterruptSystemSetup(XScuGic *XScuGicInstancePtr, XGpio *GpioInstancePtr, int INT_CH_MASK)
{
	// Enable interrupt
	XGpio_InterruptEnable(GpioInstancePtr, INT_CH_MASK);
	XGpio_InterruptGlobalEnable(GpioInstancePtr);

	/* Connect the interrupt controller interrupt handler to the hardware
	* interrupt handling logic in the ARM processor. */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 	 	 	 	 	 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
			 	 	 	 	 	 XScuGicInstancePtr);

	/* Enable interrupts in the ARM */
	Xil_ExceptionEnable();

	return XST_SUCCESS;

}


int IntcGpioInitFunction(u16 DeviceId, XGpio *GpioInstancePtr, Xil_ExceptionHandler X_Intr_Handler, int INT_X_ID, int INT_CH_MASK)
{
	XScuGic_Config *IntcConfig;
	int status;

	// Interrupt controller driver initialization
	IntcConfig = XScuGic_LookupConfig(DeviceId);
	status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress);
	if(status != XST_SUCCESS) return XST_FAILURE;

	// Call to interrupt setup
	//status = InterruptSystemSetup(&INTCInst, &BTNInst, INT_CH_MASK);
	status = InterruptSystemSetup(&INTCInst, GpioInstancePtr, INT_CH_MASK);
	if(status != XST_SUCCESS) return XST_FAILURE;

	// Connect GPIO interrupt to device driver handler
	status = XScuGic_Connect(&INTCInst,
							 INT_X_ID,
					  	  	 (Xil_ExceptionHandler)X_Intr_Handler,
					  	  	 (void *)GpioInstancePtr);


	if(status != XST_SUCCESS) return XST_FAILURE;

	// Enable GPIO interrupts interrupt
	XGpio_InterruptEnable(GpioInstancePtr, 1);
	XGpio_InterruptGlobalEnable(GpioInstancePtr);

	// Enable GPIO and timer interrupts in the controller
	XScuGic_Enable(&INTCInst, INT_X_ID);

	return XST_SUCCESS;
}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

int main(void)
{

	int status;

	init_platform();

	printf("::((Zedboard Audio System))::\n\r");


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // LED & Switch AXI GPIO Initialization
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize Push Buttons (built-in)
    status = XGpio_Initialize(&BTNInst, GPIO_BTNS_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;
    // Set all buttons direction to inputs
    XGpio_SetDataDirection(&BTNInst, 1, 0xFF);

    // Initialize Push Buttons interrupt controller
    INTC_X_INTERRUPT_ID = INTC_BTNS_INTERRUPT_ID;
    INT_CH_MASK = BTN_CH_MASK;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &BTNInst, &BTN_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;


    // Initialize BTN0x1 Push Buttons (Pmod)
//    status = XGpio_Initialize(&BTN0x1Inst, GPIO_BTN0x1_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;
//    // Set all buttons direction to inputs
//    XGpio_SetDataDirection(&BTN0x1Inst, 1, 0xFF);
//
//    // Initialize Push Buttons interrupt controller
//    INTC_X_INTERRUPT_ID = INTC_BTN0x1_INTERRUPT_ID;
//    INT_CH_MASK = BTN_CH_MASK;
//    status = IntcGpioInitFunction(INTC_DEVICE_ID, &BTN0x1Inst, &BTN0x1_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
//    if(status != XST_SUCCESS) return XST_FAILURE;


    // Initialize LEDs & Switches
    status = XGpio_Initialize(&LEDSWSInst, GPIO_LED_SW_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;
    // Set LEDs direction to outputs, SW to inputs
    XGpio_SetDataDirection(&LEDSWSInst, 1, 0x00);
    XGpio_SetDataDirection(&LEDSWSInst, 2, 0xFF);



    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // AXI Timer Initialization
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    status = XTmrCtr_Initialize(&TIMERInst, TMRCTR_TIMER_ID);
//	if(XST_SUCCESS != status)
//		print("TIMER INIT FAILED \n\r");
//	// Set Timer Handler
//	//	XTmrCtr_SetHandler(&TIMERInst, TIMER_Intr_Handler, &TIMERInst);
//	//	//XTmrCtr_SetHandler(&TIMERInst, &XTmrCtr_InterruptHandler, &TIMERInst);
//	// Set timer Reset Value
//	XTmrCtr_SetResetValue(&TIMERInst, 0, 0xf0000000);	//Change with generic value
//	// Set timer Option (Interrupt Mode And Auto Reload )
//	XTmrCtr_SetOptions(&TIMERInst, TMRCTR_TIMER_ID, (XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION ));


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // AXI-Lite Control Initialization
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// Set correction positions frame memory address in Undistort IP
	if(XXodefx_top_Initialize(&XodAXICtrl, AXI_CTRL_ID) == XST_FAILURE) {
		xil_printf("Failed to initialize AXI-Lite Control Bus.\r\n");
		return XST_FAILURE;
	}

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize Oscillators
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    // Initialize DDS Freq1 & SSB setStep Gpio
    status = XGpio_Initialize(&SSBMODFreq1Inst, GPIO_SSBMODFreq1_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

//    // Initialize DDS Freq2 & PWM setStep Gpio
//    status = XGpio_Initialize(&OSCF2PWMStepInst, GPIO_OSCF2_PWM_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize Filters
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Moog Half Ladder Filter

//    // Initialize Moog Half Ladder CTRL 1 Gpio
//    status = XGpio_Initialize(&MHLALPHAKInst, GPIO_MHL_ALPHA_K_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;
//
//    // Initialize Moog Half Ladder CTRL 1 Gpio
//    status = XGpio_Initialize(&MHLCTRL1Inst, GPIO_MHL_PARAM1_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;
//
//    // Initialize Moog Half Ladder CTRL 2 Gpio
//    status = XGpio_Initialize(&MHLCTRL2Inst, GPIO_MHL_PARAM2_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;
//
//    // SEM State Variable Filter
//
//    // Initialize SEM SVF CTRL 1 Gpio
//    status = XGpio_Initialize(&SEMCTRL1Inst, GPIO_SEMALPHA_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;
//
//    // Initialize SEM SVF CTRL 2 Gpio
//    status = XGpio_Initialize(&SEMCTRL2Inst, GPIO_SEMRHO_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize Delay
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize DDL CTRL Gpio
    status = XGpio_Initialize(&DDLCTRLInst, GPIO_DDLCTRL_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize ..others..
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize Volume Control Gpio
    status = XGpio_Initialize(&VOLWSGAINInst, GPIO_VOLWSGAIN_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

//    // Initialize Audio Mux Gpio
//    status = XGpio_Initialize(&AUDIOMUXInst, GPIO_AUDIOMUX_ID);
//    if(status != XST_SUCCESS) return XST_FAILURE;


    // Initialize Rotary Encoders

    // __Rotary Encoder 1__
    // Initialize Rotary Encoder 1 Gpio
    status = XGpio_Initialize(&ROTARY1Inst, GPIO_ROTARY1_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 1 interrupt controller
    INTC_X_INTERRUPT_ID = INTC_ROTARY1_INTERRUPT_ID;
    INT_CH_MASK = ROTARY_INT;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &ROTARY1Inst, &ROTARY1_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // __Rotary Encoder 2__
    // Initialize Rotary Encoder 2 Gpio
    status = XGpio_Initialize(&ROTARY2Inst, GPIO_ROTARY2_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 2 interrupt controller
    INTC_X_INTERRUPT_ID = INTC_ROTARY2_INTERRUPT_ID;
    INT_CH_MASK = ROTARY_INT;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &ROTARY2Inst, &ROTARY2_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // __Rotary Encoder 3__
    // Initialize Rotary Encoder 3 Gpio
    status = XGpio_Initialize(&ROTARY3Inst, GPIO_ROTARY3_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 3 interrupt controller
    INTC_X_INTERRUPT_ID = INTC_ROTARY3_INTERRUPT_ID;
    INT_CH_MASK = ROTARY_INT;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &ROTARY3Inst, &ROTARY3_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // __Rotary Encoder 4__
    // Initialize Rotary Encoder 4 Gpio
    status = XGpio_Initialize(&ROTARY4Inst, GPIO_ROTARY4_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 3 interrupt controller
    INTC_X_INTERRUPT_ID = INTC_ROTARY4_INTERRUPT_ID;
    INT_CH_MASK = ROTARY_INT;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &ROTARY4Inst, &ROTARY4_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;



    // Initialize OLEDrgb display
    OLEDrgbInitialize();


	Xil_ICacheEnable();
	Xil_DCacheEnable();


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // INITIALIZE Power-on System Behavior
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	printf("\n::((Parameter Initialization...))::\n");

	// 1us = 100000

	BPM = 136.0;	// Default BPM
	uSECPERBEAT = (unsigned long)(1000000*60/BPM);

	odmkInfoScreen(&oledrgb, BPM);


	// Default Fader Target settings
	// 0: MSTRVOL, 1: DDLLENGTH, 2: DDLMIX, 3: DDLFBGAIN, 4: SSBMODFreq1
	fader1MuxSel = 4;
	fader2MuxSel = 1;
	fader3MuxSel = 3;
	fader4MuxSel = 0;


	// **Default Parameter Init**
	// outputGain = 1.86;
	// wetMix = 0.9;
	// selectExtFB = 1;
	// feedbackMix = 0.95;
	// dlyLength = 1760.5;
	// selectCross = 1;

	volCtrlVal = 60948;                                                   // 1.86 << 15
	XGpio_DiscreteWrite(&VOLWSGAINInst, 1, volCtrlVal);
	//XGpio_DiscreteWrite(&VOLWSGAINInst, 2, wsGainVal);

	// TEMP DEBUG
	printf("\n::((Update VOLWSGAINInst volCtrlVal))::\n");

	// init output audio mux to EXT
	//auSrcMuxUnion.audioSrcSelect.audioSrcMux = 4;
	//XGpio_DiscreteWrite( &AUDIOMUXInst, 1, (auSrcMuxUnion.audioSrcMuxSel<<14) + (16383&auMuxUnion.audioMuxSel) );
	//auMuxUnion.audioSelect.flt6Mux = 0;
	//auMuxUnion.audioSelect.flt4Mux = 0;
	//auMuxUnion.audioSelect.audioSrcMux = 4;
	//XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);

	// OSC INIT
	XGpio_DiscreteWrite(&SSBMODFreq1Inst, 1, ssbSetStepVal);
	//XGpio_DiscreteWrite(&OSCF1SSBStepInst, 2, ssbSetStepVal);

	//XGpio_DiscreteWrite(&OSCF2PWMStepInst, 1, oscFreq2StepVal);
	//XGpio_DiscreteWrite(&OSCF2PWMStepInst, 2, oscPWMStepVal);

	// DDL INIT
	DDLCTRL1Union.DDLCTRL1.wetMix = 29491;									// 0.9 << 15
	DDLCTRL1Union.DDLCTRL1.feedbackGain = 31129;							// 0.95 << 15
	XGpio_DiscreteWrite(&DDLCTRLInst, 1, DDLCTRL1Union.DDLCTRL_CH1Val);

	DDLCTRL2Union.DDLCTRL2.delayLength = 450688;
	XGpio_DiscreteWrite(&DDLCTRLInst, 2, DDLCTRL2Union.DDLCTRL_CH2Val);

	// TEMP DEBUG
	printf("\n::((Update DDLCTRLInst wetMix, delayLength))::\n");

	// MOOG Half Ladder Filter INIT
	//setFcAndRes_ARM(fs, mooghlCutoff, 0.0, &mooghlParam);
	//XGpio_DiscreteWrite(&MHLALPHAKInst, 1, mooghlParam.alpha);
	//XGpio_DiscreteWrite(&MHLALPHAKInst, 2, mooghlParam.K);
	//XGpio_DiscreteWrite(&MHLCTRL1Inst, 1, mooghlParam.alpha0);
	//XGpio_DiscreteWrite(&MHLCTRL1Inst, 2, mooghlParam.beta1);
	//XGpio_DiscreteWrite(&MHLCTRL2Inst, 1, mooghlParam.beta2);
	//XGpio_DiscreteWrite(&MHLCTRL2Inst, 2, mooghlParam.beta3);

	//printf("MOOG Half Ladder Initial Parameters:\n");
	//printf("moog-hl Cutoff = %6.2f,   moog-hl Resonance = %2.2f\n", mooghlCutoff, mooghlRes);
	//printf("alpha = %i,  K = %i\n", mooghlParam.alpha, mooghlParam.K);
	//printf("alpha0 = %i,  beta1 = %i\n", mooghlParam.alpha0, mooghlParam.beta1);
	//printf("beta2 = %i,  beta3 = %i\n", mooghlParam.beta2, mooghlParam.beta3);


	// SEM State Variable Filter INIT
	//semQscaled = odmkSEMSVF_setQ_ref(semQ);
	//odmkSEMSVF_update(fs, semCutoff, semQscaled, semlParam.alpha0, semlParam.alpha, semlParam.rho);
	//XGpio_DiscreteWrite(&SEMCTRL1Inst, 1, semlParam.alpha0);
	//XGpio_DiscreteWrite(&SEMCTRL1Inst, 2, semlParam.alpha);
	//XGpio_DiscreteWrite(&SEMCTRL2Inst, 1, semlParam.rho);

	//printf("SEM State Variable Filter Initial Parameters:\n");
	//printf("sem-svf Cutoff = %6.2f,   sem-svf Q = %2.2f\n", semCutoff, semQ);

	/*------------------------------------------------------------------------------------------------*/


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Begin Master Routine
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1) {

		if (sysmode == 0) {
			/* Write Info to OLEDRGB - center button */

			if (trigOnce == 1) {

				//btnDirection = !btnDirection;

				// format OSC freq value for printing
				//oscFreq1 = 100000000.0 * (float)oscFreq1StepVal / pow(2, hirezPhaseAccWidth);
				//oscFreq2 = 100000000.0 * (float)oscFreq2StepVal / pow(2, lorezPhaseAccWidth);

				printf("// *-------------------------------------------------* //\n");
				printf("\n::((XODMK xodZedAudio 2022))::\n");
				//printf("sysmode = %d:\n",sysmode);
				//printf("Zedboard Button Function:\n");
				//printf("Center: Top State <sysmode == 0>\n");
				//printf("Bottom: Empty Function\n");
				//printf("Left: Empty Function\n");
				//printf("Right: Empty Function\n");
				//printf("Top: Empty Function\n");

				printf("// *-------------------------------------------------* //\n");
				printf("BPM = %3.2f\n", BPM);
				printf("volCtrlDB = %5.2f,   volCtrlVal = %d\n", volCtrlDB, (int)volCtrlVal);
//				printf("OSC Freq1 = %5.2f,   OSC Freq1 (LFO) = %5.2f\n", oscFreq1, oscFreq2);
//				printf("oscFreq1StepVal = %d,   oscFreq2StepVal = %d\n", oscFreq1StepVal, oscFreq2StepVal);
//				printf("MOOG Half Ladder Filter Parameters:\n");
//				printf("moog-hl Cutoff = %6.2f,   moog-hl Resonance = %2.2f\n", mooghlCutoff, mooghlRes);
//				printf("SEM State Variable Filter Parameters:\n");
//				printf("sem-svf Cutoff = %6.2f,   sem-svf Q = %2.2f\n", semCutoff, semQ);
				printf("ddlWetMixFloat = %2.2f,   wetMix = %d\n", ddlWetMixFloat, (int)DDLCTRL1Union.DDLCTRL1.wetMix);
				printf("delayLength = %d\n", (int)DDLCTRL2Union.DDLCTRL2.delayLength);
				printf("ddlFBGainFloat = %3.2f,   feedbackGain = %d\n", ddlFBGainFloat, (int)DDLCTRL1Union.DDLCTRL1.feedbackGain);
//				printf("waveshaperGainFloat= %4.2f,   waveshaperGain = %d\n", wsGainFloat, (int)wsGainVal);

				printf("// *-------------------------------------------------* //\n");
				printf("--Switch System Update--\n");
				printf("All Switch - swsUnion.swValue = %d\n",(int)swsUnion.swValue);
				printf("Switches[8:1]:   %d %d %d %d %d %d %d %d\n",
						swsUnion.switches.bypass_sw1,
						swsUnion.switches.multiChan_sw2,
						swsUnion.switches.wavShaperOn_sw3,
						swsUnion.switches.selectCross_sw4,
						swsUnion.switches.selectHouse_sw5,
						swsUnion.switches.selectSSB_sw6,
						swsUnion.switches.lfoUpDn_sw7,
						swsUnion.switches.lfoSync_sw8);

//				printf("audioSelect - CH1_outMux = %d\n", auMuxUnion.audioSelect.CH1_outMux);
//				printf("audioSelect - CH2_outMux = %d\n", auMuxUnion.audioSelect.CH2_outMux);
//				printf("audioSelect - wsInMux = %d\n", auMuxUnion.audioSelect.wsInMux);
//				printf("audioSelect - ddlInMux = %d\n", auMuxUnion.audioSelect.ddlInMux);
//				printf("audioSelect - ddlFBSrcSel = %d\n", auMuxUnion.audioSelect.ddlFBSrcSel);
//				printf("audioSelect - ddlExtFBSel = %d\n", auMuxUnion.audioSelect.ddlExtFBSel);
//				printf("audioSelect - fltInMux = %d\n", auMuxUnion.audioSelect.fltInMux);
//				printf("audioSelect - ssbInSel = %d\n", auMuxUnion.audioSelect.ssbInSel);
//				printf("audioSelect - flt6Mux = %d\n", auMuxUnion.audioSelect.flt6Mux);
//				printf("audioSelect - flt4Mux = %d\n", auMuxUnion.audioSelect.flt4Mux);
//				printf("audioSelect - audioSrcMux = %d\n", auMuxUnion.audioSelect.audioSrcMux);

				//printf("audioMuxSel = %d\n", (int)auMuxUnion.audioMuxSel);

				OLEDrgb_Clear(&oledrgb);
				odmkInfoScreen(&oledrgb, BPM);
				trigOnce = 0;

				usleep(3300000);	//Wait a moment, then switch to chebyshev poly's
			}

			odmkZynqLED1(uSECPERBEAT);		// *** may have to adjust timing for bpm sync???

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon01_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon02_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon03_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon04_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon05_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon06_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			//OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)masakiKmno01);
			//usleep(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			// wait for button interrupt to change state

		}

		else if (sysmode == 1) {
			/* increment audio mux select - right button */
			if (trigOnce==1) {
				printf("sysmode = %d:   Increment Audio Mux Select\n",sysmode);
				/* Update OLED display */
				//OLEDrgb_Clear(&oledrgb);
				//odmkAudioMuxScreen(&oledrgb, auMuxUnion.audioSelect.audioSrcMux);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 2) {
			/* increment fader 1 mux select - top button */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader 1 Mux Select = %d\n", sysmode, (int)fader1MuxSel);
				/* Update OLED display */
				//OLEDrgb_Clear(&oledrgb);
				//odmkFaderMuxScreen(&oledrgb, fader1MuxSel, (u8)0);
				//odmkFader1MuxScreen(&oledrgb, fader1MuxSel);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 3) {
			/* increment Scene Selection - bottom button */
			if (trigOnce == 1) {
				//printf("sysmode {%d}:   Scene Select = %d\n", sysmode, (int)sceneSel);


				//OLEDrgb_Clear(&oledrgb);
				//odmkFaderMuxScreen(&oledrgb, fader2MuxSel, (u8)1);

			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 4) {
			/* increment fader 3 mux select - left button */
			if (trigOnce == 1) {
				//printf("sysmode {%d}:   Fader 3 Mux Select = %d\n",sysmode, (int)fader2MuxSel);
				//OLEDrgb_Clear(&oledrgb);
				//odmkFaderMuxScreen(&oledrgb, fader3MuxSel, (u8)2);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 5) {
			/* increment fader 4 mux select - left button */
			if (trigOnce == 1) {
				//printf("sysmode {%d}:   Fader 3 Mux Select = %d\n",sysmode, (int)fader2MuxSel);
				//OLEDrgb_Clear(&oledrgb);
				//odmkFaderMuxScreen(&oledrgb, fader4MuxSel, (u8)3);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 6) {
			/* update OLED - fader 1 Val */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader 1 Select = %d\n", sysmode, (int)fader1Lvl);
				OLEDrgb_Clear(&oledrgb);
				faderVal2Screen(&oledrgb, fader1MuxSel, fader1Lvl, (u8)1);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 7) {
			/* update OLED - fader 2 Val */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader 2 Val = %d\n", sysmode, (int)fader2Lvl);
				/* Update OLED display */
				OLEDrgb_Clear(&oledrgb);
				faderVal2Screen(&oledrgb, fader2MuxSel, fader2Lvl, (u8)2);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 8) {
			/* update OLED - fader 3 Val */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader 3 Val = %d\n", sysmode, (int)fader3Lvl);
				/* Update OLED display */
				OLEDrgb_Clear(&oledrgb);
				faderVal2Screen(&oledrgb, fader3MuxSel, fader3Lvl, (u8)3);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

		else if (sysmode == 9) {
			/* update OLED - fader 4 Val */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader 4 Val = %d\n", sysmode, (int)fader4Lvl);
				/* Update OLED display */
				OLEDrgb_Clear(&oledrgb);
				faderVal2Screen(&oledrgb, fader4MuxSel, fader4Lvl, (u8)4);
			}
			trigOnce = 0;
			// wait for button interrupt to change state
		}

	}

	cleanup_platform();

	return 0;
}
