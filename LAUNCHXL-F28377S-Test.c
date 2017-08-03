//###########################################################################
//
// FILE:   adc_soc_continuous_cpu01.c
//
// TITLE:  ADC continuous self-triggering for F2837xS.
//
//! \addtogroup cpu01_example_list
//! <h1> ADC Continuous Triggering (adc_soc_continuous)</h1>
//!
//! This example sets up the ADC to convert continuously, achieving maximum
//! sampling rate.\n
//!
//! After the program runs, the memory will contain:
//!
//! - \b AdcaResults \b: A sequence of analog-to-digital conversion samples
//! from pin A0. The time between samples is the minimum possible based on the
//! ADC speed.
//
//###########################################################################
// $TI Release: F2837xS Support Library v210 $
// $Release Date: Tue Nov  1 15:35:23 CDT 2016 $
// $Copyright: Copyright (C) 2014-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <file.h>

#include "F28x_Project.h"     // DSP28x Headerfile
#include "sci_io.h"
#include "sgen.h"

//
// Defines
//

// Timers
#define CPU_TIMER0_PERIOD 10 // us
#define CPU_TIMER1_PERIOD 1 // us
#define CPU_TIMER2_PERIOD 1 // us

// GPIO Pins
#define LED_GPIO_RED    12
#define LED_GPIO_BLUE	13

#define PWM1_GPIO	12
#define PWM2_GPIO	13
#define PWM3_GPIO	14
#define PWM4_GPIO	15

// Over-voltage / Over-current Limit
#define VDC1_LIMIT 9.0
#define VDC2_LIMIT 9.0
#define VDC12_LIMIT 20.0

#define IL1_LIMIT 3.0

#define EPWM7B_MIN_DUTY 0.2
#define EPWM7B_MAX_DUTY 0.7

#define V2AB_LIMIT 9.0

// Analog to Digital (ADC)
#define MOV_AVG_SIZE_HI 256
#define MOV_AVG_SIZE 128

#define ACQPS_TIME 88
#define EPWM7_DUTY_UPPER 0.5
#define EPWM7_DUTY_LOWER 0.1
#define EPWM7_INC 0.02

enum ADCINID {
	ADCINID_A0 = 0x00,
	ADCINID_A1 = 0x01,
	ADCINID_A2 = 0x02,
	ADCINID_A3 = 0x03,
	ADCINID_A4 = 0x04,
	ADCINID_A5 = 0x05,
	ADCINID_14 = 0x0E,
	ADCINID_15 = 0x0F,

	ADCINID_B0 = 0x10,
	ADCINID_B1 = 0x11,
	ADCINID_B2 = 0x12,
	ADCINID_B3 = 0x13,
	ADCINID_B4 = 0x14,
	ADCINID_B5 = 0x15
};

// EPWM
#define EPWMCLK 200000000 // 200MHz
#define DB_TIME 0.000005 // 5uS

#define HYSTERESIS_BAND 0.05

// DAC
#define DLOG_SIZE             1024
#define REFERENCE_VDAC        0
#define REFERENCE_VREF        1
#define LOW_THD_SINE          0
#define HIGH_PRECISION_SINE   1
#define DACA                  1
#define DACB                  2
#define DACC                  3
#define SINEWAVE_TYPE         LOW_THD_SINE
#define REFERENCE             REFERENCE_VREF
#define CPUFREQ_MHZ           200
#define DAC_NUM               DACA

//
// Globals
//

// FILE -> printf variables
volatile int status = 0;
volatile FILE *fid;

// GPIO LED pin states
char blueLED_state = 0;
char redLED_state = 0;

// GPIO PWM pin states
char PWM1_state = 0;
char PWM2_state = 0;
char PWM3_state = 0;
char PWM4_state = 0;

// Hysteresis Limits
float CMPHI = 8.0;
float CMPLO = -8.0;
char up = 1;

//
// Analog to Digital (ADC)
//

// Digital Values
// J1 and J3
uint16_t SGND51R_ADC = 0; // Short to SGND through 51R
uint16_t VC4_ADC = 0;
uint16_t V2AB_ADC = 0;
uint16_t VC3_ADC = 0;
uint16_t V1AB_ADC = 0;
uint16_t IL4_ADC = 0;
uint16_t IL3_ADC = 0;

// J5 and J7
uint16_t VDC2_ADC = 0;
uint16_t VDC1_ADC = 0;
uint16_t IL2_ADC = 0;
uint16_t SGND_ADC = 0; // Short to SGND
uint16_t S1V5_ADC = 0; // Short to 1.5V
uint16_t ANALOG_ADC = 0;
uint16_t IL1_ADC = 0;

// Actual Values
uint16_t buf_idx = 0;
uint16_t buf_idx_hi = 0;
float VC4_Real = 0;

float V1AB_Real = 0;
float V1AB_Real_Buf[MOV_AVG_SIZE];
float V1AB_Real_BufSum = 0;
float V1AB_Real_Avg = 0;

float V2AB_Real = 0;
float V2AB_Real_Buf[MOV_AVG_SIZE];
float V2AB_Real_BufSum = 0;
float V2AB_Real_Avg = 0;

float VC3_Real = 0;
float IL4_Real = 0;
float IL3_Real = 0;

// J5 and J7
float VDC1_Real = 0;
float VDC1_Real_Buf[MOV_AVG_SIZE];
float VDC1_Real_BufSum = 0;
float VDC1_Real_Avg = 0;

float VDC2_Real = 0;
float VDC2_Real_Buf[MOV_AVG_SIZE];
float VDC2_Real_BufSum = 0;
float VDC2_Real_Avg = 0;

float VDC12_Real_Avg = 0;

float ANALOG_Real = 0;

float IL1_Real = 0;
float IL1_Real_Buf[MOV_AVG_SIZE_HI];
float IL1_Real_BufSum = 0;
float IL1_Real_Avg = 0;

unsigned int IL1_Ref = 0;
float IL1_Hyst_SetPoint = 2.0;

float IL2_Real = 0;
float IL2_Real_Buf[MOV_AVG_SIZE_HI];
float IL2_Real_BufSum = 0;
float IL2_Real_Avg = 0;
unsigned int IL2_Ref = 0;

// EPWM
double EPWM7_Freq = 10000.0 * 2.0;
double EPWM7_Duty = 0.5;

int PWM7_seekUpperBand = 2;

double EPWM8_Freq = 10000.0 * 2.0;
double EPWM8_Duty = 0.6;

char EPWM7_Up = 1;

// DAC
Uint16 DataLog[DLOG_SIZE];
volatile struct DAC_REGS* DAC_PTR[4] = { 0x0, &DacaRegs, &DacbRegs, &DaccRegs };
Uint32 samplingFreq_hz = 200000;
Uint32 outputFreqA_hz = 13;
Uint32 outputFreqB_hz = 17;
Uint32 maxOutputFreq_hz = 5000;
float waveformGainA = 0.5; // Range 0.0 -> 1.0
float waveformOffsetA = 0.0;    // Range -1.0 -> 1.0
float waveformGainB = 0.5; // Range 0.0 -> 1.0
float waveformOffsetB = 0.0;    // Range -1.0 -> 1.0

RMPGEN triwave1 = RMPGEN_DEFAULTS; //initialize ramp wave
#if SINEWAVE_TYPE==LOW_THD_SINE //initialize sine wave type
SGENTI_1 sgenIL1a = SGENTI_1_DEFAULTS;
SGENTI_1 sgenIL1b = SGENTI_1_DEFAULTS;
#elif SINEWAVE_TYPE==HIGH_PRECISION_SINE
SGENHP_1 sgenIL1a = SGENHP_1_DEFAULTS;
SGENHP_1 sgenIL1b = SGENHP_1_DEFAULTS;
#endif

Uint16 sgenIL1a_out = 0;
Uint16 sgenIL1b_out = 0;
Uint16 ndx = 0;
float freqResolution_hz = 0;
float cpuPeriod_us = 0;
Uint32 interruptCycles = 0;
float interruptDuration_us = 0;
float samplingPeriod_us = 0;
Uint16 maxOutput_lsb = 0;
Uint16 minOutput_lsb = 0;
Uint16 pk_to_pk_lsb = 0;

//
// Function Prototypes
//

// General
extern void DSP28x_usDelay(Uint32 Count);
void scia_init(void);
void init_LEDs(void);
void init_printf(void);
void printAndyBoard(void);
void toggleBlueLED(void);
void toggleRedLED(void);

// GPIO-PWM Functions
void init_GPIOPWM(void);
void toggleGPIOPWM1(void);
void toggleGPIOPWM2(void);
void toggleGPIOPWM3(void);
void toggleGPIOPWM4(void);

void setGPIOPWM1Freq(double freq);
void setGPIOPWM2Freq(double freq);
void setGPIOPWM3Freq(double freq);
void setGPIOPWM4Freq(double freq);

// Analog to Digital (ADC)
void init_ADC(void);
void convertADCBank(uint16_t id);
void storeADCValues(void);
void movingAvgADC(void);

void haltOverVoltage(void);
void disableEPWM(void);
char checkVDC1(void);
char checkVDC2(void);
char checkVDC12(void);
char checkV2AB(void);

void hysteresisControl(void);

// EPWM
void InitEPwm7Example(void);
void InitEPwm8Example(void);
__interrupt void epwm7_isr(void);
__interrupt void epwm8_isr(void);
void enablePWMBuffer(void);

void PWM7CurrentControl(void);

void PWM7HysteresisControl(void);

// CPU Timers
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);

// DAC
static inline void dlog(Uint16 value);
static inline void setFreq(void);
static inline void setGain(void);
static inline void setOffset(void);
static inline Uint16 getMax(void);
static inline Uint16 getMin(void);
void configureDAC(Uint16 dac_num);
void configureWaveform(void);
interrupt void cpu_timer0_isr(void);

void main(void) {

#ifdef _FLASH
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t) &RamfuncsLoadSize);
#endif

	//
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2837xS_SysCtrl.c file.
	//
	InitSysCtrl();

	//
	// Step 2. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	//
	DINT;

	//
	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F2837xS_PieCtrl.c file.
	//
	InitPieCtrl();

	//
	// Clear all interrupts and initialize PIE vector table:
	//
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();

	// EPWM

	//
	// enable PWM7 and PWM8
	//
	CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
	CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;

	//
	// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
	// These functions are in the F2837xS_EPwm.c file
	//
	InitEPwm7Gpio();
	InitEPwm8Gpio();

	//
	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	//
	EALLOW;
	// This is needed to write to EALLOW protected registers
//	PieVectTable.EPWM7_INT = &epwm7_isr;
//	PieVectTable.EPWM8_INT = &epwm8_isr;

	PieVectTable.TIMER0_INT = &cpu_timer0_isr; // Need this for CPU Timer0
//	PieVectTable.TIMER1_INT = &cpu_timer1_isr; // Need this for CPU Timer1
//	PieVectTable.TIMER2_INT = &cpu_timer2_isr; // Need this for CPU Timer2

	EDIS;
	// This is needed to disable write to EALLOW protected registers

	//
	// Step 4. Initialize the Device Peripherals:
	//
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

	InitEPwm7Example();
	//InitEPwm8Example();

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	// -------------------------
	//
	// Initialize variables
	//
	cpuPeriod_us = (1.0 / CPUFREQ_MHZ);
	samplingPeriod_us = (1000000.0 / samplingFreq_hz);

	//
	// Initialize datalog
	//
	for (ndx = 0; ndx < DLOG_SIZE; ndx++) {
		DataLog[ndx] = 0;
	}
	ndx = 0;

	//
	// Configure DAC
	//
	// configureDAC(DAC_NUM);

	//
	// Configure Waveform
	//
	configureWaveform();
	// --------

	InitCpuTimers();   // For this example, only initialize the Cpu Timers

	// Configure CPU-Timer 0,1,2 to __interrupt
	// ConfigCpuTimer(&CpuTimer0, 200, CPU_TIMER0_PERIOD); // lower than 10us too fast, prevents other ISR from running
	ConfigCpuTimer(&CpuTimer0, CPUFREQ_MHZ, 1000000.0 / samplingFreq_hz);

//	ConfigCpuTimer(&CpuTimer1, 200, CPU_TIMER1_PERIOD);
//	ConfigCpuTimer(&CpuTimer2, 200, CPU_TIMER2_PERIOD);

	//
	// To ensure precise timing, use write-only instructions to write to the entire
	// register. Therefore, if any of the configuration bits are changed in
	// ConfigCpuTimer and InitCpuTimers (in F2837xS_cputimervars.h), the below
	// settings must also be updated.
	//
//	CpuTimer0Regs.TCR.all = 0x4000;
	CpuTimer0Regs.TCR.all = 0x4001;
//	CpuTimer1Regs.TCR.all = 0x4001;
//	CpuTimer2Regs.TCR.all = 0x4001;

	//
	// Step 5. User specific code, enable interrupts:
	//

	//
	// Enable CPU INT1, INT13, INT14 which is connected to CPU-Timer 0, 1, 2:
	//
	IER |= M_INT1; // Enable CPU Timer 0 Interrupts
	// IER |= M_INT13; // Enable CPU Timer 1 Interrupts
	// IER |= M_INT14; // Enable CPU Timer 2 Interrupts

	// Enable Timer0 Interrupt, TINT0 in the PIE: Group 1 __interrupt 7
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	//
	// Enable EPWM INTn in the PIE: Group 3 interrupt 1-11
	//
//	PieCtrlRegs.PIEIER3.bit.INTx7 = 1; // Interrupt on EPWM7
//	PieCtrlRegs.PIEIER3.bit.INTx8 = 1; // Interrupt on EPWM8

	// User Configuration Functions
	// configureLEDs();
	// GPIOGateDrivers();
	init_ADC();
	init_printf();

	EINT;
	// Enable Global interrupt INTM
	ERTM;
	// Enable Global realtime interrupt DBGM

// take conversions indefinitely in loop
	// disableEPWM();

	do {

		setFreq();   // Set Output Frequency and Max Output Frequency
		setGain();   // Set Magnitude of Waveform
		setOffset(); // Set Offset of Waveform
//		maxOutput_lsb = getMax();
//		minOutput_lsb = getMin();
//		pk_to_pk_lsb = maxOutput_lsb - minOutput_lsb;

//		if (EPWM7_Up) {
//			EPWM7_Duty += EPWM7_INC;
//			if (EPWM7_Duty >= EPWM7_DUTY_UPPER)
//				EPWM7_Up = 0;
//		} else {
//			EPWM7_Duty -= EPWM7_INC;
//			if (EPWM7_Duty <= EPWM7_DUTY_LOWER)
//				EPWM7_Up = 1;
//		}
//		EPwm7Regs.CMPA.bit.CMPA = EPWM7_Duty * EPwm7Regs.TBPRD;
//		DELAY_US(500);
	} while (1);
}

void scia_init() {
	// Note: Clocks were turned on to the SCIA peripheral
	// in the InitSysCtrl() function

	SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
									// No parity,8 char bits,
									// async mode, idle-line protocol
	SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
									// Disable RX ERR, SLEEP, TXWAKE

	SciaRegs.SCICTL2.bit.TXINTENA = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

	SciaRegs.SCIHBAUD.all = 0x0000; // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
	SciaRegs.SCILBAUD.all = 53;

	SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset

	return;
}

void init_LEDs() {
	// Pin 12 - Blue LED
	GPIO_SetupPinMux(LED_GPIO_BLUE, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_BLUE, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(LED_GPIO_BLUE, 1);

	// Pin 13 - Red LED
	GPIO_SetupPinMux(LED_GPIO_RED, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_RED, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(LED_GPIO_RED, 1);
}

void init_GPIOPWM() {
	// Ken Drives
	// PWM1+
	InitGpio();
	GPIO_SetupPinMux(PWM1_GPIO, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(PWM1_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(PWM1_GPIO, 0);

	// PWM2+
	GPIO_SetupPinMux(PWM2_GPIO, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(PWM2_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(PWM2_GPIO, 0);

	// PWM3+
	GPIO_SetupPinMux(PWM3_GPIO, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(PWM3_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(PWM3_GPIO, 0);

	// PWM4+
	GPIO_SetupPinMux(PWM4_GPIO, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(PWM4_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(PWM4_GPIO, 0);
}

void init_printf() {
	EALLOW;
	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;
	GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;
	GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;
	GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
	EDIS;

	// Redirect STDOUT to SCI
	scia_init();
	status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write,
			SCI_lseek, SCI_unlink, SCI_rename);
	fid = fopen("scia", "w");
	freopen("scia:", "w", stdout);
	setvbuf(stdout, NULL, _IONBF, 0);
}

void init_ADC() {
	// Initialize Moving Average Arrays
	int idx;
	for (idx = 0; idx < MOV_AVG_SIZE_HI; idx++) {
		if (idx < MOV_AVG_SIZE) {
			V1AB_Real_Buf[idx] = 0;
			V2AB_Real_Buf[idx] = 0;
			VDC1_Real_Buf[idx] = 0;
			VDC2_Real_Buf[idx] = 0;
		}
		IL1_Real_Buf[idx] = 0;
		IL2_Real_Buf[idx] = 0;
	}

	// ADC1 configuation

	// Configure the ADC:
	// Initialize the ADC
	EALLOW;

	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADCs
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for 1ms to allow ADC time to power up
	DELAY_US(1000);

	//ADC Register Configuration
	EALLOW;

	//
	//ADCINA
	//
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x07; //end of SOC7 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//ADCINA0
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x00;  //SOC0 will convert pin ADCINA0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINA1
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x01;  //SOC1 will convert pin ADCINA1
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINA2
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 0x02;  //SOC2 will convert pin ADCINA2
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINA3
	AdcaRegs.ADCSOC3CTL.bit.CHSEL = 0x03;  //SOC3 will convert pin ADCINA3
	AdcaRegs.ADCSOC3CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINA4
	AdcaRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC4 will convert pin ADCINA4
	AdcaRegs.ADCSOC4CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINA5
	AdcaRegs.ADCSOC5CTL.bit.CHSEL = 0x05;  //SOC5 will convert pin ADCINA5
	AdcaRegs.ADCSOC5CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCIN14
	AdcaRegs.ADCSOC6CTL.bit.CHSEL = 0x0E;  //SOC6 will convert pin ADCIN14
	AdcaRegs.ADCSOC6CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCIN15
	AdcaRegs.ADCSOC7CTL.bit.CHSEL = 0x0F;  //SOC7 will convert pin ADCIN15
	AdcaRegs.ADCSOC7CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC7CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//
	//ADCINB
	//
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x05; //end of SOC5 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x00;  //SOC0 will convert pin ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINB1
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 0x01;  //SOC1 will convert pin ADCINB1
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINB2
	AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0x02;  //SOC2 will convert pin ADCINB2
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.CHSEL = 0x03;  //SOC3 will convert pin ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINB4
	AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC4 will convert pin ADCINB4
	AdcbRegs.ADCSOC4CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 1; // Trigger on Timer0

	//ADCINB5
	AdcbRegs.ADCSOC5CTL.bit.CHSEL = 0x05;  //SOC5 will convert pin ADCINB5
	AdcbRegs.ADCSOC5CTL.bit.ACQPS = ACQPS_TIME; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 1; // Trigger on Timer0

}

void convertADCBank(uint16_t id) {
	if (id >= 0x10) {
		AdcbRegs.ADCSOCFRC1.all = 0xFF; //Force Convert ADCINB
		while (AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0) {
		}  //Wait for ADCINT1
		AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1
	}
	// Convert A
	else {
		AdcaRegs.ADCSOCFRC1.all = 0xFF; //Force Convert ADCINA
		while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0) {
		}  //Wait for ADCINT1
		AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1
	}
}

//
// epwm7_isr - EPWM2 ISR
//
__interrupt void epwm7_isr(void) {
	EPwm7Regs.ETCLR.bit.INT = 1;

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

//
// InitEPwm7Example - Initialize EPWM7 configuration
//
void InitEPwm7Example() {
	int HSPCLKDIV;
	int CLKDIV;
	double TPWM;
	double TBCLK;
	double TBPRD;
	double EPWM7_MIN_DB;

	//
	// Setup TBCLK
	//
	EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up-down
	// EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
	EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading

	if (EPWM7_Freq < 200.0) {
		EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
		EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV4;
	} else if (EPWM7_Freq < 400.0) {
		EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
		EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;
	} else if (EPWM7_Freq < 800.0) {
		EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
		EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	} else if (EPWM7_Freq < 1550.0) {
		EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;
		EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	} else {
		EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
		EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	}
	switch (EPwm7Regs.TBCTL.bit.HSPCLKDIV) {
	case TB_DIV1:
		HSPCLKDIV = 1;
		break;
	case TB_DIV2:
		HSPCLKDIV = 2;
		break;
	case TB_DIV4:
		HSPCLKDIV = 4;
		break;
	}
	switch (EPwm7Regs.TBCTL.bit.CLKDIV) {
	case TB_DIV1:
		CLKDIV = 1;
		break;
	case TB_DIV2:
		CLKDIV = 2;
		break;
	case TB_DIV4:
		CLKDIV = 4;
		break;
	}
	TPWM = 1 / (EPWM7_Freq);
	TBCLK = 1.0 / (EPWMCLK / (HSPCLKDIV * CLKDIV));
	TBPRD = TPWM / (2.0 * TBCLK);
	EPwm7Regs.TBPRD = TBPRD; // Set timer period
	EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
	EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

	//
	// Set EMP7A,EPWM7B Low on Over-voltage/Over-current condition
	//
	EALLOW;
	EPwm7Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
	EPwm7Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
	EPwm7Regs.TZCLR.bit.OST = 1;

	// Enable Single Software Force
	EPwm7Regs.AQSFRC.bit.ACTSFA = 0x2; // Output A High
	EPwm7Regs.AQSFRC.bit.ACTSFB = 0x2; // Output B High
	EDIS;

	//
	// Set Duty Cycle
	//
	EPwm7Regs.CMPA.bit.CMPA = EPWM7_Duty * EPwm7Regs.TBPRD;

	//
	// Set actions
	//
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR;

	EPwm7Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2A on Zero
	EPwm7Regs.AQCTLB.bit.CAD = AQ_SET;


	//
	// Active High complementary PWMs - setup the deadband
	//
	EPWM7_MIN_DB = DB_TIME / TBCLK;

    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBRED.bit.DBRED = EPWM7_MIN_DB;
    EPwm7Regs.DBFED.bit.DBFED = EPWM7_MIN_DB;

    //
    // Interrupt where we will modify the deadband
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
	enablePWMBuffer();
}

//
// InitEPwm8Example - Initialize EPWM8 configuration
//
void InitEPwm8Example() {
	int HSPCLKDIV;
	int CLKDIV;
	double TPWM;
	double TBCLK;
	double TBPRD;
	double EPWM8_MIN_DB;

	//
	// Setup TBCLK
	//
	EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
	EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading

	if (EPWM8_Freq < 200.0) {
		EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
		EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV4;
	} else if (EPWM8_Freq < 400.0) {
		EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
		EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV2;
	} else if (EPWM8_Freq < 800.0) {
		EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;
		EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	} else if (EPWM8_Freq < 1550.0) {
		EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;
		EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	} else {
		EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
		EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	}
	switch (EPwm8Regs.TBCTL.bit.HSPCLKDIV) {
	case TB_DIV1:
		HSPCLKDIV = 1;
		break;
	case TB_DIV2:
		HSPCLKDIV = 2;
		break;
	case TB_DIV4:
		HSPCLKDIV = 4;
		break;
	}
	switch (EPwm8Regs.TBCTL.bit.CLKDIV) {
	case TB_DIV1:
		CLKDIV = 1;
		break;
	case TB_DIV2:
		CLKDIV = 2;
		break;
	case TB_DIV4:
		CLKDIV = 4;
		break;
	}
	TPWM = 1 / (EPWM8_Freq);
	TBCLK = 1.0 / (EPWMCLK / (HSPCLKDIV * CLKDIV));
	TBPRD = TPWM / (2.0 * TBCLK);
	EPwm8Regs.TBPRD = TBPRD; // Set timer period
	EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
	EPwm8Regs.TBCTR = 0x0000;                     // Clear counter

	//
	// Set EMP8A, EPWM8B Low on Over-voltage/Over-current condition
	//
	EALLOW;
	EPwm8Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
	EPwm8Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
	EPwm8Regs.TZCLR.bit.OST = 1;
	EDIS;

	//
	// Set Duty Cycle
	//
	EPwm8Regs.CMPA.bit.CMPA = (1.0 - EPWM8_Duty) * EPwm8Regs.TBPRD;

	//
	// Set actions
	//
	EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
	EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR;

	EPwm8Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2A on Zero
	EPwm8Regs.AQCTLB.bit.CAD = AQ_SET;

	//
	// Active High complementary PWMs - setup the deadband
	//
	EPWM8_MIN_DB = DB_TIME / TBCLK;

	EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
	EPwm8Regs.DBRED.bit.DBRED = EPWM8_MIN_DB;
	EPwm8Regs.DBFED.bit.DBFED = EPWM8_MIN_DB;

	//
	// Interrupt where we will modify the deadband
	//
	EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm8Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	EPwm8Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
	enablePWMBuffer();
}

void storeADCValues() {
//	// Digital Values
//	// J1 and J3
//	SGND51R_ADC = sampleADC(ADCINID_14); // Short to SGND through 51R
//	VC4_ADC = sampleADC(ADCINID_B1);
//	V2AB_ADC = sampleADC(ADCINID_B4);
//	VC3_ADC = sampleADC(ADCINID_B2);
//	V1AB_ADC = sampleADC(ADCINID_A0);
//	IL4_ADC = sampleADC(ADCINID_B0);
//	IL3_ADC = sampleADC(ADCINID_A1);
//
//	// J5 and J7
//	VDC2_ADC = sampleADC(ADCINID_15);
//	VDC1_ADC = sampleADC(ADCINID_A2);
//	IL2_ADC = sampleADC(ADCINID_A5);
//	SGND_ADC = sampleADC(ADCINID_B5); // Short to SGND
//	S1V5_ADC = sampleADC(ADCINID_A3); // Short to 1.5V
//	ANALOG_ADC = sampleADC(ADCINID_B3);
//	IL1_ADC = sampleADC(ADCINID_A4);

	// Optimization
	SGND51R_ADC = AdcaResultRegs.ADCRESULT6;
	VC4_ADC = AdcbResultRegs.ADCRESULT1;
	V2AB_ADC = AdcbResultRegs.ADCRESULT4;
	VC3_ADC = AdcbResultRegs.ADCRESULT2;
	V1AB_ADC = AdcaResultRegs.ADCRESULT0;
	IL4_ADC = AdcbResultRegs.ADCRESULT0;
	IL3_ADC = AdcaResultRegs.ADCRESULT1;

	// J5 and J7
	VDC2_ADC = AdcaResultRegs.ADCRESULT7;
	VDC1_ADC = AdcaResultRegs.ADCRESULT2;
	IL2_ADC = AdcaResultRegs.ADCRESULT5;
	SGND_ADC = AdcbResultRegs.ADCRESULT7; // Short to SGND
	S1V5_ADC = AdcaResultRegs.ADCRESULT3; // Short to 1.5V
	ANALOG_ADC = AdcbResultRegs.ADCRESULT3;
	IL1_ADC = AdcaResultRegs.ADCRESULT4;


	// Actual Values
	V1AB_Real = (2.0 * V1AB_ADC) - 6249.0;

	V2AB_Real = (2.0 * V2AB_ADC) - 6234.0;

	// VDC1_Real = VDC1_ADC;
	VDC1_Real = -450.709 + 2.19091 * VDC1_ADC;
	//VDC1_Real = (3.3 * VDC1_ADC/4095.0 - 0.165) * 1700.0;

	//VDC2_Real = VDC2_ADC;
	VDC2_Real = -468.707 + 2.307 * VDC2_ADC;
//	VDC2_Real = (3.3 * VDC2_ADC/4095.0 - 0.165) * (-1700.0);

	//IL1_Real = IL1_ADC;
	IL1_Real = 730.05 - 0.25 * IL1_ADC;
	//IL1_Real = (3.3 * IL1_ADC/4095.0 - 1.65);


	IL2_Real = (3.3 * (IL2_ADC-2070.0)/4095.0) * 2.0 * 36 * 17;

}

void printAndyBoard(void) {
	printf("Raw Values\n\r");
	printf("ADCINA0 Sample = %u\n\r", AdcaResultRegs.ADCRESULT0);

	printf("Real Values\n\r");
	printf("V2AB_Real = %.2f\n\r", V2AB_Real);
}

char checkV2AB() {
	return 0;
}

char checkVDC1() {
	if (VDC1_Real_Avg > VDC1_LIMIT)
		return 1;
	return 0;
}

char checkVDC2() {
	return 0;
}

char checkVDC12() {
	if (VDC12_Real_Avg > VDC12_LIMIT)
		return 1;
	return 0;
}

char checkIL1() {
	if (IL1_Real_Avg > IL1_LIMIT)
		return 1;
	return 0;
}

void disableEPWM(void) {

	GPIO_WritePin(69, 0);
	EALLOW;
	EPwm7Regs.TZFRC.bit.OST = 1;
	EPwm8Regs.TZFRC.bit.OST = 1;
	EDIS;

}

void haltOverVoltage(void) {
	char flag = 0;
//	flag |= checkVDC1();
//	flag |= checkVDC2();
//	flag |= checkV2AB();

//	flag |= checkVDC12();
	flag |= checkIL1();

	if (flag) {
		disableEPWM();
		while (1) {
		}
	}
}

void hysteresisControl() {
	if (up && (V2AB_Real_Avg >= 9.0)) {
		up = 0;
		toggleBlueLED();
	} else if ((!up) && (V2AB_Real_Avg <= -9.0)) {
		up = 1;
		toggleBlueLED();
	}
}

void toggleBlueLED() {
	blueLED_state = !blueLED_state;
	GPIO_WritePin(LED_GPIO_BLUE, blueLED_state);
}

void toggleRedLED() {
	redLED_state = !redLED_state;
	GPIO_WritePin(LED_GPIO_RED, redLED_state);
}

void toggleGPIOPWM1() {
	PWM1_state = !PWM1_state;
	GPIO_WritePin(PWM1_GPIO, PWM1_state);
}

void toggleGPIOPWM2() {
	PWM2_state = !PWM2_state;
	GPIO_WritePin(PWM2_GPIO, PWM2_state);
}

void toggleGPIOPWM3() {
	PWM3_state = !PWM3_state;
	GPIO_WritePin(PWM3_GPIO, PWM3_state);
}

void toggleGPIOPWM4() {
	PWM4_state = !PWM4_state;
	GPIO_WritePin(PWM4_GPIO, PWM4_state);
}

void PWM7HysteresisControl() {
	// Initially discharging
	if (PWM7_seekUpperBand == 2) {
		EPwm7Regs.CMPA.bit.CMPA = EPwm7Regs.TBPRD;
		PWM7_seekUpperBand = 0;
	}

	// Seeking upper band
	if (PWM7_seekUpperBand == 1) {

		if (IL1_Real_Avg > IL1_Hyst_SetPoint + HYSTERESIS_BAND) {
			// Switch to Discharge


			PWM7_seekUpperBand = 0;
		}
		else
			EPwm7Regs.AQSFRC.bit.OTSFB = 0x1; // Output B High to charge inductor

	}
	// Seeking Lower Band
	else if (PWM7_seekUpperBand == 0) {

		if (IL1_Real_Avg < IL1_Hyst_SetPoint - HYSTERESIS_BAND) {
			// Switch to Charging
			PWM7_seekUpperBand = 1;
		}
		else
			EPwm7Regs.AQSFRC.bit.OTSFA = 0x1; // Output A High to discharge inductor
	}

}

__interrupt void cpu_timer0_isr(void) {
	//printf("timer0\n\r");

	storeADCValues(); // Store and Convert ADC Values in variables in memory
	movingAvgADC();
	haltOverVoltage();
//	toggleGPIOPWM1();
//	toggleGPIOPWM2();
//	toggleGPIOPWM3();
//	toggleGPIOPWM4();



	// hysteresisControl();

//    //
//    // Start Cpu Timer1 to indicate begin of interrupt
//    //
//    CpuTimer1Regs.TCR.all = 0x0000;

	//
	// Write current sine value to buffered DAC
	//
	DAC_PTR[DAC_NUM]->DACVALS.all = IL1_Ref;

	//
	// Log current sine value
	//
//	dlog(sgenIL1a_out);

	//
	// Compute next sine value
	//
	sgenIL1a.calc(&sgenIL1a);
	sgenIL1b.calc(&sgenIL1b);
	// triwave1.calc(&triwave1);

	//
	// Scale next sine value
	//
	//IL1_Ref = (sgenIL1a.out + 32768) >> 4; // A
	//IL1_Ref = (sgenIL1b.out + 32768) >> 4; // B
	IL1_Ref = ((sgenIL1a.out + sgenIL1b.out + 32768) >> 4); // A+B
	// IL1_Ref = ((triwave1.out + 32768) >> 4); // Ramp

	// IL1_Ref = (IL1_Ref/4095.0) * 0.3 + 0.5;
	// IL1_Hyst_SetPoint = (IL1_Ref/4095.0) * 1.5;

	// PWM7CurrentControl();
	PWM7HysteresisControl();

	//
	// Acknowledge this interrupt to receive more interrupts from group 1
	//
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

//    //
//    // Stop Cpu Timer1 to indicate end of interrupt
//    //
//    CpuTimer1Regs.TCR.all = 0x0010;
//
//    //
//    // Calculate interrupt duration in cycles
//    //
//    interruptCycles = 0xFFFFFFFFUL - CpuTimer1Regs.TIM.all;
//
//    //
//    // Calculate interrupt duration in micro seconds
//    //
//    interruptDuration_us = cpuPeriod_us * interruptCycles;
//
//    //
//    // Reload Cpu Timer1
//    //
//    CpuTimer1Regs.TCR.all = 0x0030;
}

__interrupt void cpu_timer1_isr(void) {
	//printf("timer1\n\r");

}

__interrupt void cpu_timer2_isr(void) {
	//printf("timer2\n\r");

}

void movingAvgADC() {
	if (buf_idx >= MOV_AVG_SIZE)
		buf_idx = 0;
	if (buf_idx_hi >= MOV_AVG_SIZE_HI)
		buf_idx_hi = 0;

	// VDC1
	VDC1_Real_BufSum -= VDC1_Real_Buf[buf_idx];
	VDC1_Real_Buf[buf_idx] = VDC1_Real;
	VDC1_Real_BufSum += VDC1_Real_Buf[buf_idx];
	VDC1_Real_Avg = VDC1_Real_BufSum / MOV_AVG_SIZE;

	// VDC2
	VDC2_Real_BufSum -= VDC2_Real_Buf[buf_idx];
	VDC2_Real_Buf[buf_idx] = VDC2_Real;
	VDC2_Real_BufSum += VDC2_Real_Buf[buf_idx];
	VDC2_Real_Avg = VDC2_Real_BufSum / MOV_AVG_SIZE;

	// DC Link
	VDC12_Real_Avg = VDC1_Real_Avg - VDC2_Real_Avg;

	// V1AB
	V1AB_Real_BufSum -= V1AB_Real_Buf[buf_idx];
	V1AB_Real_Buf[buf_idx] = V1AB_Real;
	V1AB_Real_BufSum += V1AB_Real_Buf[buf_idx];
	V1AB_Real_Avg = V1AB_Real_BufSum / MOV_AVG_SIZE;

	// V2AB
	V2AB_Real_BufSum -= V2AB_Real_Buf[buf_idx];
	V2AB_Real_Buf[buf_idx] = V2AB_Real;
	V2AB_Real_BufSum += V2AB_Real_Buf[buf_idx];
	V2AB_Real_Avg = V2AB_Real_BufSum / MOV_AVG_SIZE;

	// IL1
	IL1_Real_BufSum -= IL1_Real_Buf[buf_idx_hi];
	IL1_Real_Buf[buf_idx_hi] = IL1_Real;
	IL1_Real_BufSum += IL1_Real_Buf[buf_idx_hi];
	IL1_Real_Avg = IL1_Real_BufSum / MOV_AVG_SIZE_HI;

	// IL2
	IL2_Real_BufSum -= IL2_Real_Buf[buf_idx_hi];
	IL2_Real_Buf[buf_idx_hi] = IL2_Real;
	IL2_Real_BufSum += IL2_Real_Buf[buf_idx_hi];
	IL2_Real_Avg = IL2_Real_BufSum / MOV_AVG_SIZE_HI;


	buf_idx++;
	buf_idx_hi++;

	// printf("%f\n\r",IL1_Real_Avg);
}

void enablePWMBuffer() {
	GPIO_SetupPinMux(69, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(69, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(69, 1);

	GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(66, 0);
}

// DAC Test Stuff
//
// dlog - Circular DataLog. DataLog[0] contains the next index to
//        be overwritten
//
static inline void dlog(Uint16 value) {
	DataLog[ndx] = value;
	if (++ndx == DLOG_SIZE) {
		ndx = 0;
	}
	DataLog[0] = ndx;
}

//
// setFreq - Set the SINE frequency in sgenIL1a
//
static inline void setFreq(void) {
#if SINEWAVE_TYPE==LOW_THD_SINE
	//
	// Range(Q0) = 0x0000 -> 0x7FFF, step_max(Q0) =
	// (Max_Freq_hz*0x10000)/Sampling_Freq_hz
	//
	sgenIL1a.step_max = (maxOutputFreq_hz * 0x10000) / samplingFreq_hz;
	sgenIL1b.step_max = (maxOutputFreq_hz * 0x10000) / samplingFreq_hz;

	//
	// Range(Q15) = 0x0000 -> 0x7FFF, freq(Q15) =
	// (Required_Freq_hz/Max_Freq_hz)*0x8000
	//
	sgenIL1a.freq = ((float) outputFreqA_hz / maxOutputFreq_hz) * 0x8000;
	sgenIL1b.freq = ((float) outputFreqB_hz / maxOutputFreq_hz) * 0x8000;
#elif SINEWAVE_TYPE==HIGH_PRECISION_SINE
	//
	// Range(Q0) = 0x00000000 -> 0x7FFFFFFF, step_max(Q0) =
	// (Max_Freq_hz*0x100000000)/Sampling_Freq_hz
	//
	sgenIL1a.step_max = (maxOutputFreq_hz*0x100000000)/samplingFreq_hz;
	sgenIL1b.step_max = (maxOutputFreq_hz*0x100000000)/samplingFreq_hz;

	//
	// Range(Q31) = 0x00000000 -> 0x7FFFFFFF, freq(Q31) =
	// (Required_Freq_hz/Max_Freq_hz)*0x80000000
	//
	sgenIL1a.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x80000000;
	sgenIL1b.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x80000000;
#endif

	freqResolution_hz = (float) maxOutputFreq_hz / sgenIL1a.step_max;
}

//
// setGain - Set the gain in sgenIL1a
//
static inline void setGain(void) {
	sgenIL1a.gain = waveformGainA * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
	sgenIL1b.gain = waveformGainB * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
}

//
// setOffset - Set the offset in sgenIL1a
//
static inline void setOffset(void) {
	sgenIL1a.offset = waveformOffsetA * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
	sgenIL1b.offset = waveformOffsetB * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
}

//
// getMax - Get the max value in the data log
//
static inline Uint16 getMax(void) {
	Uint16 index = 0;
	Uint16 tempMax = 0;

	for (index = 1; index < DLOG_SIZE; index++) {
		if (tempMax < DataLog[index]) {
			tempMax = DataLog[index];
		}
	}

	return tempMax;
}

//
// getMin - Get the min value in the data log
//
static inline Uint16 getMin(void) {
	Uint16 index = 0;
	Uint16 tempMin = 0xFFFF;

	for (index = 1; index < DLOG_SIZE; index++) {
		if (tempMin > DataLog[index]) {
			tempMin = DataLog[index];
		}
	}
	return tempMin;
}

//
// configureDAC - Enable and configure the requested DAC module
//
void configureDAC(Uint16 dac_num) {
	EALLOW;

	DAC_PTR[dac_num]->DACCTL.bit.DACREFSEL = REFERENCE;
	DAC_PTR[dac_num]->DACOUTEN.bit.DACOUTEN = 1;
	DAC_PTR[dac_num]->DACVALS.all = 0;

	DELAY_US(10); // Delay for buffered DAC to power up

	EDIS;
}

//
// configureWaveform - Configure the SINE waveform
//
void configureWaveform(void) {
	sgenIL1a.alpha = 0; // Range(16) = 0x0000 -> 0xFFFF
	sgenIL1b.alpha = 0;
	setFreq();
	setGain();
	setOffset();
}

void PWM7CurrentControl() {
	float EPWM7_NewDuty = 0.5;

	EPWM7_NewDuty = EPWM7B_MIN_DUTY + (IL1_Ref/4095.0)*(EPWM7B_MAX_DUTY - EPWM7B_MIN_DUTY);
	if (EPWM7_NewDuty > EPWM7B_MAX_DUTY)
		EPWM7_NewDuty = EPWM7B_MAX_DUTY;
	if (EPWM7_NewDuty < EPWM7B_MIN_DUTY)
		EPWM7_NewDuty = EPWM7B_MIN_DUTY;
	EPwm7Regs.CMPA.bit.CMPA = EPWM7_NewDuty * EPwm7Regs.TBPRD;
}

//
// End of file
//

