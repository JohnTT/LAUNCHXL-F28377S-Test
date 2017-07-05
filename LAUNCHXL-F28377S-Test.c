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

// General
#define LED_GPIO_RED    12
#define LED_GPIO_BLUE	13

#define PWM1_GPIO	12
#define PWM2_GPIO	13
#define PWM3_GPIO	14
#define PWM4_GPIO	15

#define VOLTAGE_LIMIT 300.0

// Analog to Digital (ADC)
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

#define MOV_AVG_SIZE 8

// EPWM
#define EPWM1_MAX_DB   0x03FF
#define EPWM2_MAX_DB   0x03FF
#define EPWM3_MAX_DB   0x03FF
#define EPWM1_MIN_DB   0
#define EPWM2_MIN_DB   0
#define EPWM3_MIN_DB   0
#define DB_UP          1
#define DB_DOWN        0


//
// Globals
//

// General
// FILE -> printf variables
volatile int status = 0;
volatile FILE *fid;

// LED states
char blueLED_state = 0;
char redLED_state = 0;

char PWM1_state = 0;
char PWM2_state = 0;
char PWM3_state = 0;
char PWM4_state = 0;

float CMPHI = 8.0;
float CMPLO = -8.0;
char up = 1;

// Analog to Digital (ADC)

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
float VC4_Real = 0;

float V2AB_Real = 0;
float V2AB_Real_Buf[MOV_AVG_SIZE];
float V2AB_Real_BufSum = 0;
float V2AB_Real_Avg = 0;

float VC3_Real = 0;
float V1AB_Real = 0;
float IL4_Real = 0;
float IL3_Real = 0;

// J5 and J7
float VDC2_Real = 0;
float VDC2_Real_Buf[MOV_AVG_SIZE];
float VDC2_Real_BufSum = 0;
float VDC2_Real_Avg = 0;

float VDC1_Real = 0;
float VDC1_Real_Buf[MOV_AVG_SIZE];
float VDC1_Real_BufSum = 0;
float VDC1_Real_Avg = 0;

float IL2_Real = 0;
float ANALOG_Real = 0;
float IL1_Real = 0;

// EPWM
Uint32 EPwm1TimerIntCount;
Uint32 EPwm2TimerIntCount;
Uint32 EPwm3TimerIntCount;
Uint16 EPwm1_DB_Direction;
Uint16 EPwm2_DB_Direction;
Uint16 EPwm3_DB_Direction;

//
// Function Prototypes
//

// General
extern void DSP28x_usDelay(Uint32 Count);
void configureLEDs(void);
void scia_init(void);
void configureSCIprintf(void);
void printAndyBoard(void);

void toggleBlueLED(void);
void toggleRedLED(void);

void togglePWM1(void);
void togglePWM2(void);
void togglePWM3(void);
void togglePWM4(void);

// Analog to Digital (ADC)
void configureADC(void);
uint16_t sampleADC(uint16_t id);
void convertADCBank(uint16_t id);
void storeADCValues(void);
void movingAvgADC(void);

void haltOverVoltage(void);
void hysteresisControl(void);

// EPWM
void configureGateDriver(void);
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);


// CPU Timers
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);


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
	// Step 2. Initialize GPIO:
	// This example function is found in the F2837xS_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.
	//
	InitGpio();

	//
	// Step 3. Clear all interrupts and initialize PIE vector table:
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
	// enable PWM1, PWM2 and PWM3
	//
//	CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
//	CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
//	CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;

	//
	// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
	// These functions are in the F2837xS_EPwm.c file
	//
//	InitEPwm1Gpio();
//	InitEPwm2Gpio();
//	InitEPwm3Gpio();

	//
	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	//
	EALLOW;
	// This is needed to write to EALLOW protected registers
//	PieVectTable.EPWM1_INT = &epwm1_isr;
//	PieVectTable.EPWM2_INT = &epwm2_isr;
//	PieVectTable.EPWM3_INT = &epwm3_isr;

	PieVectTable.TIMER0_INT = &cpu_timer0_isr; // Need this for CPU Timer0
	PieVectTable.TIMER1_INT = &cpu_timer1_isr; // Need this for CPU Timer1
	PieVectTable.TIMER2_INT = &cpu_timer2_isr; // Need this for CPU Timer2

	EDIS;
	// This is needed to disable write to EALLOW protected registers

	//
	// Step 4. Initialize the Device Peripherals:
	//
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

//	InitEPwm1Example();
//	InitEPwm2Example();
//	InitEPwm3Example();

	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	InitCpuTimers();   // For this example, only initialize the Cpu Timers


	// Configure CPU-Timer 0,1,2 to __interrupt
	ConfigCpuTimer(&CpuTimer0, 200, 10); // lower than 10us too fast, prevents other ISR from running
	ConfigCpuTimer(&CpuTimer1, 200, 500000);
	ConfigCpuTimer(&CpuTimer2, 200, 1000000);

	//
	// To ensure precise timing, use write-only instructions to write to the entire
	// register. Therefore, if any of the configuration bits are changed in
	// ConfigCpuTimer and InitCpuTimers (in F2837xS_cputimervars.h), the below
	// settings must also be updated.
	//
	CpuTimer0Regs.TCR.all = 0x4001;
	CpuTimer1Regs.TCR.all = 0x4001;
	CpuTimer2Regs.TCR.all = 0x4001;

	//
	// Step 5. User specific code, enable interrupts:
	// Initialize counters:
	//
//	EPwm1TimerIntCount = 0;
//	EPwm2TimerIntCount = 0;
//	EPwm3TimerIntCount = 0;

	//
	// Enable CPU INT1, INT13, INT14 which is connected to CPU-Timer 0, 1, 2:
	//
	IER |= M_INT1; // Enable CPU Timer 0 Interrupts
	IER |= M_INT13; // Enable CPU Timer 1 Interrupts
	IER |= M_INT14; // Enable CPU Timer 2 Interrupts

	//
	// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
	//
//	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
//	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
//	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;


	// Enable Timer0 Interrupt, TINT0 in the PIE: Group 1 __interrupt 7
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;


	// User Configuration Functions
	//configureLEDs();
	//configureGateDrivers();
	configureADC();
	configureSCIprintf();

	EINT;// Enable Global interrupt INTM
	ERTM;// Enable Global realtime interrupt DBGM

// take conversions indefinitely in loop
	do {

//		toggleBlueLED();

		// DELAY_US(100*1);
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

void configureLEDs() {
	// Pin 12 - Blue LED
	GPIO_SetupPinMux(LED_GPIO_BLUE, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_BLUE, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(LED_GPIO_BLUE, 1);

	// Pin 13 - Red LED
	GPIO_SetupPinMux(LED_GPIO_RED, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_RED, GPIO_OUTPUT, GPIO_PUSHPULL);
	GPIO_WritePin(LED_GPIO_RED, 1);

	// Ken Drives
	// PWM1+
	GPIO_SetupPinMux(LED_GPIO_RED, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_RED, GPIO_OUTPUT, GPIO_PUSHPULL);
}

void configureGateDrivers() {
	// Ken Drives
	// PWM1+
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

void configureSCIprintf() {
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

void configureADC() {
	// Initialize Moving Average Arrays
	int idx;
	for (idx = 0; idx < MOV_AVG_SIZE; idx++) {
		V2AB_Real_Buf[idx] = 0;
		VDC1_Real_Buf[idx] = 0;
		VDC2_Real_Buf[idx] = 0;
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
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;

	//ADCINA1
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x01;  //SOC1 will convert pin ADCINA1
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 1;

	//ADCINA2
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 0x02;  //SOC2 will convert pin ADCINA2
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 1;

	//ADCINA3
	AdcaRegs.ADCSOC3CTL.bit.CHSEL = 0x03;  //SOC3 will convert pin ADCINA3
	AdcaRegs.ADCSOC3CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 1;

	//ADCINA4
	AdcaRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC4 will convert pin ADCINA4
	AdcaRegs.ADCSOC4CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 1;

	//ADCINA5
	AdcaRegs.ADCSOC5CTL.bit.CHSEL = 0x05;  //SOC5 will convert pin ADCINA5
	AdcaRegs.ADCSOC5CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 1;

	//ADCIN14
	AdcaRegs.ADCSOC14CTL.bit.CHSEL = 0x06;  //SOC6 will convert pin ADCIN14
	AdcaRegs.ADCSOC14CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC14CTL.bit.TRIGSEL = 1;

	//ADCIN15
	AdcaRegs.ADCSOC15CTL.bit.CHSEL = 0x07;  //SOC7 will convert pin ADCIN15
	AdcaRegs.ADCSOC15CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCSOC15CTL.bit.TRIGSEL = 1;

	//
	//ADCINB
	//
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x05; //end of SOC5 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x00;  //SOC0 will convert pin ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 1;

	//ADCINB1
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 0x01;  //SOC1 will convert pin ADCINB1
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 1;

	//ADCINB2
	AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0x02;  //SOC2 will convert pin ADCINB2
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 1;

	//ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.CHSEL = 0x03;  //SOC3 will convert pin ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 1;

	//ADCINB4
	AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC4 will convert pin ADCINB4
	AdcbRegs.ADCSOC4CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 1;

	//ADCINB5
	AdcbRegs.ADCSOC5CTL.bit.CHSEL = 0x05;  //SOC5 will convert pin ADCINB5
	AdcbRegs.ADCSOC5CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 1;

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

uint16_t sampleADC(uint16_t id) {
	uint16_t temp;

	switch (id) {
	case ADCINID_A0:
		temp = AdcaResultRegs.ADCRESULT0;
		break;
	case ADCINID_A1:
		temp = AdcaResultRegs.ADCRESULT1;
		break;
	case ADCINID_A2:
		temp = AdcaResultRegs.ADCRESULT2;
		break;
	case ADCINID_A3:
		temp = AdcaResultRegs.ADCRESULT3;
		break;
	case ADCINID_A4:
		temp = AdcaResultRegs.ADCRESULT4;
		break;
	case ADCINID_A5:
		temp = AdcaResultRegs.ADCRESULT5;
		break;
	case ADCINID_14:
		temp = AdcaResultRegs.ADCRESULT6;
		break;
	case ADCINID_15:
		temp = AdcaResultRegs.ADCRESULT7;
		break;

	case ADCINID_B0:
		temp = AdcbResultRegs.ADCRESULT0;
		break;
	case ADCINID_B1:
		temp = AdcbResultRegs.ADCRESULT1;
		break;
	case ADCINID_B2:
		temp = AdcbResultRegs.ADCRESULT2;
		break;
	case ADCINID_B3:
		temp = AdcbResultRegs.ADCRESULT3;
		break;
	case ADCINID_B4:
		temp = AdcbResultRegs.ADCRESULT4;
		break;
	case ADCINID_B5:
		temp = AdcbResultRegs.ADCRESULT5;
		break;
	default:
		break;
	}

	return (temp);
}


//
// epwm1_isr - EPWM1 ISR
//
__interrupt void epwm1_isr(void)
{
    if(EPwm1_DB_Direction == DB_UP)
    {
        if(EPwm1Regs.DBFED.bit.DBFED < EPWM1_MAX_DB)
        {
            EPwm1Regs.DBFED.bit.DBFED++;
            EPwm1Regs.DBRED.bit.DBRED++;
        }
        else
        {
            EPwm1_DB_Direction = DB_DOWN;
            EPwm1Regs.DBFED.bit.DBFED--;
            EPwm1Regs.DBRED.bit.DBRED--;
        }
    }
    else
    {
        if(EPwm1Regs.DBFED.bit.DBFED == EPWM1_MIN_DB)
        {
            EPwm1_DB_Direction = DB_UP;
            EPwm1Regs.DBFED.bit.DBFED++;
            EPwm1Regs.DBRED.bit.DBRED++;
        }
        else
        {
            EPwm1Regs.DBFED.bit.DBFED--;
            EPwm1Regs.DBRED.bit.DBRED--;
        }
    }
    EPwm1TimerIntCount++;

    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// epwm2_isr - EPWM2 ISR
//
__interrupt void epwm2_isr(void)
{
    if(EPwm2_DB_Direction == DB_UP)
    {
        if(EPwm2Regs.DBFED.bit.DBFED < EPWM2_MAX_DB)
        {
            EPwm2Regs.DBFED.bit.DBFED++;
            EPwm2Regs.DBRED.bit.DBRED++;
        }
        else
        {
            EPwm2_DB_Direction = DB_DOWN;
            EPwm2Regs.DBFED.bit.DBFED--;
            EPwm2Regs.DBRED.bit.DBRED--;
        }
    }
    else
    {
        if(EPwm2Regs.DBFED.bit.DBFED == EPWM2_MIN_DB)
        {
            EPwm2_DB_Direction = DB_UP;
            EPwm2Regs.DBFED.bit.DBFED++;
            EPwm2Regs.DBRED.bit.DBRED++;
        }
        else
        {
            EPwm2Regs.DBFED.bit.DBFED--;
            EPwm2Regs.DBRED.bit.DBRED--;
        }
    }

    EPwm2TimerIntCount++;

    //
    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// epwm3_isr - EPWM3 ISR
//
__interrupt void epwm3_isr(void)
{
    if(EPwm3_DB_Direction == DB_UP)
    {
        if(EPwm3Regs.DBFED.bit.DBFED < EPWM3_MAX_DB)
        {
            EPwm3Regs.DBFED.bit.DBFED++;
            EPwm3Regs.DBRED.bit.DBRED++;
        }
        else
        {
            EPwm3_DB_Direction = DB_DOWN;
            EPwm3Regs.DBFED.bit.DBFED--;
            EPwm3Regs.DBRED.bit.DBRED--;
        }
    }
    else
    {
        if(EPwm3Regs.DBFED.bit.DBFED == EPWM3_MIN_DB)
        {
            EPwm3_DB_Direction = DB_UP;
            EPwm3Regs.DBFED.bit.DBFED++;
            EPwm3Regs.DBRED.bit.DBRED++;
        }
        else
        {
            EPwm3Regs.DBFED.bit.DBFED--;
            EPwm3Regs.DBRED.bit.DBRED--;
        }
    }

    EPwm3TimerIntCount++;

    //
    // Clear INT flag for this timer
    //
    EPwm3Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// InitEPwm1Example - Initialize EPWM1 configuration
//
void InitEPwm1Example()
{
    EPwm1Regs.TBPRD = 6000;                       // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV4;

    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Setup compare
    //
    EPwm1Regs.CMPA.bit.CMPA = 3000;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM1A on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low PWMs - Setup Deadband
    //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LO;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED.bit.DBRED = EPWM1_MIN_DB;
    EPwm1Regs.DBFED.bit.DBFED = EPWM1_MIN_DB;
    EPwm1_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the Deadband
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;               // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;          // Generate INT on 3rd event
}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//
void InitEPwm2Example()
{
    EPwm2Regs.TBPRD = 6000;                       // Set timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow just to observe on
                                                   // the scope

    //
    // Setup compare
    //
    EPwm2Regs.CMPA.bit.CMPA = 3000;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2A on Zero
    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED.bit.DBRED = EPWM2_MIN_DB;
    EPwm2Regs.DBFED.bit.DBFED = EPWM2_MIN_DB;
    EPwm2_DB_Direction = DB_UP;

    //
    // Interrupt where we will modify the deadband
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
}

//
// InitEPwm3Example - Initialize EPWM3 configuration
//
void InitEPwm3Example()
{
    EPwm3Regs.TBPRD = 6000;                        // Set timer period
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;            // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                      // Clear counter

    //
    // Setup TBCLK
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV4;          // Slow so we can observe on
                                                   // the scope

    //
    // Setup compare
    //
    EPwm3Regs.CMPA.bit.CMPA = 3000;

    //
    // Set actions
    //
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM3A on Zero
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

    EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR;           // Set PWM3A on Zero
    EPwm3Regs.AQCTLB.bit.CAD = AQ_SET;

    //
    // Active high complementary PWMs - Setup the deadband
    //
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED.bit.DBRED = EPWM3_MIN_DB;
    EPwm3Regs.DBFED.bit.DBFED = EPWM3_MIN_DB;
    EPwm3_DB_Direction = DB_UP;

    //
    // Interrupt where we will change the deadband
    //
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event
}

void storeADCValues() {
	// Digital Values
	// J1 and J3
	SGND51R_ADC = sampleADC(ADCINID_14); // Short to SGND through 51R
	VC4_ADC = sampleADC(ADCINID_B1);
	V2AB_ADC = sampleADC(ADCINID_B4);
	VC3_ADC = sampleADC(ADCINID_B2);
	V1AB_ADC = sampleADC(ADCINID_A0);
	IL4_ADC = sampleADC(ADCINID_B0);
	IL3_ADC = sampleADC(ADCINID_A1);

	// J5 and J7
	VDC2_ADC = sampleADC(ADCINID_15);
	VDC1_ADC = sampleADC(ADCINID_A2);
	IL2_ADC = sampleADC(ADCINID_A5);
	SGND_ADC = sampleADC(ADCINID_B5); // Short to SGND
	S1V5_ADC = sampleADC(ADCINID_A3); // Short to 1.5V
	ANALOG_ADC = sampleADC(ADCINID_B3);
	IL1_ADC = sampleADC(ADCINID_A4);

	// Actual Values
	V2AB_Real = (2.0 * V2AB_ADC) - 6234.0;
	VDC1_Real = 2.0*VDC1_ADC-426.0;
	VDC2_Real = VDC2_ADC - 218.0;
}

void printAndyBoard(void) {
	printf("Raw Values\n\r");
	printf("ADCINA0 Sample = %u\n\r", sampleADC(ADCINID_A0));
	printf("ADCINA1 Sample = %u\n\r", sampleADC(ADCINID_A1));
	printf("ADCINA2 Sample = %u\n\r", sampleADC(ADCINID_A2));
	printf("ADCINA3 Sample = %u\n\r", sampleADC(ADCINID_A3));
	printf("ADCINA4 Sample = %u\n\r", sampleADC(ADCINID_A4));
	printf("ADCINA5 Sample = %u\n\r", sampleADC(ADCINID_A5));
	printf("ADCIN14 Sample = %u\n\r", sampleADC(ADCINID_14));
	printf("ADCIN15 Sample = %u\n\r", sampleADC(ADCINID_15));

	printf("ADCINB0 Sample = %u\n\r", sampleADC(ADCINID_B0));
	printf("ADCINB1 Sample = %u\n\r", sampleADC(ADCINID_B1));
	printf("ADCINB2 Sample = %u\n\r", sampleADC(ADCINID_B2));
	printf("ADCINB3 Sample = %u\n\r", sampleADC(ADCINID_B3));
	printf("ADCINB4 Sample = %u\n\r", sampleADC(ADCINID_B4));
	printf("ADCINB5 Sample = %u\n\r", sampleADC(ADCINID_B5));

	printf("Real Values\n\r");
	printf("V2AB_Real = %.2f\n\r", V2AB_Real);
}

void haltOverVoltage(void) {
	if (V2AB_Real_Avg > VOLTAGE_LIMIT || V2AB_Real_Avg < (-1.0*VOLTAGE_LIMIT)) {
		while (1) {
			toggleBlueLED();
			DELAY_US(1000*1000);
			toggleRedLED();
			DELAY_US(1000*1000);
		}
	}

}

void hysteresisControl() {
	if (up && (V2AB_Real_Avg >= 9.0)) {
		up = 0;
		toggleBlueLED();
	}
	else if ((!up) && (V2AB_Real_Avg <= -9.0)) {
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

void togglePWM1() {
	PWM1_state = !PWM1_state;
	GPIO_WritePin(PWM1_GPIO, PWM1_state);
}

void togglePWM2() {
	PWM2_state = !PWM2_state;
	GPIO_WritePin(PWM2_GPIO, PWM2_state);
}

void togglePWM3() {
	PWM3_state = !PWM3_state;
	GPIO_WritePin(PWM3_GPIO, PWM3_state);
}

void togglePWM4() {
	PWM4_state = !PWM4_state;
	GPIO_WritePin(PWM4_GPIO, PWM4_state);
}

__interrupt void cpu_timer0_isr(void) {
	//printf("timer0\n\r");

	storeADCValues(); // Store and Convert ADC Values in variables in memory
	movingAvgADC();
	// haltOverVoltage();

	hysteresisControl();


	// Acknowledge this __interrupt to receive more __interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void cpu_timer1_isr(void) {
	//printf("timer1\n\r");

	togglePWM1();
	togglePWM3();
}

__interrupt void cpu_timer2_isr(void) {
	//printf("timer2\n\r");

	togglePWM2();
	togglePWM4();
}

void movingAvgADC() {
	if (buf_idx >= MOV_AVG_SIZE)
		buf_idx = 0;

	// V2AB
	V2AB_Real_BufSum -= V2AB_Real_Buf[buf_idx];
	V2AB_Real_Buf[buf_idx] = V2AB_Real;
	V2AB_Real_BufSum += V2AB_Real_Buf[buf_idx];
	V2AB_Real_Avg = V2AB_Real_BufSum / MOV_AVG_SIZE;

	// VDC1
	VDC1_Real_BufSum -= VDC1_Real_Buf[buf_idx];
	VDC1_Real_Buf[buf_idx] = VDC1_Real;
	VDC1_Real_BufSum += VDC1_Real_Buf[buf_idx];
	VDC1_Real_Avg = VDC1_Real_BufSum / MOV_AVG_SIZE;

	// VDC2
	VDC2_Real_BufSum -= VDC2_Real_Buf[buf_idx];
	VDC2_Real_Buf[buf_idx] = VDC1_Real;
	VDC2_Real_BufSum += VDC2_Real_Buf[buf_idx];
	VDC2_Real_Avg = VDC2_Real_BufSum / MOV_AVG_SIZE;

	buf_idx++;
}

//
// End of file
//

