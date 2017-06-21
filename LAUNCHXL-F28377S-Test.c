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
#include "ti_ascii.h"
#include "sci_io.h"

//
// Function Prototypes
//
extern void DSP28x_usDelay(Uint32 Count);

void configureLEDs(void);
void configureADC(void);
void configureSCIprintf(void);

void scia_init(void);
uint16_t sampleADC(uint16_t id);

//
// Defines
//
#define RESULTS_BUFFER_SIZE 256 //buffer for storing conversion results (size must be multiple of 16)

#define LED_GPIO_RED    12
#define LED_GPIO_BLUE	13

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

//
// Globals
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;

// FILE -> printf variables
volatile int status = 0;
volatile FILE *fid;

// LED states
int blueLED_state = 0;
int redLED_state = 0;



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
	InitGpio(); // Skipped for this example

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
// Disable CPU interrupts and clear all CPU interrupt flags:
//
	IER = 0x0000;
	IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xS_DefaultIsr.c.
// This function is found in F2837xS_PieVect.c.
//
	InitPieVectTable();

	// Enable global Interrupts and higher priority real-time debug events:
	EINT; // Enable Global interrupt INTM
	ERTM; // Enable Global realtime interrupt DBGM

	configureLEDs();
	configureADC();
	configureSCIprintf();


// take conversions indefinitely in loop
	do {
		printf("main while\n\r");
		printf("ADCINA0 Sample = %d\n\r", sampleADC(ADCINID_A0));
		printf("ADCINA1 Sample = %d\n\r", sampleADC(ADCINID_A1));
		printf("ADCINA2 Sample = %d\n\r", sampleADC(ADCINID_A2));
		printf("ADCINA3 Sample = %d\n\r", sampleADC(ADCINID_A3));
		printf("ADCINA4 Sample = %d\n\r", sampleADC(ADCINID_A4));
		printf("ADCINA5 Sample = %d\n\r", sampleADC(ADCINID_A5));
		printf("ADCIN14 Sample = %d\n\r", sampleADC(ADCINID_14));
		printf("ADCIN15 Sample = %d\n\r", sampleADC(ADCINID_15));

		printf("ADCINB0 Sample = %d\n\r", sampleADC(ADCINID_B0));
		printf("ADCINB1 Sample = %d\n\r", sampleADC(ADCINID_B1));
		printf("ADCINB2 Sample = %d\n\r", sampleADC(ADCINID_B2));
		printf("ADCINB3 Sample = %d\n\r", sampleADC(ADCINID_B3));
		printf("ADCINB4 Sample = %d\n\r", sampleADC(ADCINID_B4));
		printf("ADCINB5 Sample = %d\n\r", sampleADC(ADCINID_B5));

		GPIO_WritePin(LED_GPIO_RED, redLED_state);
		redLED_state = !redLED_state;

		DELAY_US(1000*1000);
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
	GPIO_SetupPinMux(LED_GPIO_RED, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_RED, GPIO_OUTPUT, GPIO_PUSHPULL);

	GPIO_SetupPinMux(LED_GPIO_BLUE, GPIO_MUX_CPU1, 0);
	GPIO_SetupPinOptions(LED_GPIO_BLUE, GPIO_OUTPUT, GPIO_PUSHPULL);

	GPIO_WritePin(LED_GPIO_RED, 1);
	GPIO_WritePin(LED_GPIO_BLUE, 1);

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
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x07; //end of SOC5 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//ADCINA0
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x00;  //SOC0 will convert pin ADCINA0
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINA1
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x01;  //SOC1 will convert pin ADCINA1
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINA2
	AdcaRegs.ADCSOC2CTL.bit.CHSEL = 0x02;  //SOC2 will convert pin ADCINA2
	AdcaRegs.ADCSOC2CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINA3
	AdcaRegs.ADCSOC3CTL.bit.CHSEL = 0x03;  //SOC3 will convert pin ADCINA3
	AdcaRegs.ADCSOC3CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINA4
	AdcaRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC4 will convert pin ADCINA4
	AdcaRegs.ADCSOC4CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINA5
	AdcaRegs.ADCSOC5CTL.bit.CHSEL = 0x05;  //SOC5 will convert pin ADCINA5
	AdcaRegs.ADCSOC5CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCIN14
	AdcaRegs.ADCSOC14CTL.bit.CHSEL = 0x06;  //SOC6 will convert pin ADCIN14
	AdcaRegs.ADCSOC14CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCIN15
	AdcaRegs.ADCSOC15CTL.bit.CHSEL = 0x07;  //SOC7 will convert pin ADCIN15
	AdcaRegs.ADCSOC15CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//
	//ADCINB
	//
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x05; //end of SOC5 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x00;  //SOC0 will convert pin ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINB1
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 0x01;  //SOC1 will convert pin ADCINB1
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINB2
	AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0x02;  //SOC2 will convert pin ADCINB2
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.CHSEL = 0x03;  //SOC3 will convert pin ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINB4
	AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC4 will convert pin ADCINB4
	AdcbRegs.ADCSOC4CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

	//ADCINB5
	AdcbRegs.ADCSOC5CTL.bit.CHSEL = 0x05;  //SOC5 will convert pin ADCINB5
	AdcbRegs.ADCSOC5CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles

}

uint16_t sampleADC(uint16_t id) {
	uint16_t temp;

	// Convert B
	if (id >= 0x10) {
		AdcbRegs.ADCSOCFRC1.all = 0xFF; //Force Convert ADCINB
		while (AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0) {}  //Wait for ADCINT1
		AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1

	}

	// Convert A
	else {
		AdcaRegs.ADCSOCFRC1.all = 0xFF; //Force Convert ADCINA
		while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0) {}  //Wait for ADCINT1
		AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1
	}


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
