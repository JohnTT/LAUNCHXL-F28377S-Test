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

void ConfigureADC(void);
void SetupADCContinuous(Uint16 channel);

//
// Defines
//
#define RESULTS_BUFFER_SIZE 256 //buffer for storing conversion results
                                //(size must be multiple of 16)

#define LED_GPIO_RED    12
#define LED_GPIO_BLUE	13

enum ADCINID {
	ADCINID_B4 = 0x14
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


int16_t sampleADC(int id)
{
	int16_t temp;

	switch (id) {
	case ADCINID_B4:
		//Force start of conversion on SOC4
		AdcbRegs.ADCSOCFRC1.bit.SOC4 = 1;

		//Wait for end of conversion.
		while (AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0) {
		}  //Wait for ADCINT1
		AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1

		temp = AdcbResultRegs.ADCRESULT4;
		break;
	default:
		//Force start of conversion on SOC0
		AdcaRegs.ADCSOCFRC1.bit.SOC0 = 1;

		//Wait for end of conversion.
		while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0) {
		}  //Wait for ADCINT1
		AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1

		//Get temp sensor sample result from SOC0
		temp = AdcaResultRegs.ADCRESULT0;
		break;

	}


    //Return the raw temperature because the devices don't have slope/offset values
    return(temp);

}

void scia_init()
{

    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE

	SciaRegs.SCICTL2.bit.TXINTENA =1;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;

	SciaRegs.SCIHBAUD.all    =0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
    SciaRegs.SCILBAUD.all    =53;

	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

    return;
}

void main(void)
{

	#ifdef _FLASH
	    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
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
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

// LED initializations
    {
		GPIO_SetupPinMux(LED_GPIO_RED, GPIO_MUX_CPU1, 0);
		GPIO_SetupPinOptions(LED_GPIO_RED, GPIO_OUTPUT, GPIO_PUSHPULL);

		GPIO_SetupPinMux(LED_GPIO_BLUE, GPIO_MUX_CPU1, 0);
		GPIO_SetupPinOptions(LED_GPIO_BLUE, GPIO_OUTPUT, GPIO_PUSHPULL);

		GPIO_WritePin(LED_GPIO_RED, 1);
		GPIO_WritePin(LED_GPIO_BLUE, 1);
    }


// Printf via SCI initialization
    {
		// For this example, only init the pins for the SCI-A port.
		EALLOW;
		GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;
		GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;
		GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;
		GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
		EDIS;

		// Redirect STDOUT to SCI
		scia_init();
		status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read,
				SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
		fid = fopen("scia", "w");
		freopen("scia:", "w", stdout);
		setvbuf(stdout, NULL, _IONBF, 0);
    }





// ADC1 configuation
    {
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

		//ADCINA0
		AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x00;  //SOC0 will convert pin ADCINA0
		AdcaRegs.ADCSOC0CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
		AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
		AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
		AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

		//ADCINB4
		AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0x04;  //SOC0 will convert pin ADCINB4
		AdcbRegs.ADCSOC4CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
		AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 4; //end of SOC4 will set INT1 flag
		AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
		AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared


    }


//
// Initialize results buffer
//
    for(resultsIndex = 0; resultsIndex < RESULTS_BUFFER_SIZE; resultsIndex++)
    {
        AdcaResults[resultsIndex] = 0;
    }
    resultsIndex = 0;

//
// take conversions indefinitely in loop
//
    do
    {
    	printf("We main while\n\r");
    	printf("ADCINA0 Sample = %d\n\r", sampleADC(0));
    	printf("ADCINB4 Sample = %d\n\r", sampleADC(ADCINID_B4));
    	GPIO_WritePin(LED_GPIO_RED, redLED_state);
    	redLED_state = !redLED_state;

		DELAY_US(1000*1000);
    }while(1);
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
//void ConfigureADC(void)
//{
//    EALLOW;
//
//    //
//    //write configurations
//    //
//    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
//    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//
//    //
//    //Set pulse positions to late
//    //
//    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
//
//    //
//    //power up the ADC
//    //
//    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
//
//    //
//    //delay for 1ms to allow ADC time to power up
//    //
//    DELAY_US(1000);
//
//    EDIS;
//}

//
// End of file
//






//
////###########################################################################
////
//// FILE:	Example_F28377xLaunchPadDemo.c
////
//// TITLE:	F28377S LaunchPad Out of Box Demo
////
//// DESCRIPTION:
////! \addtogroup f28377xX_example_list
////! <h1>Out of Box Demo (LaunchPadDemo)</h1>
////!
////!  This program is the demo program that comes pre-loaded on the F28377S
////!  LaunchPad development kit.  The program starts by flashing the two user
////!  LEDs. After a few seconds the LEDs stop flashing and the device starts
////!  sampling ADCIN14 once a second.  If the sample is greater than midscale
////!  the red LED on the board is lit, while if it is lower a blue LED is lit.
////!  Sample data is also display in a serial terminal via the boards back
////!  channel UART.  You may view this data by configuring a serial termainal
////!  to the correct COM port at 115200 Baud 8-N-1.
////!
////
////###########################################################################
//// $TI Release: 2837xS C/C++ Header Files V1.00 $
//// $Release Date: July 15, 2015 $
////###########################################################################
//
//
//#include <stdint.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include <file.h>
//
//#include "F28x_Project.h"     // DSP28x Headerfile
//#include "ti_ascii.h"
//#include "sci_io.h"
//
//#define CONV_WAIT 1L //Micro-seconds to wait for ADC conversion. Longer than necessary.
//
//extern void DSP28x_usDelay(Uint32 Count);
//
//static unsigned short indexX=0;
//static unsigned short indexY=0;
//
//const unsigned char escRed[] = {0x1B, 0x5B, '3','1', 'm'};
//const unsigned char escWhite[] = {0x1B, 0x5B, '3','7', 'm'};
//const unsigned char escLeft[] = {0x1B, 0x5B, '3','7', 'm'};
//const unsigned char pucTempString[] = "ADCIN14 Sample:     ";
//
//
//
//
//int16_t currentSample;
//
//
//
//int16_t sampleADC(void)
//{
//	int16_t temp;
//
//    //Force start of conversion on SOC0
//    AdcaRegs.ADCSOCFRC1.all = 0x03;
//
//    //Wait for end of conversion.
//    while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0){}  //Wait for ADCINT1
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        //Clear ADCINT1
//
//    //Get temp sensor sample result from SOC0
//    temp = AdcaResultRegs.ADCRESULT1;
//
//    //Return the raw temperature because the devices don't have slope/offset values
//    return(temp);
//
//}
//
//void drawTILogo(void)
//{
//    unsigned char ucChar, lastChar;
//
//    putchar('\n');
//    while(indexY<45)
//    {
//        if(indexY<45){
//            if(indexX<77){
//                ucChar = ti_ascii[indexY][indexX++] ;
//
//                //We are in the TI logo make it red
//                if(ucChar != '7' && lastChar=='7'){
//                    putchar(escRed[0]);
//                    putchar(escRed[1]);
//                    putchar(escRed[2]);
//                    putchar(escRed[3]);
//                    putchar(escRed[4]);
//                }
//
//                //We are in the TI logo make it red
//                if(ucChar == '7' && lastChar!='7'){
//                    putchar(escWhite[0]);
//                    putchar(escWhite[1]);
//                    putchar(escWhite[2]);
//                    putchar(escWhite[3]);
//                    putchar(escWhite[4]);
//                }
//
//                putchar(ucChar);
//                lastChar = ucChar;
//            }else{
//                ucChar = 10;
//                putchar(ucChar);
//                ucChar = 13;
//                putchar(ucChar);
//                indexX=0;
//                indexY++;
//            }
//        }
//    }
//}
//
//void clearTextBox(void)
//{
//
//    putchar(0x08);
//
//    // Move back 24 columns
//    putchar(0x1B);
//    putchar('[');
//    putchar('2');
//    putchar('6');
//    putchar('D');
//
//    // Move up 3 lines
//    putchar(0x1B);
//    putchar('[');
//    putchar('3');
//    putchar('A');
//
//    //Change to Red text
//    putchar(escRed[0]);
//    putchar(escRed[1]);
//    putchar(escRed[2]);
//    putchar(escRed[3]);
//    putchar(escRed[4]);
//
//    printf((char*)pucTempString);
//
//    // Move down 1 lines
//    putchar(0x1B);
//    putchar('[');
//    putchar('1');
//    putchar('B');
//
//    // Move back 20 columns
//    putchar(0x1B);
//    putchar('[');
//    putchar('2');
//    putchar('0');
//    putchar('D');
//
//    // Save cursor position
//    putchar(0x1B);
//    putchar('[');
//    putchar('s');
//
//}
//
//void updateDisplay(void)
//{
//    // Restore cursor position
//    putchar(0x1B);
//    putchar('[');
//    putchar('u');
//
//
//    printf("ADC Value: %d", currentSample);
//
//}
//
//// SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
//void scia_init()
//{
//
//    // Note: Clocks were turned on to the SCIA peripheral
//    // in the InitSysCtrl() function
//
// 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//
//	SciaRegs.SCICTL2.bit.TXINTENA =1;
//	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
//
//	SciaRegs.SCIHBAUD.all    =0x0000;  // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
//    SciaRegs.SCILBAUD.all    =53;
//
//	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
//
//    return;
//}
//
//
//
//void main()
//{
//    volatile int status = 0;
//    uint16_t i;
//    volatile FILE *fid;
//
//    // If running from flash copy RAM only functions to RAM
//#ifdef _FLASH
//    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
//#endif
//
//    // Initialize System Control:
//    // PLL, WatchDog, enable Peripheral Clocks
//    // This example function is found in the F2806x_SysCtrl.c file.
//       InitSysCtrl();
//
//
//
//    // For this example, only init the pins for the SCI-A port.
//    EALLOW;
//    GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;
//    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;
//    GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;
//    GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
//    EDIS;
//
//
//    //  Clear all interrupts and initialize PIE vector table:
//    // Disable CPU interrupts
//       DINT;
//
//    // Initialize PIE control registers to their default state.
//    // The default state is all PIE interrupts disabled and flags
//    // are cleared.
//    // This function is found in the F2806x_PieCtrl.c file.
//       InitPieCtrl();
//
//    // Disable CPU interrupts and clear all CPU interrupt flags:
//       IER = 0x0000;
//       IFR = 0x0000;
//
//    // Initialize the PIE vector table with pointers to the shell Interrupt
//    // Service Routines (ISR).
//    // This will populate the entire table, even if the interrupt
//    // is not used in this example.  This is useful for debug purposes.
//    // The shell ISR routines are found in F2806x_DefaultIsr.c.
//    // This function is found in F2806x_PieVect.c.
//       InitPieVectTable();
//
//
//    // Initialize SCIA
//    scia_init();
//
//    //Initialize GPIOs for the LEDs and turn them off
//    EALLOW;
//    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
//    GpioDataRegs.GPADAT.bit.GPIO12 = 1;
//    GpioDataRegs.GPADAT.bit.GPIO13 = 1;
//    EDIS;
//
//
//    // Enable global Interrupts and higher priority real-time debug events:
//    EINT;   // Enable Global interrupt INTM
//    ERTM;   // Enable Global realtime interrupt DBGM
//
//
//    // Configure the ADC:
//    // Initialize the ADC
//	EALLOW;
//
//	//write configurations
//	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
//	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
//    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
//
//	//Set pulse positions to late
//	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
//	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
//
//	//power up the ADCs
//	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
//	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
//
//	//delay for 1ms to allow ADC time to power up
//	DELAY_US(1000);
//
//
//   //ADCA
//   EALLOW;
//   AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x0E;  //SOC0 will convert pin ADCIN14
//   AdcaRegs.ADCSOC0CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
//   AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x0E;  //SOC1 will convert pin ADCIN14
//   AdcaRegs.ADCSOC1CTL.bit.ACQPS = 25; //sample window is acqps + 1 SYSCLK cycles
//   AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//   AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
//
//
//
//
//    //Redirect STDOUT to SCI
//    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write, SCI_lseek, SCI_unlink, SCI_rename);
//    fid = fopen("scia","w");
//    freopen("scia:", "w", stdout);
//    setvbuf(stdout, NULL, _IONBF, 0);
//
//    //Print a TI Logo to STDOUT
//    drawTILogo();
//
//    //Twiddle LEDs
//    GpioDataRegs.GPADAT.bit.GPIO12 = 0;
//    GpioDataRegs.GPADAT.bit.GPIO13 = 1;
//
//
//    for(i = 0; i < 50; i++){
//
//        GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1;
//        GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;
//        DELAY_US(50000);
//
//
//    }
//
//    //LEDs off
//    GpioDataRegs.GPADAT.bit.GPIO12 = 1;
//    GpioDataRegs.GPADAT.bit.GPIO13 = 1;
//
//
//    //Clear out one of the text boxes so we can write more info to it
//    clearTextBox();
//
//    currentSample = sampleADC();
//
//
//    //Main program loop - continually sample temperature
//    for(;;) {
//
//
//        //Sample ADCIN14
//        currentSample = sampleADC();
//
//        //Update the serial terminal output
//        updateDisplay();
//
//        //If the sample is above midscale light one LED
//        if(currentSample > 2048){
//    	    GpioDataRegs.GPADAT.all = 0x2000;
//        }else{
//            //Otherwise light the other
//    	    GpioDataRegs.GPADAT.all = 0x1000;
//        }
//
//        DELAY_US(1000000);
//
//    }
//}
//
//
//
//
