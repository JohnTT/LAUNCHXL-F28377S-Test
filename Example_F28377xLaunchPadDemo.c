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

//
// Globals
//
Uint16 AdcaResults[RESULTS_BUFFER_SIZE];
Uint16 resultsIndex;

// FILE -> printf variables
volatile int status = 0;
volatile FILE *fid;

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
//    InitGpio(); // Skipped for this example

    // For this example, only init the pins for the SCI-A port.
	EALLOW;
	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 1;
	GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 1;
	GpioCtrlRegs.GPCGMUX2.bit.GPIO84 = 1;
	GpioCtrlRegs.GPCGMUX2.bit.GPIO85 = 1;
	EDIS;

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

    //Redirect STDOUT to SCI
    scia_init();
	status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write,
			SCI_lseek, SCI_unlink, SCI_rename);
	fid = fopen("scia", "w");
	freopen("scia:", "w", stdout);
	setvbuf(stdout, NULL, _IONBF, 0);

//
// Configure the ADC and power it up
//
    ConfigureADC();

//
// Setup the ADC for continuous conversions on channel 0
//
    SetupADCContinuous(0);

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

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
//		putchar(0x1B);
//		putchar('[');
//		putchar('u');
    	printf("We main while\n\r");
//        //
//        //enable ADCINT flags
//        //
//        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
//        AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;
//        AdcaRegs.ADCINTSEL3N4.bit.INT3E = 1;
//        AdcaRegs.ADCINTSEL3N4.bit.INT4E = 1;
//        AdcaRegs.ADCINTFLGCLR.all = 0x000F;
//
//        //
//        //initialize results index
//        //
//        resultsIndex = 0;
//
//        //
//        //software force start SOC0 to SOC7
//        //
//        AdcaRegs.ADCSOCFRC1.all = 0x00FF;
//
//        //
//        //keep taking samples until the results buffer is full
//        //
//        while(resultsIndex < RESULTS_BUFFER_SIZE)
//        {
//            //
//            //wait for first set of 8 conversions to complete
//            //
//            while(0 == AdcaRegs.ADCINTFLG.bit.ADCINT3);
//
//            //
//            //clear both INT flags generated by first 8 conversions
//            //
//            AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
//            AdcaRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;
//
//            //
//            //save results for first 8 conversions
//            //
//            //note that during this time, the second 8 conversions have
//            //already been triggered by EOC6->ADCIN1 and will be actively
//            //converting while first 8 results are being saved
//            //
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT0;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT1;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT2;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT3;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT4;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT5;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT6;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT7;
//
//            //
//            //wait for the second set of 8 conversions to complete
//            //
//            while(0 == AdcaRegs.ADCINTFLG.bit.ADCINT4);
//
//            //
//            //clear both INT flags generated by second 8 conversions
//            //
//            AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
//            AdcaRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;
//
//            //
//            //save results for second 8 conversions
//            //
//            //note that during this time, the first 8 conversions have
//            //already been triggered by EOC14->ADCIN2 and will be actively
//            //converting while second 8 results are being saved
//            //
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT8;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT9;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT10;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT11;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT12;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT13;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT14;
//            AdcaResults[resultsIndex++] = AdcaResultRegs.ADCRESULT15;
//        }
//
//        //
//        //disable all ADCINT flags to stop sampling
//        //
//        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0;
//        AdcaRegs.ADCINTSEL1N2.bit.INT2E = 0;
//        AdcaRegs.ADCINTSEL3N4.bit.INT3E = 0;
//        AdcaRegs.ADCINTSEL3N4.bit.INT4E = 0;
//
//        //
//        //at this point, AdcaResults[] contains a sequence of conversions
//        //from the selected channel
//        //
//
//        //
//        //software breakpoint, hit run again to get updated conversions
//        //
//
//        asm("   ESTOP0");
		DELAY_US(1000*1000);
    }while(1);
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCContinuous - setup the ADC to continuously convert on one channel
//
void SetupADCContinuous(Uint16 channel)
{
    Uint16 acqps;

    //
    //determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC1CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC2CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC3CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC4CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC5CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC6CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC7CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC8CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC9CTL.bit.CHSEL  = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC10CTL.bit.CHSEL = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC11CTL.bit.CHSEL = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC12CTL.bit.CHSEL = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC13CTL.bit.CHSEL = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC14CTL.bit.CHSEL = channel;  //SOC will convert on channel
    AdcaRegs.ADCSOC15CTL.bit.CHSEL = channel;  //SOC will convert on channel

    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC6CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC7CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC9CTL.bit.ACQPS  = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC10CTL.bit.ACQPS = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC11CTL.bit.ACQPS = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC12CTL.bit.ACQPS = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC13CTL.bit.ACQPS = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC14CTL.bit.ACQPS = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles
    AdcaRegs.ADCSOC15CTL.bit.ACQPS = acqps;    //sample window is acqps +
                                               //1 SYSCLK cycles

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0; //disable INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 0; //disable INT2 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT3E = 0; //disable INT3 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT4E = 0; //disable INT4 flag

    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;
    AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 0;
    AdcaRegs.ADCINTSEL3N4.bit.INT3CONT = 0;
    AdcaRegs.ADCINTSEL3N4.bit.INT4CONT = 0;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 6;  //end of SOC6 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 14; //end of SOC14 will set INT2 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT3SEL = 7;  //end of SOC7 will set INT3 flag
    AdcaRegs.ADCINTSEL3N4.bit.INT4SEL = 15; //end of SOC15 will set INT4 flag

    //
    //ADCINT2 will trigger first 8 SOCs
    //
    AdcaRegs.ADCINTSOCSEL1.bit.SOC0 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC1 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC2 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC3 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC4 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC5 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC6 = 2;
    AdcaRegs.ADCINTSOCSEL1.bit.SOC7 = 2;

    //
    //ADCINT1 will trigger second 8 SOCs
    //
    AdcaRegs.ADCINTSOCSEL2.bit.SOC8 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC9 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC10 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC11 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC12 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC13 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC14 = 1;
    AdcaRegs.ADCINTSOCSEL2.bit.SOC15 = 1;
    EDIS;
}

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
