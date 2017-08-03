################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/cmd/2837xS_Generic_FLASH_lnk.cmd \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/cmd/F2837xS_Headers_nonBIOS.cmd 

ASM_SRCS += \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_CodeStartBranch.asm \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_usDelay.asm \
C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/profilec.asm \
C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/rampgc.asm \
C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/sgti1c.asm 

ASM_UPPER_SRCS += \
C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/SINTB360.ASM 

C_SRCS += \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Adc.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_CpuTimers.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_DefaultISR.c \
C:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_common/source/F2837xS_EPwm.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/source/F2837xS_GlobalVariableDefs.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Gpio.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_PieCtrl.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_PieVect.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Sci.c \
C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_SysCtrl.c \
../LAUNCHXL-F28377S-Test.c \
../sci_io.c 

OBJS += \
./F2837xS_Adc.obj \
./F2837xS_CodeStartBranch.obj \
./F2837xS_CpuTimers.obj \
./F2837xS_DefaultISR.obj \
./F2837xS_EPwm.obj \
./F2837xS_GlobalVariableDefs.obj \
./F2837xS_Gpio.obj \
./F2837xS_PieCtrl.obj \
./F2837xS_PieVect.obj \
./F2837xS_Sci.obj \
./F2837xS_SysCtrl.obj \
./F2837xS_usDelay.obj \
./LAUNCHXL-F28377S-Test.obj \
./SINTB360.obj \
./profilec.obj \
./rampgc.obj \
./sci_io.obj \
./sgti1c.obj 

ASM_DEPS += \
./F2837xS_CodeStartBranch.d \
./F2837xS_usDelay.d \
./profilec.d \
./rampgc.d \
./sgti1c.d 

C_DEPS += \
./F2837xS_Adc.d \
./F2837xS_CpuTimers.d \
./F2837xS_DefaultISR.d \
./F2837xS_EPwm.d \
./F2837xS_GlobalVariableDefs.d \
./F2837xS_Gpio.d \
./F2837xS_PieCtrl.d \
./F2837xS_PieVect.d \
./F2837xS_Sci.d \
./F2837xS_SysCtrl.d \
./LAUNCHXL-F28377S-Test.d \
./sci_io.d 

ASM_UPPER_DEPS += \
./SINTB360.d 

ASM_UPPER_DEPS__QUOTED += \
"SINTB360.d" 

C_DEPS__QUOTED += \
"F2837xS_Adc.d" \
"F2837xS_CpuTimers.d" \
"F2837xS_DefaultISR.d" \
"F2837xS_EPwm.d" \
"F2837xS_GlobalVariableDefs.d" \
"F2837xS_Gpio.d" \
"F2837xS_PieCtrl.d" \
"F2837xS_PieVect.d" \
"F2837xS_Sci.d" \
"F2837xS_SysCtrl.d" \
"LAUNCHXL-F28377S-Test.d" \
"sci_io.d" 

OBJS__QUOTED += \
"F2837xS_Adc.obj" \
"F2837xS_CodeStartBranch.obj" \
"F2837xS_CpuTimers.obj" \
"F2837xS_DefaultISR.obj" \
"F2837xS_EPwm.obj" \
"F2837xS_GlobalVariableDefs.obj" \
"F2837xS_Gpio.obj" \
"F2837xS_PieCtrl.obj" \
"F2837xS_PieVect.obj" \
"F2837xS_Sci.obj" \
"F2837xS_SysCtrl.obj" \
"F2837xS_usDelay.obj" \
"LAUNCHXL-F28377S-Test.obj" \
"SINTB360.obj" \
"profilec.obj" \
"rampgc.obj" \
"sci_io.obj" \
"sgti1c.obj" 

ASM_DEPS__QUOTED += \
"F2837xS_CodeStartBranch.d" \
"F2837xS_usDelay.d" \
"profilec.d" \
"rampgc.d" \
"sgti1c.d" 

C_SRCS__QUOTED += \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Adc.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_CpuTimers.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_DefaultISR.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_common/source/F2837xS_EPwm.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/source/F2837xS_GlobalVariableDefs.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Gpio.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_PieCtrl.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_PieVect.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Sci.c" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_SysCtrl.c" \
"../LAUNCHXL-F28377S-Test.c" \
"../sci_io.c" 

ASM_SRCS__QUOTED += \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_CodeStartBranch.asm" \
"C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_usDelay.asm" \
"C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/profilec.asm" \
"C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/rampgc.asm" \
"C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/sgti1c.asm" 

ASM_UPPER_SRCS__QUOTED += \
"C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/SINTB360.ASM" 


