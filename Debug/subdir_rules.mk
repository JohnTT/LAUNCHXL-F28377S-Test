################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
F2837xS_Adc.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Adc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_Adc.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_CodeStartBranch.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_CodeStartBranch.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_CpuTimers.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_CpuTimers.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_CpuTimers.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_DefaultISR.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_DefaultISR.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_DefaultISR.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_EPwm.obj: C:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_common/source/F2837xS_EPwm.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_EPwm.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_GlobalVariableDefs.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/source/F2837xS_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_GlobalVariableDefs.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_Gpio.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Gpio.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_Gpio.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_PieCtrl.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_PieCtrl.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_PieCtrl.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_PieVect.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_PieVect.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_PieVect.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_Sci.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_Sci.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_Sci.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_SysCtrl.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_SysCtrl.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_SysCtrl.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

F2837xS_usDelay.obj: C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/source/F2837xS_usDelay.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="F2837xS_usDelay.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

LAUNCHXL-F28377S-Test.obj: ../LAUNCHXL-F28377S-Test.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="LAUNCHXL-F28377S-Test.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

profilec.obj: C:/ti/controlSUITE/libs/dsp/SGEN/v101/source/C28x_SGEN_LIB/profilec.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="profilec.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

sci_io.obj: ../sci_io.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -ml -mt --float_support=fpu32 --tmu_support=tmu0 --cla_support=cla0 --vcu_support=vcu0 --include_path="C:/ti/controlSUITE/libs/dsp/SGEN/v101/include" --include_path="C:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/packages/ti/xdais" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_headers/include" --include_path="C:/ti/controlSUITE/device_support/F2837xS/v180/F2837xS_common/include" --include_path="C:/ti/controlSUITE/libs/math/IQmath/v160/include" --include_path="C:/ti/controlSUITE/libs/math/FPUfastRTS/V100/include" --advice:performance=all -g --define="_DEBUG" --define=_FLASH --define=_LAUNCHXL_F28377S --define="LARGE_MODEL" --quiet --diag_warning=225 --verbose_diagnostics --issue_remarks --cdebug_asm_data --output_all_syms --preproc_with_compile --preproc_dependency="sci_io.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


