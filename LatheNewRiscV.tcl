# Copyright (C) 2023  Intel Corporation. All rights reserved.
# Your use of Intel Corporation's design tools, logic functions 
# and other software and tools, and any partner logic 
# functions, and any output files from any of the foregoing 
# (including device programming or simulation files), and any 
# associated documentation or information are expressly subject 
# to the terms and conditions of the Intel Program License 
# Subscription Agreement, the Intel Quartus Prime License Agreement,
# the Intel FPGA IP License Agreement, or other applicable license
# agreement, including, without limitation, that your use is for
# the sole purpose of programming logic devices manufactured by
# Intel and sold by Intel or its authorized distributors.  Please
# refer to the applicable agreement for further details, at
# https://fpgasoftware.intel.com/eula.

# Quartus Prime: Generate Tcl File for Project
# File: LatheNewRiscV.tcl
# Generated on: Mon Dec  4 16:52:39 2023

# Load Quartus Prime Tcl Project package
package require ::quartus::project

set need_to_close_project 0
set make_assignments 1

# Check that the right project is open
if {[is_project_open]} {
	if {[string compare $quartus(project) "LatheNewRiscV"]} {
		puts "Project LatheNewRiscV is not open"
		set make_assignments 0
	}
} else {
	# Only open if not already open
	if {[project_exists LatheNewRiscV]} {
		project_open -revision LatheNewRiscV LatheNewRiscV
	} else {
		project_new -revision LatheNewRiscV LatheNewRiscV
	}
	set need_to_close_project 1
}

# Make assignments
if {$make_assignments} {
	set_global_assignment -name FAMILY "Cyclone IV E"
	set_global_assignment -name DEVICE EP4CE22F17C6
	set_global_assignment -name TOP_LEVEL_ENTITY LatheTop
	set_global_assignment -name ORIGINAL_QUARTUS_VERSION 22.1STD.1
	set_global_assignment -name PROJECT_CREATION_TIME_DATE "04:46:33  JULY 27, 2023"
	set_global_assignment -name LAST_QUARTUS_VERSION "22.1std.2 Lite Edition"
	set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
	set_global_assignment -name MIN_CORE_JUNCTION_TEMP 0
	set_global_assignment -name MAX_CORE_JUNCTION_TEMP 85
	set_global_assignment -name ERROR_CHECK_FREQUENCY_DIVISOR 1
	set_global_assignment -name NOMINAL_CORE_SUPPLY_VOLTAGE 1.2V
	set_global_assignment -name NUM_PARALLEL_PROCESSORS ALL
	set_global_assignment -name PARTITION_NETLIST_TYPE SOURCE -section_id Top
	set_global_assignment -name PARTITION_FITTER_PRESERVATION_LEVEL PLACEMENT_AND_ROUTING -section_id Top
	set_global_assignment -name PARTITION_COLOR 16764057 -section_id Top
	set_global_assignment -name POWER_PRESET_COOLING_SOLUTION "23 MM HEAT SINK WITH 200 LFPM AIRFLOW"
	set_global_assignment -name POWER_BOARD_THERMAL_MODEL "NONE (CONSERVATIVE)"
	set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008
	set_global_assignment -name VHDL_SHOW_LMF_MAPPING_MESSAGES OFF
	set_global_assignment -name ENABLE_SIGNALTAP ON
	set_global_assignment -name USE_SIGNALTAP_FILE stp2.stp
	set_global_assignment -name SLD_NODE_CREATOR_ID 110 -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_ENTITY_NAME sld_signaltap -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_RAM_BLOCK_TYPE=AUTO" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_NODE_INFO=805334528" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_POWER_UP_TRIGGER=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_ATTRIBUTE_MEM_MODE=OFF" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_STATE_FLOW_USE_GENERATED=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_STATE_BITS=11" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_BUFFER_FULL_STOP=1" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_CURRENT_RESOURCE_WIDTH=1" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_TRIGGER_LEVEL=1" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_TRIGGER_IN_ENABLED=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_TRIGGER_PIPELINE=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_RAM_PIPELINE=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_COUNTER_PIPELINE=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_TRIGGER_LEVEL_PIPELINE=1" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_ENABLE_ADVANCED_TRIGGER=1" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_STORAGE_QUALIFIER_INVERSION_MASK_LENGTH=0" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_INCREMENTAL_ROUTING=1" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_INVERSION_MASK=00000000000000000000" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_INVERSION_MASK_LENGTH=20" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_SEGMENT_SIZE=64" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_SAMPLE_DEPTH=64" -section_id auto_signaltap_0
	set_global_assignment -name EDA_SIMULATION_TOOL "Questa Intel FPGA (VHDL)"
	set_global_assignment -name EDA_TIME_SCALE "1 ps" -section_id eda_simulation
	set_global_assignment -name EDA_OUTPUT_DATA_FORMAT VHDL -section_id eda_simulation
	set_global_assignment -name EDA_GENERATE_FUNCTIONAL_NETLIST OFF -section_id eda_board_design_timing
	set_global_assignment -name EDA_GENERATE_FUNCTIONAL_NETLIST OFF -section_id eda_board_design_symbol
	set_global_assignment -name EDA_GENERATE_FUNCTIONAL_NETLIST OFF -section_id eda_board_design_signal_integrity
	set_global_assignment -name EDA_GENERATE_FUNCTIONAL_NETLIST OFF -section_id eda_board_design_boundary_scan
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_DATA_BITS=33" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_TRIGGER_BITS=12" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_STORAGE_QUALIFIER_BITS=33" -section_id auto_signaltap_0
	set_global_assignment -name SLD_NODE_PARAMETER_ASSIGNMENT "SLD_ADVANCED_TRIGGER_ENTITY=sld_reserved_LatheNewRiscV_auto_signaltap_0_1_764d," -section_id auto_signaltap_0
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/RegDef.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ExtDataRec.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/DbgRecord.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/IORecord.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/FpgaLatheBits.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/FpgaLatheRec.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/FpgaLatheFunc.vhd
	set_global_assignment -name QSYS_FILE ../LatheNew/Proj/SystemClk.qsys
	set_global_assignment -name QIP_FILE ../LatheNew/Proj/CmpTmrMem2.qip
	set_global_assignment -name QIP_FILE ../LatheNew/Proj/CMem.qip
	set_global_assignment -name QIP_FILE ../LatheNew/Proj/RdMem.qip
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/DoutDelay.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/DbgMap.vhd
	set_global_assignment -name VHDL_FILE ../Encoder/VHDL/Display.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ShiftOp.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ShiftOpLoad.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ShiftOpSel.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ShiftOutN.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ShiftOutNS.vhd
	set_global_assignment -name VHDL_FILE ../Encoder/VHDL/CmpTmrNewMem.vhd
	set_global_assignment -name VHDL_FILE ../Encoder/VHDL/IntTmrNew.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/SPI.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/PulseGen.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ClockEnaN.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/ClockA.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/PhaseCounter.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/IndexClocks.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Conversion.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Encoder.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/QuadEncoder.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/SyncAccelNew.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/PWM.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Spindle.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/FreqGen.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/FreqGenCtr.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/CtlReg.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Jog.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/SyncAccelDistNew.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Axis.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/neorv32/neorv32_package.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../LatheNew/neorv32/neorv32_cfs.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/MpgRecord.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Mpg.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Fifo.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_alu.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_control.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_cp_bitmanip.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_cp_cfu.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_cp_fpu.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_cp_muldiv.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_cp_shifter.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_decompressor.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_lsu.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_pmp.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_cpu_regfile.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_crc.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_dcache.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_debug_dm.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_debug_dtm.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_dma.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_fifo.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_gpio.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_gptmr.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_icache.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_intercon.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_mtime.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_neoled.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_onewire.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_pwm.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_sdi.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_slink.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_spi.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_sysinfo.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_trng.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_twi.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_uart.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_wdt.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_wishbone.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_xip.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_xirq.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_imem.entity.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_dmem.entity.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_application_image.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_bootloader_image.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/neorv32_boot_rom.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/mem/neorv32_imem.default.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../../../neorv32/rtl/core/mem/neorv32_dmem.default.vhd -library neorv32
	set_global_assignment -name VHDL_FILE ../LatheNew/neorv32/neorv32_top.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/LatheCtl.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/Interface.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/LatheInterface.vhd
	set_global_assignment -name VHDL_FILE ../LatheNew/VHDL/LatheTopNanoRiscV.vhd
	set_global_assignment -name SDC_FILE LatheNewRiscV.sdc
	set_global_assignment -name SIGNALTAP_FILE stp2.stp
	set_global_assignment -name SLD_FILE db/stp2_auto_stripped.stp
	set_location_assignment PIN_R8 -to sysClk
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to sysClk
	set_location_assignment PIN_A15 -to led[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[0]
	set_location_assignment PIN_A13 -to led[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[1]
	set_location_assignment PIN_B13 -to led[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[2]
	set_location_assignment PIN_A11 -to led[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[3]
	set_location_assignment PIN_D1 -to led[4]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[4]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[4]
	set_location_assignment PIN_F3 -to led[5]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[5]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[5]
	set_location_assignment PIN_B1 -to led[6]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[6]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[6]
	set_location_assignment PIN_L3 -to led[7]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to led[7]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to led[7]
	set_location_assignment PIN_J15 -to rstn_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to rstn_i
	set_location_assignment PIN_A8 -to dsel
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dsel
	set_location_assignment PIN_D3 -to dclk
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dclk
	set_location_assignment PIN_B8 -to din
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to din
	set_location_assignment PIN_C3 -to dout
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dout
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dout
	set_location_assignment PIN_A2 -to zDoneInt
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to zDoneInt
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to zDoneInt
	set_location_assignment PIN_A3 -to xDoneInt
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDoneInt
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to xDoneInt
	set_location_assignment PIN_A4 -to bIn
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to bIn
	set_location_assignment PIN_B5 -to aIn
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to aIn
	set_location_assignment PIN_A5 -to syncIn
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to syncIn
	set_location_assignment PIN_B6 -to bufOut[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to bufOut[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to bufOut[0]
	set_location_assignment PIN_A6 -to bufOut[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to bufOut[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to bufOut[1]
	set_location_assignment PIN_B7 -to bufOut[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to bufOut[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to bufOut[2]
	set_location_assignment PIN_D6 -to bufOut[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to bufOut[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to bufOut[3]
	set_location_assignment PIN_A7 -to dbg[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[1]
	set_location_assignment PIN_C6 -to dbg[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[0]
	set_location_assignment PIN_C8 -to dbg[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[3]
	set_location_assignment PIN_E6 -to dbg[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[2]
	set_location_assignment PIN_E7 -to dbg[5]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[5]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[5]
	set_location_assignment PIN_D8 -to dbg[4]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[4]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[4]
	set_location_assignment PIN_E8 -to dbg[7]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[7]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[7]
	set_location_assignment PIN_F8 -to dbg[6]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg[6]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg[6]
	set_location_assignment PIN_F9 -to seg[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[2]
	set_location_assignment PIN_C9 -to seg[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[1]
	set_location_assignment PIN_D9 -to seg[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[3]
	set_location_assignment PIN_E11 -to seg[6]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[6]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[6]
	set_location_assignment PIN_E10 -to seg[4]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[4]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[4]
	set_location_assignment PIN_C11 -to seg[5]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[5]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[5]
	set_location_assignment PIN_B11 -to seg[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to seg[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to seg[0]
	set_location_assignment PIN_A12 -to anode[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to anode[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to anode[3]
	set_location_assignment PIN_D11 -to anode[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to anode[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to anode[0]
	set_location_assignment PIN_D12 -to anode[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to anode[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to anode[2]
	set_location_assignment PIN_B12 -to anode[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to anode[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to anode[1]
	set_location_assignment PIN_T9 -to zDro[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to zDro[1]
	set_location_assignment PIN_F13 -to zDro[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to zDro[0]
	set_location_assignment PIN_R9 -to xDro[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDro[0]
	set_location_assignment PIN_T15 -to xDro[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDro[1]
	set_location_assignment PIN_T14 -to zMpg[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to zMpg[0]
	set_location_assignment PIN_T13 -to zMpg[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to zMpg[1]
	set_location_assignment PIN_R13 -to xMpg[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xMpg[0]
	set_location_assignment PIN_T12 -to xMpg[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xMpg[1]
	set_location_assignment PIN_R12 -to extOut[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to extOut[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to extOut[0]
	set_location_assignment PIN_T11 -to extOut[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to extOut[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to extOut[1]
	set_location_assignment PIN_P15 -to extOut[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to extOut[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to extOut[2]
	set_location_assignment PIN_T10 -to pinOut[8]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[8]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[8]
	set_location_assignment PIN_R11 -to pinOut[9]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[9]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[9]
	set_location_assignment PIN_P11 -to pinOut[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[0]
	set_location_assignment PIN_R10 -to pinOut[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[1]
	set_location_assignment PIN_N12 -to pinOut[10]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[10]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[10]
	set_location_assignment PIN_P9 -to pinOut[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[2]
	set_location_assignment PIN_N9 -to pinOut[11]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[11]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[11]
	set_location_assignment PIN_N11 -to pinOut[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[3]
	set_location_assignment PIN_L16 -to pinOut[4]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[4]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[4]
	set_location_assignment PIN_K16 -to pinOut[5]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[5]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[5]
	set_location_assignment PIN_R16 -to pinOut[6]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[6]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[6]
	set_location_assignment PIN_L15 -to pinOut[7]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinOut[7]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to pinOut[7]
	set_location_assignment PIN_R14 -to pinIn[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[0]
	set_location_assignment PIN_N16 -to pinIn[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[1]
	set_location_assignment PIN_N15 -to pinIn[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[2]
	set_location_assignment PIN_P14 -to pinIn[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[3]
	set_location_assignment PIN_P16 -to pinIn[4]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[4]
	set_location_assignment PIN_J14 -to pinIn[5]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[5]
	set_location_assignment PIN_J13 -to pinIn[6]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[6]
	set_location_assignment PIN_K15 -to pinIn[7]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[7]
	set_location_assignment PIN_J16 -to pinIn[8]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[8]
	set_location_assignment PIN_L13 -to pinIn[9]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[9]
	set_location_assignment PIN_M10 -to pinIn[10]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[10]
	set_location_assignment PIN_N14 -to pinIn[11]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[11]
	set_location_assignment PIN_L14 -to pinIn[12]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to pinIn[12]
	set_location_assignment PIN_G15 -to dbg_rxd_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg_rxd_i
	set_location_assignment PIN_G16 -to dbg_txd_o
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to dbg_txd_o
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to dbg_txd_o
	set_location_assignment PIN_C16 -to rem_rxd_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to rem_rxd_i
	set_location_assignment PIN_C14 -to rem_txd_o
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to rem_txd_o
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to rem_txd_o
	set_location_assignment PIN_E16 -to jtag_tdi_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to jtag_tdi_i
	set_location_assignment PIN_M16 -to jtag_tms_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to jtag_tms_i
	set_location_assignment PIN_E15 -to jtag_tck_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to jtag_tck_i
	set_location_assignment PIN_A14 -to jtag_tdo_o
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to jtag_tdo_o
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to jtag_tdo_o
	set_location_assignment PIN_B16 -to jtag_trst_i
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to jtag_trst_i
	set_location_assignment PIN_C15 -to xdbg[0]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDbg[0]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to xDbg[0]
	set_location_assignment PIN_D16 -to xDbg[1]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDbg[1]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to xDbg[1]
	set_location_assignment PIN_D15 -to xDbg[2]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDbg[2]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to xDbg[2]
	set_location_assignment PIN_D14 -to xDbg[3]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDbg[3]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to xDbg[3]
	set_location_assignment PIN_F15 -to xDbg[4]
	set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to xDbg[4]
	set_instance_assignment -name CURRENT_STRENGTH_NEW 8MA -to xDbg[4]
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_clk -to "Clock:pllClock|clockOut" -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[29] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[20] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[9] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[3] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[12] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[7] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[10] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[16] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[0] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:x_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[0]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[1] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:x_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[1]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[2] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:x_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[2]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[3] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:x_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[3]" -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[14] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[4] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[0]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[5] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[1]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[6] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[2]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[7] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|distCtr[3]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[0] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|extDone" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[8] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|extDone" -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[25] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[22] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[1] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.checkAccel" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[2] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.enabled" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[3] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.syncIdle" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[4] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.syncInit" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[5] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.updAccel" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[9] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.checkAccel" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[10] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.enabled" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[11] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.syncIdle" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[12] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.syncInit" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[13] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|SyncAccelDistJog:AxisSyncAccel|syncState.updAccel" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[14] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|ch" -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[8] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[13] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[18] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[19] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[24] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[28] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[6] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|stepOut" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[7] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|ena" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[8] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|state.run" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[9] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|zCh" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[10] -to rem_rxd_i -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_trigger_in[11] -to rem_txd_o -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[15] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|Axis:z_Axis|stepOut" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[16] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[0]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[17] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[1]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[18] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[2]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[19] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[3]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[20] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[4]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[21] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[5]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[22] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|CtlReg:clk_reg|data[6]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[23] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|ena" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[24] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|freqCounter[0]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[25] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|freqCounter[1]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[26] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|freqCounter[2]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[27] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|freqCounter[3]" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[28] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|pulseOut" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[29] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|FreqGenCtr:dbgFreq_gen|state.run" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[30] -to "LatheInterface:latheInt|LatheCtl:latheCtlProc|zCh" -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[31] -to rem_rxd_i -section_id auto_signaltap_0
	set_instance_assignment -name CONNECT_TO_SLD_NODE_ENTITY_PORT acq_data_in[32] -to rem_txd_o -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[0] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[1] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[2] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[4] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[5] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[6] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[11] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[15] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[17] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[21] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[23] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[26] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[27] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[30] -to auto_signaltap_0|vcc -section_id auto_signaltap_0
	set_instance_assignment -name POST_FIT_CONNECT_TO_SLD_NODE_ENTITY_PORT crc[31] -to auto_signaltap_0|gnd -section_id auto_signaltap_0
	set_instance_assignment -name PARTITION_HIERARCHY root_partition -to | -section_id Top

	# Commit assignments
	export_assignments

	# Close project
	if {$need_to_close_project} {
		project_close
	}
}
