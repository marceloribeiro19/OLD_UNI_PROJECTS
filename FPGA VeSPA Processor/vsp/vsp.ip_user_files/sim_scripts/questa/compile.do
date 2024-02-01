vlib questa_lib/work
vlib questa_lib/msim

vlib questa_lib/msim/xpm
vlib questa_lib/msim/xil_defaultlib
vlib questa_lib/msim/blk_mem_gen_v8_4_7

vmap xpm questa_lib/msim/xpm
vmap xil_defaultlib questa_lib/msim/xil_defaultlib
vmap blk_mem_gen_v8_4_7 questa_lib/msim/blk_mem_gen_v8_4_7

vlog -work xpm -64 -incr -mfcu  -sv \
"/tools/Xilinx/Vivado/2023.2/data/ip/xpm/xpm_memory/hdl/xpm_memory.sv" \

vcom -work xpm -64 -93  \
"/tools/Xilinx/Vivado/2023.2/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib -64 -incr -mfcu  \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Unify_Instruc.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Data31_to_4x8b.v" \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ip/design_1_Conct_Out_RAM_0_1/sim/design_1_Conct_Out_RAM_0_1.v" \

vlog -work blk_mem_gen_v8_4_7 -64 -incr -mfcu  \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ipshared/3c0c/simulation/blk_mem_gen_v8_4.v" \

vlog -work xil_defaultlib -64 -incr -mfcu  \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ip/design_1_RAM_IP_31_24_0/sim/design_1_RAM_IP_31_24_0.v" \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ip/design_1_RAM_IP_31_24_1/sim/design_1_RAM_IP_31_24_1.v" \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ip/design_1_RAM_IP_31_24_2/sim/design_1_RAM_IP_31_24_2.v" \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ip/design_1_blk_mem_gen_0_0_1/sim/design_1_blk_mem_gen_0_0.v" \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/ip/design_1_SliptD_IN_RAM_0_1/sim/design_1_SliptD_IN_RAM_0_1.v" \
"../../../../../Downloads/recente/vespa up/vsp.gen/sources_1/bd/design_1/sim/design_1.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/ALU.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/CondCode.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Datapath.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Decode_Instruction.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/HALT_AND_CLK.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Instruction_Memory.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Mux32b_2x1.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Mux5b_2x1.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/PCAdder.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/ProgramCounter.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/RegFile.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/Toplyr.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/VeSPA_CPU.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/new/control_unit.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sources_1/imports/hdl/design_1_wrapper.v" \
"../../../../../Downloads/recente/vespa up/vespa.srcs/sim_1/new/Toplyr_TB.v" \

vlog -work xil_defaultlib \
"glbl.v"

