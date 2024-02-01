#!/usr/bin/env bash
# ****************************************************************************
# Vivado (TM) v2023.2 (64-bit)
#
# Filename    : compile.sh
# Simulator   : AMD Vivado Simulator
# Description : Script for compiling the simulation design source files
#
# Generated by Vivado on Sat Nov 25 21:45:40 WET 2023
# SW Build 4029153 on Fri Oct 13 20:13:54 MDT 2023
#
# Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
# Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
#
# usage: compile.sh
#
# ****************************************************************************
set -Eeuo pipefail
# compile Verilog/System Verilog design sources
echo "xvlog --incr --relax -prj Toplyr_TB_vlog.prj"
xvlog --incr --relax -prj Toplyr_TB_vlog.prj 2>&1 | tee compile.log

echo "Waiting for jobs to finish..."
echo "No pending jobs, compilation finished."
