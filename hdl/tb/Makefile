# Verilator bit-parity bench (ADR-0007 M2).
# Usage: make [OTLG=/tmp/in.otlg]  (default: regenerate-free; needs a log path)
# Verilator from oss-cad-suite (versions pinned in hdl/README.md).

VERILATOR ?= $(HOME)/Projects/hardware/oss-cad-suite/bin/verilator
REPO_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
FUSION := $(REPO_ROOT)/fusion

VERILATOR_FLAGS = --cc --exe --build -j $(shell nproc) \
	--top-module predict_core --assert \
	-CFLAGS "-std=c++20 -I$(FUSION)" \
	-CFLAGS "-I/usr/local/include/eigen3"

all: obj_dir/Vpredict_core

obj_dir/Vpredict_core: tb_predict.cpp ../rtl/fixed_pkg.sv ../rtl/predict_core.sv
	$(VERILATOR) $(VERILATOR_FLAGS) --Mdir obj_dir $^

# Bit-parity over the 5 s trot (2500 steps). OTLG must exist; generate via:
#   PYTHONPATH=sim pixi run python -c \
#     "from otolith_sim.logger import record_puppet_log; record_puppet_log('/tmp/parity.otlg')"
OTLG ?= /tmp/parity.otlg
run: all
	./obj_dir/Vpredict_core $(OTLG)

# 32x64 multiply-narrow kernel parity (ADR-0007 M4). Separate Mdir so the
# full-core build is untouched.
MUL48_FLAGS = --cc --exe --build -j $(shell nproc) \
	--top-module mul48_kernel --assert \
	-CFLAGS "-std=c++20 -I$(FUSION)"

mul48: obj_dir_mul48/Vmul48_kernel

obj_dir_mul48/Vmul48_kernel: tb_mul48.cpp ../rtl/fixed_pkg.sv ../rtl/mul48_kernel.sv
	$(VERILATOR) $(MUL48_FLAGS) --Mdir obj_dir_mul48 $^

run-mul48: mul48
	./obj_dir_mul48/Vmul48_kernel

clean:
	rm -rf obj_dir obj_dir_mul48

.PHONY: all run clean
