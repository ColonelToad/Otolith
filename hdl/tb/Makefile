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

clean:
	rm -rf obj_dir

.PHONY: all run clean
