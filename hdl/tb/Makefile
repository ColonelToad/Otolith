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

# M5-A: Q16.48 primitive parity (clz64 / recip48 / mulq48) before any
# sequencing is layered on. Separate Mdir again.
PRIM_FLAGS = --cc --exe --build -j $(shell nproc) \
	--top-module fixed_pkg_wrap --assert \
	-CFLAGS "-std=c++20 -I$(FUSION)"

prim: obj_dir_prim/Vfixed_pkg_wrap

obj_dir_prim/Vfixed_pkg_wrap: tb_prim.cpp prim_wrap.sv ../rtl/fixed_pkg.sv
	$(VERILATOR) $(PRIM_FLAGS) --Mdir obj_dir_prim $^

run-prim: prim
	./obj_dir_prim/Vfixed_pkg_wrap

# M5-A: ldl_kernel parity vs the C++ model, on REAL captured innovation
# covariances (fuse_update_study --dump-s).
LDL_FLAGS = --cc --exe --build -j $(shell nproc) \
	--top-module ldl_kernel --assert \
	-CFLAGS "-std=c++20 -I$(FUSION)"

ldl: obj_dir_ldl/Vldl_kernel

obj_dir_ldl/Vldl_kernel: tb_ldl.cpp ../rtl/fixed_pkg.sv ../rtl/ldl_kernel.sv
	$(VERILATOR) $(LDL_FLAGS) --Mdir obj_dir_ldl $^

# Regenerate the capture from a log first:
#   PYTHONPATH=sim pixi run python -c "from otolith_sim.logger import \
#     record_puppet_log; record_puppet_log('/tmp/m5long.otlg', duration_s=10.0)"
#   ../../fusion/build/fuse_update_study /tmp/m5long.otlg \
#     --dump-s /tmp/s_cap.txt --dump-w 6
SCAP ?= /tmp/s_cap.txt

run-ldl: ldl
	./obj_dir_ldl/Vldl_kernel $(SCAP)

clean:
	rm -rf obj_dir obj_dir_mul48 obj_dir_prim obj_dir_ldl

.PHONY: all run clean
