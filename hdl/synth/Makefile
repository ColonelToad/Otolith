# Synthesis + P&R for predict_core (ADR-0007 M3).
# Usage: make [synth|pnr|report|clean]
# Device: ECP5-85F (headroom for the flop-heavy first cut: 3x225x64 P
# arrays + Phi as registers; BRAM-ification is named future work).
# Timing target 50 MHz; the REPORTED Fmax is the honest number.

OSS := $(HOME)/Projects/hardware/oss-cad-suite/bin
YOSYS := $(OSS)/yosys
NEXTPNR := $(OSS)/nextpnr-ecp5
DEVICE := --85k
FREQ := 50

all: report

build:
	mkdir -p build

build/predict.json: predict.ys ../rtl/fixed_pkg.sv ../rtl/predict_core.sv | build
	$(YOSYS) -l build/yosys.log predict.ys

synth: build/predict.json

# P&R without pin constraints (internal clock; no board): allow unconstrained.
build/pnr-report.json: build/predict.json
	cd build && $(NEXTPNR) $(DEVICE) --json predict.json \
		--freq $(FREQ) --lpf-allow-unconstrained \
		--textcfg predict_out.config --report pnr-report.json

pnr: build/pnr-report.json

report: pnr
	@echo '=== Yosys cell stat (tail) ==='
	@grep -A30 'Printing statistics' build/yosys.log | tail -32
	@echo '=== nextpnr utilization + Fmax ==='
	@python3 -c "import json; r=json.load(open('build/pnr-report.json')); \
	 print('util:', r.get('utilization',{})); \
	 print('fmax:', r.get('fmax',{}))" 2>/dev/null || \
	 grep -iE 'util|fmax|freq|LUT|FF|MHz' build/pnr-report.json | head -20

# --- v0.4 M5-A: ldl_kernel on ECP5 (the PPA path that closes; see ldl_ecp5.ys) ---
build/ldl.json: ldl_ecp5.ys ../rtl/fixed_pkg.sv ../rtl/ldl_kernel.sv | build
	$(YOSYS) -l build/ldl-yosys.log ldl_ecp5.ys

ldl-synth: build/ldl.json

build/ldl-report.json: build/ldl.json
	cd build && $(NEXTPNR) $(DEVICE) --json ldl.json \
		--freq $(FREQ) --lpf-allow-unconstrained \
		--textcfg ldl_out.config --report ldl-report.json

ldl-report: build/ldl-report.json
	@grep -A24 'Printing statistics' build/ldl-yosys.log | tail -20
	@python3 -c "import json; r=json.load(open('build/ldl-report.json')); \
	 print('util:', r.get('utilization',{})); print('fmax:', r.get('fmax',{}))"

clean:
	rm -rf build

.PHONY: all synth pnr report ldl-synth ldl-report clean
