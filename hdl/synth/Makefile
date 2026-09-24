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

clean:
	rm -rf build

.PHONY: all synth pnr report clean
