# Verilog Makefile using Icarus Verilog

# Compiler
IVERILOG = iverilog

# Source files
SRCS = alu.v mux.v demux.v decoder.v encoder.v comparator.v

.PHONY: all clean lint

# Default target - syntax check all files
all: lint

# Lint/check syntax only
lint:
	@$(IVERILOG) -t null alu.v
	@$(IVERILOG) -t null mux.v
	@$(IVERILOG) -t null demux.v
	@$(IVERILOG) -t null decoder.v
	@$(IVERILOG) -t null encoder.v
	@$(IVERILOG) -t null comparator.v

# Clean build artifacts
clean:
	rm -f *.vvp *.vcd *.out
