# Verilog: From Software to Silicon

### A Practical Guide for the Hardware-Curious Engineer

---

## Introduction: Why You're Here

You're building AI inference hardware. You've wrestled with CUDA kernels, optimized tensor operations, and probably cursed at PCIe bottlenecks. Now you're staring at FPGAs and ASICs thinking: "How hard could this be?"

Very different, but learnable. This book assumes you know software deeply but hardware not at all. We'll build that bridge.

**What this book is:** A practical, no-BS introduction to Verilog for someone who needs to actually build things.

**What this book isn't:** A comprehensive reference. Once you finish this, grab the IEEE 1364 spec and Sutherland's "Verilog and SystemVerilog Gotchas."

---

## Chapter 1: The Mental Model Shift

### 1.1 Everything Happens in Parallel (Until It Doesn't)

In software, you write:

```python
a = 5
b = 10
c = a + b
```

This happens sequentially. One instruction, then the next.

In hardware, you write:

```verilog
assign c = a + b;
```

This describes **physical wires and gates**. When `a` or `b` changes, `c` changes **automatically** within nanoseconds. There's no "then." Everything just... is.

### 1.2 You're Drawing Circuits, Not Writing Programs

Stop thinking about execution flow. Start thinking about:

- **Wires** (connections between things)
- **Gates** (combinational logic: AND, OR, NOT, etc.)
- **Registers** (memory elements that hold state)
- **Clocks** (the heartbeat that coordinates state changes)

When you write Verilog, you're describing a circuit topology. The synthesis tool converts your description into actual silicon layout.

### 1.3 The Two Domains: Combinational vs. Sequential

**Combinational logic:** Output depends only on current inputs. Like a pure function in FP.

- Examples: Adders, multiplexers, decoders, ALUs

**Sequential logic:** Output depends on current inputs AND past state. Like stateful computation.

- Examples: Counters, shift registers, FSMs, memories

Everything interesting in hardware uses both.

---

## Chapter 2: Basic Syntax and Modules

### 2.1 Your First Module

```verilog
module adder(
    input wire [7:0] a,    // 8-bit input
    input wire [7:0] b,    // 8-bit input
    output wire [7:0] sum  // 8-bit output
);

    assign sum = a + b;

endmodule
```

Breaking this down:

- `module adder(...)` - Module name and port list
- `input wire [7:0] a` - 8-bit input signal (MSB=7, LSB=0)
- `assign` - Continuous assignment for combinational logic
- `endmodule` - End of module definition

### 2.2 Data Types: wire vs. reg

**wire:** For combinational logic. Think "actual physical wire."

```verilog
wire [15:0] result;
assign result = x + y;  // Continuous assignment
```

**reg:** Can hold values between clock edges. Despite the name, doesn't always synthesize to a register. Think "variable that can be assigned in procedural blocks."

```verilog
reg [15:0] counter;
always @(posedge clk) begin
    counter <= counter + 1;  // Sequential assignment
end
```

### 2.3 Bit Vectors and Literals

```verilog
wire [7:0]  byte_val;       // 8 bits
wire [31:0] word_val;       // 32 bits
wire [0:7]  reversed;       // MSB=0, LSB=7 (avoid this)

// Literals
8'b10101010     // 8-bit binary: 0xAA
8'h5A           // 8-bit hex: 90 decimal
8'd90           // 8-bit decimal
4'b1010         // 4-bit binary
'b1             // 1-bit
32'hDEAD_BEEF   // Underscores for readability
```

### 2.4 Operators

Arithmetic: `+`, `-`, `*`, `/`, `%`
Bitwise: `&`, `|`, `^`, `~`
Logical: `&&`, `||`, `!`
Relational: `==`, `!=`, `<`, `>`, `<=`, `>=`
Shift: `<<`, `>>`, `<<<`, `>>>`
Concatenation: `{a, b, c}`
Replication: `{4{1'b1}}` = `4'b1111`

---

## Chapter 3: Combinational Logic

### 3.1 Continuous Assignment

```verilog
module mux2to1(
    input wire sel,
    input wire [7:0] in0,
    input wire [7:0] in1,
    output wire [7:0] out
);

    // Ternary operator
    assign out = sel ? in1 : in0;

endmodule
```

### 3.2 Always Blocks for Combinational Logic

```verilog
module alu(
    input wire [7:0] a, b,
    input wire [1:0] op,
    output reg [7:0] result
);

    always @(*) begin  // @(*) = sensitive to all inputs
        case (op)
            2'b00: result = a + b;
            2'b01: result = a - b;
            2'b10: result = a & b;
            2'b11: result = a | b;
        endcase
    end

endmodule
```

**Critical:** For combinational logic in `always` blocks:

- Use `always @(*)`
- Assign to `reg` (not `wire`)
- Cover all cases or use `default`
- Use blocking assignment `=`

### 3.3 Priority Encoder Example

```verilog
module priority_encoder(
    input wire [7:0] req,
    output reg [2:0] grant,
    output reg valid
);

    always @(*) begin
        valid = 1'b0;
        grant = 3'b000;

        if (req[7]) begin
            grant = 3'd7;
            valid = 1'b1;
        end else if (req[6]) begin
            grant = 3'd6;
            valid = 1'b1;
        end else if (req[5]) begin
            grant = 3'd5;
            valid = 1'b1;
        end
        // ... continue for all bits
    end

endmodule
```

### 3.4 Common Gotcha: Incomplete Sensitivity Lists

**Bad:**

```verilog
always @(a) begin  // Only sensitive to 'a'
    result = a + b;  // But uses 'b' too!
end
```

This creates a latch in synthesis. Use `always @(*)` instead.

---

## Chapter 4: Sequential Logic and Clocking

### 4.1 The Clock

Everything sequential happens on clock edges. Think of the clock as your event loop.

```verilog
module dff(
    input wire clk,
    input wire d,
    output reg q
);

    always @(posedge clk) begin
        q <= d;  // Non-blocking assignment
    end

endmodule
```

`posedge clk` - triggers on rising edge (0→1)
`negedge clk` - triggers on falling edge (1→0)

### 4.2 Blocking vs. Non-Blocking Assignment

**Blocking (`=`):** Execute sequentially within the block. Use for combinational logic.

```verilog
always @(*) begin
    temp = a + b;
    result = temp * c;  // Uses updated 'temp'
end
```

**Non-blocking (`<=`):** All RHS evaluated first, then all assignments happen simultaneously. Use for sequential logic.

```verilog
always @(posedge clk) begin
    q1 <= d;
    q2 <= q1;  // Uses OLD value of q1 (creates shift register)
end
```

**Golden rule:**

- Combinational: `always @(*)` with `=`
- Sequential: `always @(posedge clk)` with `<=`

### 4.3 Reset Signals

**Synchronous reset:**

```verilog
always @(posedge clk) begin
    if (reset) begin
        counter <= 0;
    end else begin
        counter <= counter + 1;
    end
end
```

**Asynchronous reset:**

```verilog
always @(posedge clk or posedge reset) begin
    if (reset) begin
        counter <= 0;
    end else begin
        counter <= counter + 1;
    end
end
```

Async reset triggers immediately. Sync reset waits for clock edge. Async is faster for initialization, but can cause timing issues. Modern designs often use sync reset.

### 4.4 Counter Example

```verilog
module counter #(
    parameter WIDTH = 8
)(
    input wire clk,
    input wire reset,
    input wire enable,
    output reg [WIDTH-1:0] count
);

    always @(posedge clk) begin
        if (reset) begin
            count <= 0;
        end else if (enable) begin
            count <= count + 1;
        end
    end

endmodule
```

---

## Chapter 5: Finite State Machines

FSMs are your bread and butter for control logic. Three styles: Moore, Mealy, and the practical hybrid.

### 5.1 Traffic Light Controller

```verilog
module traffic_light(
    input wire clk,
    input wire reset,
    output reg [1:0] light  // 2'b00=Red, 2'b01=Yellow, 2'b10=Green
);

    // State encoding
    localparam RED    = 2'b00;
    localparam YELLOW = 2'b01;
    localparam GREEN  = 2'b10;

    reg [1:0] state, next_state;
    reg [7:0] timer;

    // State register
    always @(posedge clk) begin
        if (reset) begin
            state <= RED;
            timer <= 0;
        end else begin
            state <= next_state;
            if (state != next_state)
                timer <= 0;
            else
                timer <= timer + 1;
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            RED: begin
                if (timer >= 30)
                    next_state = GREEN;
            end
            GREEN: begin
                if (timer >= 40)
                    next_state = YELLOW;
            end
            YELLOW: begin
                if (timer >= 5)
                    next_state = RED;
            end
        endcase
    end

    // Output logic
    always @(*) begin
        light = state;
    end

endmodule
```

### 5.2 FSM Best Practices

1. **Separate state register from next-state logic:** Makes code clearer and synthesis better
2. **Use localparam for state encoding:** Readable and maintainable
3. **Handle all states:** Avoid latches
4. **Consider one-hot encoding:** For large FSMs, one flip-flop per state can be faster

---

## Chapter 6: Memory and Storage

### 6.1 Register Files

```verilog
module regfile(
    input wire clk,
    input wire we,              // Write enable
    input wire [4:0] waddr,     // Write address
    input wire [31:0] wdata,    // Write data
    input wire [4:0] raddr1,    // Read address 1
    input wire [4:0] raddr2,    // Read address 2
    output wire [31:0] rdata1,  // Read data 1
    output wire [31:0] rdata2   // Read data 2
);

    reg [31:0] regs [0:31];  // 32 registers of 32 bits each

    // Write
    always @(posedge clk) begin
        if (we)
            regs[waddr] <= wdata;
    end

    // Read (combinational)
    assign rdata1 = regs[raddr1];
    assign rdata2 = regs[raddr2];

endmodule
```

### 6.2 FIFO Buffer

Critical for interfacing between different clock domains or handling bursty data.

```verilog
module fifo #(
    parameter DEPTH = 8,
    parameter WIDTH = 8
)(
    input wire clk,
    input wire reset,
    input wire wr_en,
    input wire rd_en,
    input wire [WIDTH-1:0] din,
    output reg [WIDTH-1:0] dout,
    output wire full,
    output wire empty
);

    localparam ADDR_WIDTH = $clog2(DEPTH);

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0] wr_ptr, rd_ptr;  // Extra bit for full/empty

    assign full  = (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]) &&
                   (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);
    assign empty = (wr_ptr == rd_ptr);

    // Write
    always @(posedge clk) begin
        if (reset) begin
            wr_ptr <= 0;
        end else if (wr_en && !full) begin
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= din;
            wr_ptr <= wr_ptr + 1;
        end
    end

    // Read
    always @(posedge clk) begin
        if (reset) begin
            rd_ptr <= 0;
        end else if (rd_en && !empty) begin
            dout <= mem[rd_ptr[ADDR_WIDTH-1:0]];
            rd_ptr <= rd_ptr + 1;
        end
    end

endmodule
```

---

## Chapter 7: Testbenches and Simulation

Hardware bugs are expensive. Simulate everything before synthesis.

### 7.1 Basic Testbench Structure

```verilog
`timescale 1ns/1ps

module counter_tb;

    reg clk;
    reg reset;
    reg enable;
    wire [7:0] count;

    // Instantiate DUT (Device Under Test)
    counter dut (
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .count(count)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns period = 100MHz
    end

    // Test stimulus
    initial begin
        // Initialize
        reset = 1;
        enable = 0;
        #20;

        // Release reset
        reset = 0;
        #10;

        // Enable counter
        enable = 1;
        #100;

        // Disable
        enable = 0;
        #50;

        // Reset again
        reset = 1;
        #10;
        reset = 0;

        #100;
        $finish;
    end

    // Monitor outputs
    initial begin
        $monitor("Time=%0t reset=%b enable=%b count=%d",
                 $time, reset, enable, count);
    end

    // Dump waveforms
    initial begin
        $dumpfile("counter.vcd");
        $dumpvars(0, counter_tb);
    end

endmodule
```

### 7.2 Running Simulations

With Icarus Verilog (open-source):

```bash
iverilog -o counter_tb counter.v counter_tb.v
vvp counter_tb
gtkwave counter.vcd  # View waveforms
```

With ModelSim/QuestaSim:

```bash
vlog counter.v counter_tb.v
vsim counter_tb
run -all
```

### 7.3 Self-Checking Testbenches

```verilog
initial begin
    // Test 1: Reset
    reset = 1;
    #20;
    reset = 0;
    #10;

    if (count != 0) begin
        $display("ERROR: Counter not reset!");
        $finish;
    end

    // Test 2: Counting
    enable = 1;
    repeat (10) @(posedge clk);

    if (count != 10) begin
        $display("ERROR: Expected count=10, got %d", count);
        $finish;
    end

    $display("All tests passed!");
    $finish;
end
```

---

## Chapter 8: Real-World Patterns

### 8.1 Handshake Protocols (Ready/Valid)

Standard way to transfer data between modules.

```verilog
module handshake_producer(
    input wire clk,
    input wire reset,
    input wire ready,           // Downstream is ready
    output reg valid,           // I have valid data
    output reg [31:0] data
);

    reg [31:0] counter;

    always @(posedge clk) begin
        if (reset) begin
            valid <= 0;
            counter <= 0;
        end else begin
            if (valid && ready) begin
                // Transaction completed
                counter <= counter + 1;
                data <= counter;
            end else if (!valid) begin
                // New data available
                valid <= 1;
                data <= counter;
            end
        end
    end

endmodule
```

```verilog
module handshake_consumer(
    input wire clk,
    input wire reset,
    input wire valid,           // Upstream has valid data
    input wire [31:0] data,
    output reg ready            // I'm ready to accept
);

    always @(posedge clk) begin
        if (reset) begin
            ready <= 1;
        end else begin
            if (valid && ready) begin
                // Process data
                $display("Received: %d", data);
                ready <= 1;  // Ready for next
            end
        end
    end

endmodule
```

### 8.2 Pipeline Stages

For high-performance compute, pipeline everything.

```verilog
module mult_pipeline(
    input wire clk,
    input wire [15:0] a, b,
    output reg [31:0] result
);

    // Stage 1: Input registers
    reg [15:0] a_r1, b_r1;
    always @(posedge clk) begin
        a_r1 <= a;
        b_r1 <= b;
    end

    // Stage 2: Multiply
    reg [31:0] mult_r2;
    always @(posedge clk) begin
        mult_r2 <= a_r1 * b_r1;
    end

    // Stage 3: Output
    always @(posedge clk) begin
        result <= mult_r2;
    end

endmodule
```

### 8.3 Clock Domain Crossing

**Never directly connect signals between different clock domains.** Use synchronizers.

```verilog
module synchronizer #(
    parameter STAGES = 2
)(
    input wire clk,
    input wire async_in,
    output wire sync_out
);

    reg [STAGES-1:0] sync_chain;

    always @(posedge clk) begin
        sync_chain <= {sync_chain[STAGES-2:0], async_in};
    end

    assign sync_out = sync_chain[STAGES-1];

endmodule
```

For multi-bit data, use async FIFOs with gray-coded pointers.

---

## Chapter 9: Synthesis and Timing

### 9.1 What Happens During Synthesis

Your Verilog → Logic optimization → Technology mapping → Place and route → Bitstream

Key points:

- Not all Verilog is synthesizable (delays, initial blocks, etc.)
- Synthesis tools infer structures (RAMs, multipliers, DSPs)
- Timing constraints guide optimization

### 9.2 Critical Path and Timing

**Setup time:** How long before clock edge data must be stable
**Hold time:** How long after clock edge data must stay stable
**Critical path:** Longest combinational path between registers

To meet timing at frequency f:

```
T_clk > T_logic + T_setup + T_clk-to-q + T_routing
```

Where:

- T_logic: Combinational delay
- T_setup: Flip-flop setup time
- T_clk-to-q: Flip-flop output delay
- T_routing: Wire delay

### 9.3 Synthesis Directives

```verilog
// Prevent optimization
(* keep *) wire intermediate_signal;

// Force specific implementation
(* use_dsp = "yes" *) reg [31:0] product;

// Mark false paths
(* false_path *) reg async_signal;
```

### 9.4 Resource Inference

```verilog
// This infers BRAM
reg [31:0] memory [0:1023];

// This infers shift register
reg [7:0] shift_reg [0:15];

// This infers DSP block
wire [31:0] mult = a * b + c;
```

---

## Chapter 10: FPGA-Specific Considerations

### 10.1 FPGA Architecture Basics

FPGAs contain:

- **LUTs (Look-Up Tables):** Implement any 4-6 input logic function
- **Flip-flops:** Registers
- **Block RAMs:** Efficient memory blocks
- **DSP blocks:** Dedicated multipliers and MACs
- **I/O blocks:** Interface to external world
- **Clock management:** PLLs, clock buffers

### 10.2 Using Block RAMs Efficiently

```verilog
module bram_example(
    input wire clk,
    input wire [9:0] addr,
    input wire [31:0] din,
    input wire we,
    output reg [31:0] dout
);

    (* ram_style = "block" *)  // Force BRAM inference
    reg [31:0] mem [0:1023];

    always @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];  // Note: 1 cycle latency
    end

endmodule
```

### 10.3 DSP Block Usage

Modern FPGAs have dedicated DSP slices for multiply-accumulate:

```verilog
module mac_unit(
    input wire clk,
    input wire [17:0] a, b,
    input wire [47:0] c,
    output reg [47:0] p
);

    always @(posedge clk) begin
        p <= (a * b) + c;  // Infers DSP48E1 on Xilinx
    end

endmodule
```

---

## Chapter 11: For Your AI Inference Hardware

### 11.1 Matrix Multiply Unit

Core operation for neural networks.

```verilog
module systolic_pe(
    input wire clk,
    input wire reset,
    input wire [15:0] a_in,
    input wire [15:0] b_in,
    input wire [31:0] c_in,
    output reg [15:0] a_out,
    output reg [15:0] b_out,
    output reg [31:0] c_out
);

    always @(posedge clk) begin
        if (reset) begin
            a_out <= 0;
            b_out <= 0;
            c_out <= 0;
        end else begin
            a_out <= a_in;      // Pass through
            b_out <= b_in;      // Pass through
            c_out <= c_in + (a_in * b_in);  // MAC
        end
    end

endmodule
```

Wire these into a grid for systolic array architecture.

### 11.2 Memory Subsystem

```verilog
module weight_buffer #(
    parameter WIDTH = 16,
    parameter DEPTH = 1024
)(
    input wire clk,
    input wire [WIDTH-1:0] din,
    input wire [$clog2(DEPTH)-1:0] waddr,
    input wire [$clog2(DEPTH)-1:0] raddr,
    input wire we,
    output wire [WIDTH-1:0] dout
);

    (* ram_style = "block" *)
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= din;
    end

    assign dout = mem[raddr];

endmodule
```

### 11.3 Quantization Support

For INT8 inference:

```verilog
module quantized_mac(
    input wire clk,
    input wire signed [7:0] activation,
    input wire signed [7:0] weight,
    input wire signed [31:0] bias,
    output reg signed [31:0] result
);

    wire signed [15:0] product;
    assign product = activation * weight;

    always @(posedge clk) begin
        result <= bias + product;
    end

endmodule
```

### 11.4 Data Movement Engine

```verilog
module dma_controller(
    input wire clk,
    input wire reset,
    input wire start,
    input wire [31:0] src_addr,
    input wire [31:0] dst_addr,
    input wire [15:0] length,
    output reg done,
    // AXI-like interface
    output reg [31:0] ar_addr,
    output reg ar_valid,
    input wire ar_ready,
    // ... (simplified)
);

    typedef enum {IDLE, READ_REQ, READ_DATA, WRITE_REQ, WRITE_DATA, DONE} state_t;
    state_t state;

    reg [15:0] count;

    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            done <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        count <= 0;
                        ar_addr <= src_addr;
                        state <= READ_REQ;
                    end
                end

                READ_REQ: begin
                    ar_valid <= 1;
                    if (ar_ready) begin
                        ar_valid <= 0;
                        state <= READ_DATA;
                    end
                end

                // ... implement full FSM

                DONE: begin
                    done <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
```

---

## Chapter 12: Debugging and Verification

### 12.1 Common Synthesis Warnings

**Inferred latches:** You forgot a case in combinational logic

```verilog
// Bad
always @(*) begin
    if (sel == 2'b00)
        out = in0;
    // Missing other cases!
end

// Good
always @(*) begin
    case (sel)
        2'b00: out = in0;
        2'b01: out = in1;
        default: out = 0;
    endcase
end
```

**Multi-driven nets:** Multiple sources driving same wire

```verilog
// Bad
assign out = a;
assign out = b;  // Can't do this!

// Good
assign out = sel ? b : a;
```

### 12.2 Formal Verification Hooks

```verilog
`ifdef FORMAL
    // Assertions
    always @(posedge clk) begin
        if (!reset && enable) begin
            assert (count <= MAX_COUNT);
        end
    end

    // Assumptions
    assume property (@(posedge clk) $stable(config));

    // Coverage
    cover property (@(posedge clk) count == MAX_COUNT);
`endif
```

### 12.3 ChipScope / SignalTap

For debugging on actual hardware:

```verilog
(* mark_debug = "true" *) wire [31:0] debug_signal;
```

Then use Xilinx Vivado ILA or Intel SignalTap to probe signals in real-time.

---

## Chapter 13: SystemVerilog Upgrade Path

Once you master Verilog, SystemVerilog adds:

**Interfaces:** Bundle related signals

```systemverilog
interface axi_if;
    logic [31:0] addr;
    logic [31:0] data;
    logic valid;
    logic ready;
endinterface
```

**Classes:** For testbenches (not synthesizable)

```systemverilog
class Transaction;
    rand bit [31:0] data;
    rand bit [7:0] addr;
endclass
```

**Assertions:** Formal properties

```systemverilog
property req_ack;
    @(posedge clk) req |-> ##[1:5] ack;
endproperty
```

**Better types:** `logic` (replaces wire/reg), enums, structs, unions

---

## Appendix A: Toolchain Setup

### Open Source (Free)

```bash
# Icarus Verilog (simulator)
sudo apt install iverilog

# Yosys (synthesis)
sudo apt install yosys

# GTKWave (waveform viewer)
sudo apt install gtkwave

# For FPGAs
# Xilinx Vivado (free webpack edition)
# Intel Quartus (free lite edition)
```

### Simulation Workflow

```bash
# 1. Compile
iverilog -o sim design.v testbench.v

# 2. Run
vvp sim

# 3. View waves
gtkwave dump.vcd
```

### FPGA Workflow

```bash
# Xilinx
vivado -mode batch -source build.tcl

# Intel
quartus_sh --flow compile project_name
```

---

## Appendix B: Quick Reference

### Module Template

```verilog
module name #(
    parameter WIDTH = 8
)(
    input wire clk,
    input wire reset,
    input wire [WIDTH-1:0] din,
    output reg [WIDTH-1:0] dout
);

    // Combinational logic
    wire [WIDTH-1:0] internal;
    assign internal = din + 1;

    // Sequential logic
    always @(posedge clk) begin
        if (reset)
            dout <= 0;
        else
            dout <= internal;
    end

endmodule
```

### FSM Template

```verilog
// State encoding
localparam S0 = 2'b00;
localparam S1 = 2'b01;

reg [1:0] state, next_state;

// State register
always @(posedge clk)
    if (reset)
        state <= S0;
    else
        state <= next_state;

// Next state logic
always @(*)
    case (state)
        S0: next_state = condition ? S1 : S0;
        S1: next_state = S0;
        default: next_state = S0;
    endcase

// Output logic
always @(*)
    output_signal = (state == S1);
```

---

## Appendix C: Books and Resources

**After this:**

1. "SystemVerilog for Verification" - Chris Spear
2. "Digital Design and Computer Architecture" - Harris & Harris
3. "FPGA Prototyping by Verilog Examples" - Pong P. Chu
4. "Writing Testbenches" - Janick Bergeron

**Online:**

- asic-world.com/verilog - Great tutorials
- fpga4fun.com - Practical projects
- zipcpu.com - Advanced techniques
- IEEE 1364-2005 standard (the actual spec)

**For AI Hardware:**

- "Computer Architecture: A Quantitative Approach" - Hennessy & Patterson
- Google TPU papers
- Cerebras architecture papers

---

## Final Thoughts

You now know enough Verilog to be dangerous. The next step is practice:

1. **Build a simple CPU:** Start with a 8-bit accumulator machine, then RISC-V
2. **Design your MAC array:** Systolic array for matmul
3. **Add memory hierarchy:** Caches, scratchpads, DMA
4. **Optimize:** Pipeline everything, fix timing violations
5. **Test on FPGA:** Get an Arty A7 or DE10-Nano board

Verilog is weird coming from software. You'll make mistakes. Your first design will have timing violations. Your second will have race conditions. By your third, you'll start thinking in hardware.

The metalayer is different here. In software, you abstract to manage complexity. In hardware, you fight the abstraction to get performance. Every flip-flop costs power. Every clock cycle costs time. Every gate costs area. You're optimizing a different cost function now.
