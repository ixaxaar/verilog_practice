module alu_reg #(
    parameter DATA_WIDTH = 8  // Width of inputs (2^n bits)
)(
    input wire clk,                          // Clock signal
    input wire [DATA_WIDTH-1:0] a,           // N-bit input
    input wire [DATA_WIDTH-1:0] b,           // N-bit input
    input wire [1:0] op,                     // 2-bit operation selector
    output reg [2*DATA_WIDTH-1:0] result     // 2N-bit output
);

    always @(posedge clk) begin
        case (op)
            2'b00: result <= a + b;   // Addition
            2'b01: result <= a - b;   // Subtraction
            2'b10: result <= a * b;   // Multiplication
            2'b11: result <= a / b;   // Division
            default: result <= {2*DATA_WIDTH{1'b0}};
        endcase
    end

endmodule

module alu_wire #(
    parameter DATA_WIDTH = 8  // Width of inputs (2^n bits)
)(
    input wire clk,                          // Clock signal
    input wire [DATA_WIDTH-1:0] a,           // N-bit input
    input wire [DATA_WIDTH-1:0] b,           // N-bit input
    input wire [1:0] op,                     // 2-bit operation selector
    output wire [2*DATA_WIDTH-1:0] result    // 2N-bit wire output
);

    reg [2*DATA_WIDTH-1:0] result_reg;

    always @(posedge clk) begin
        case (op)
            2'b00: result_reg <= a + b;   // Addition
            2'b01: result_reg <= a - b;   // Subtraction
            2'b10: result_reg <= a * b;   // Multiplication
            2'b11: result_reg <= a / b;   // Division
            default: result_reg <= {2*DATA_WIDTH{1'b0}};
        endcase
    end

    assign result = result_reg;

endmodule
