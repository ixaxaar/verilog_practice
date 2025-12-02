module mux_reg #(
    parameter SEL_WIDTH = 2,           // Number of select bits
    parameter DATA_WIDTH = 8           // Width of each input/output
)(
    input wire clk,
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH*(2**SEL_WIDTH)-1:0] inputs,  // Packed array of all inputs
    output reg [DATA_WIDTH-1:0] out
);

    // Directly select the appropriate slice using part-select
    always @(posedge clk) begin
        out <= inputs[DATA_WIDTH*sel +: DATA_WIDTH];
    end
endmodule

module mux_wire #(
    parameter SEL_WIDTH = 2,           // Number of select bits
    parameter DATA_WIDTH = 8           // Width of each input/output
)(
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH*(2**SEL_WIDTH)-1:0] inputs,  // Packed array of all inputs
    output wire [DATA_WIDTH-1:0] out
);

    // Directly select the appropriate slice using part-select
    assign out = inputs[DATA_WIDTH*sel +: DATA_WIDTH];
endmodule
