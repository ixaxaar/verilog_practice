module encoder_reg #(
    parameter WIDTH = 2           // Number of bits in output
)(
    input wire clk,
    input wire [2**WIDTH-1:0] in,                     // One-hot input
    output reg [WIDTH-1:0] out                   // Encoded output
);

    always @(posedge clk) begin
        out <= $clog2(in);  // Find index of highest set bit
    end
endmodule

module encoder_wire #(
    parameter WIDTH = 2
)(
    input wire [2**WIDTH-1:0] in,                     // One-hot input
    output wire [WIDTH-1:0] out                   // Encoded output
);

    assign out = $clog2(in);  // Find index of highest set bit
endmodule
