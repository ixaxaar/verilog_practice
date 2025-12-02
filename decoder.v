module decoder_reg #(
    parameter WIDTH = 2           // Number of bits in input
)(
    input wire clk,
    input wire [WIDTH-1:0] in,                     // Input bits
    output reg [2**WIDTH-1:0] out                   // One-hot output
);

    always @(posedge clk) begin
        out <= 1 << in;  // Shift 1 to the left by 'in' positions
    end
endmodule

module decoder_wire #(
    parameter WIDTH = 2
)(
    input wire [WIDTH-1:0] in,                     // Input bits
    output wire [2**WIDTH-1:0] out                   // One-hot output
);

    assign out = 1 << in;  // Shift 1 to the left by 'in' positions
endmodule
