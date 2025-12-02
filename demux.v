module demux_reg #(
    parameter SEL_WIDTH = 2,
    parameter DATA_WIDTH = 8
)(
    input wire clk,
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH-1:0] in,                         // Single input
    output reg [DATA_WIDTH*(2**SEL_WIDTH)-1:0] out      // Multiple out (packed)
);

    // Route input to selected output, others get 0
    always @(posedge clk) begin
        out <= 0;
        out[DATA_WIDTH*sel +: DATA_WIDTH] <= in;
    end
endmodule

module demux_wire #(
    parameter SEL_WIDTH = 2,
    parameter DATA_WIDTH = 8
)(
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH-1:0] in,                         // Single input
    output wire [DATA_WIDTH*(2**SEL_WIDTH)-1:0] out      // Multiple out (packed)
);

    reg [DATA_WIDTH*(2**SEL_WIDTH)-1:0] result_reg;

    // Route input to selected output, others get 0
    always @(*) begin
        result_reg = 0;
        result_reg[DATA_WIDTH*sel +: DATA_WIDTH] = in;
    end

    assign out = result_reg;
endmodule
