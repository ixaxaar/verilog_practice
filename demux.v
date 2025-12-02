module demux_reg #(
    parameter SEL_WIDTH = 2,
    parameter DATA_WIDTH = 8
)(
    input wire clk,
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH-1:0] in,                         // Single input
    output reg [DATA_WIDTH*(2**SEL_WIDTH)-1:0] outputs      // Multiple outputs (packed)
);

    // Route input to selected output, others get 0
    always @(posedge clk) begin
        outputs <= 0;
        outputs[DATA_WIDTH*sel +: DATA_WIDTH] <= in;
    end
endmodule

module demux_wire #(
    parameter SEL_WIDTH = 2,
    parameter DATA_WIDTH = 8
)(
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH-1:0] in,                         // Single input
    output wire [DATA_WIDTH*(2**SEL_WIDTH)-1:0] outputs      // Multiple outputs (packed)
);

    reg [DATA_WIDTH*(2**SEL_WIDTH)-1:0] result_reg;

    // Route input to selected output, others get 0
    always @(*) begin
        result_reg = 0;
        result_reg[DATA_WIDTH*sel +: DATA_WIDTH] = in;
    end

    assign outputs = result_reg;
endmodule
