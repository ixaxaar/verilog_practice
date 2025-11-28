module mux_reg #(
    parameter SEL_WIDTH = 2,           // Number of select bits
    parameter DATA_WIDTH = 8           // Width of each input/output
)(
    input wire [SEL_WIDTH-1:0] sel,
    input wire [DATA_WIDTH*(2**SEL_WIDTH)-1:0] inputs,  // Packed array of all inputs
    output reg [DATA_WIDTH-1:0] out
);

    integer i;
    reg [DATA_WIDTH-1:0] input_array [0:(2**SEL_WIDTH)-1];

    // Unpack the inputs into an array
    always @(posedge clk) begin
        for (i = 0; i < 2**SEL_WIDTH; i = i + 1) begin
            input_array[i] = inputs[DATA_WIDTH*i +: DATA_WIDTH];
        end
        out = input_array[sel];
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

    reg [DATA_WIDTH-1:0] out_reg;
    integer i;
    reg [DATA_WIDTH-1:0] input_array [0:(2**SEL_WIDTH)-1];

    // Unpack the inputs into an array and select output
    always @(*) begin
        for (i = 0; i < 2**SEL_WIDTH; i = i + 1) begin
            input_array[i] = inputs[DATA_WIDTH*i +: DATA_WIDTH];
        end
        out_reg = input_array[sel];
    end

    assign out = out_reg;
endmodule
