module comparator_reg #(
    parameter WIDTH = 8           // Width of each input
)(
    input wire clk,
    input wire [WIDTH-1:0] a,
    input wire [WIDTH-1:0] b,
    input wire [1:0] op,                     // 2-bit operation selector
    output reg result
);

    always @(posedge clk) begin
        case(op)
            2'b00: result <= a < b;
            2'b01: result <= a > b;
            2'b10: result <= a == b;
            default: result <= 0;
        endcase
    end
endmodule


module comparator_wire #(
    parameter WIDTH = 8           // Width of each input
)(
    input wire clk,
    input wire [WIDTH-1:0] a,
    input wire [WIDTH-1:0] b,
    input wire [1:0] op,                     // 2-bit operation selector
    output wire result
);

    reg res;

    always @(posedge clk) begin
        case(op)
            2'b00: res <= a < b;
            2'b01: res <= a > b;
            2'b10: res <= a == b;
            default: res <= 0;
        endcase
    end

    assign result = res;
endmodule
