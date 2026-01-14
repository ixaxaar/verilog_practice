module barrel_shifter #(
    parameter WIDTH = 8
) (
    input wire [WIDTH-1:0] in,
    input wire [$clog2(WIDTH)-1:0] shift_amt,
    input wire dir, // 0 for left shift, 1 for right shift
    output wire [WIDTH-1:0] out
);

    wire [WIDTH-1:0] left_shifted;
    wire [WIDTH-1:0] right_shifted;

    assign left_shifted = in << shift_amt;
    assign right_shifted = in >> shift_amt;

    assign out = dir ? right_shifted : left_shifted;

endmodule
