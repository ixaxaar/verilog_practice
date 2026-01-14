module ripple_carry #(
    parameter DATA_WIDTH = 128 // Width of the adder
) (
    input wire [DATA_WIDTH-1:0] a,      // First input operand
    input wire [DATA_WIDTH-1:0] b,      // Second input operand
    input wire carry_in,                // Input carry
    output wire [DATA_WIDTH-1:0] result, // Sum output
    output wire carry_out               // Output carry
);

    genvar i;
    wire [DATA_WIDTH:0] carry;

    // Connect input/output to carry chain
    assign carry[0] = carry_in;
    assign carry_out = carry[DATA_WIDTH];

    generate
        for (i = 0; i < DATA_WIDTH; i = i + 1) begin : fa_chain
            full_adder fa (
                .a(a[i]),
                .b(b[i]),
                .cin(carry[i]),
                .sum(result[i]),
                .cout(carry[i+1])
            );
        end
    endgenerate

endmodule


module full_adder (
    input wire a,
    input wire b,
    input wire cin,
    output wire sum,
    output wire cout
);

    assign sum = a ^ b ^ cin;
    assign cout = (a & b) | (cin & (a ^ b));

endmodule
