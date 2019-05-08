module dreg_pipelined # (parameter WIDTH = 32) (
        input  wire             clk,
        input  wire             rst,
        input  wire [WIDTH-1:0] d,
        input  wire stall,
        output reg  [WIDTH-1:0] q
    );

    always @ (posedge clk, posedge rst) begin
        if (rst) q <= 0;
        else if (stall) q <= q;
        else     q <= d;
    end
endmodule