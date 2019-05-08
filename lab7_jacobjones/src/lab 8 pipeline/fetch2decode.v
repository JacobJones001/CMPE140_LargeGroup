// `timescale 1ns / 1ps

module fetch2decode(
	input wire stall_f2d,

    input wire clk,
	input wire rst,
	input wire [31:0] instr,
	input wire [31:0] pc_plus4,

    output reg [31:0] instr_D,
    output reg [31:0] pc_plus4_D
);
always @ (posedge clk, posedge rst) begin
if (rst)
begin
    instr_D 	<= 0;
	pc_plus4_D 	<= 0;
end
else if (stall_f2d)
begin
	instr_D 	<= instr_D;
	pc_plus4_D 	<= pc_plus4_D;
end
else
begin
	instr_D 	<= instr;
	pc_plus4_D 	<= pc_plus4;
end
end
endmodule