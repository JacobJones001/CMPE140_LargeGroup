module decode2execute(
	input wire stall_d2e,
	input wire [31:0] instr_D,
	output reg [4:0] rs1E,
	output reg [4:0] rs2E,

	input wire clk,
	input wire rst,
	input wire [31:0] rd1out_D,
	input wire [31:0] wd_dm_D,
	input wire [31:0] sext_imm_D, 
	input wire [31:0] pc_plus4_D,
	input wire [4:0]  rf_wa,

	output reg [31:0] rd1out_E, 
	output reg [31:0] wd_dm_E,
	output reg [31:0] sext_imm_E,
	output reg [31:0] pc_plus4_E,
	output reg [4:0]  rf_wa_E,
	
	//control unit signals
	input wire we_hilo, alu_out_sel, shift_mux_sel, jal, hilo_sel, reg_jump, jump, we_dm, branch, alu_src,
	input wire dm2reg,
	input wire [2:0] alu_ctrl,
	input wire we_reg,
	
	output reg we_hilo_E, alu_out_sel_E, shift_mux_sel_E, jal_E, hilo_sel_E, reg_jump_E, jump_E, we_dm_E, branch_E, alu_src_E,
	output reg dm2reg_E,
	output reg [2:0] alu_ctrl_E,
	output reg we_reg_E
);

always @ (posedge clk, posedge rst) begin
if (rst)
begin
	// Hazard
	rs1E <= 0;
	rs2E <= 0;

	rd1out_E 	<= 0;
	wd_dm_E 	<= 0;
	sext_imm_E 	<= 0;
	pc_plus4_E  <= 0;
	rf_wa_E		<= 0;
	
	//CONTROL UNIT
	we_hilo_E 		<= 0;
	alu_out_sel_E 	<= 0;
	shift_mux_sel_E <= 0;
	jal_E 			<= 0;
	hilo_sel_E 		<= 0;
	reg_jump_E 		<= 0;
	jump_E 			<= 0;
	dm2reg_E 		<= 0;
	we_dm_E 		<= 0;
	branch_E 		<= 0;
	alu_src_E 		<= 0;
	alu_ctrl_E 		<= 0;
	we_reg_E		<= 0;
end
else if(stall_d2e)
begin
	// Hazard
	rs1E <= rs1E;
	rs2E <= rs2E;

	rd1out_E 	<= rd1out_E;
	wd_dm_E 	<= wd_dm_E;
	sext_imm_E 	<= sext_imm_E;
	pc_plus4_E  <= pc_plus4_E;
	rf_wa_E		<= rf_wa_E;
	
	//CONTROL UNIT
	we_hilo_E 		<= we_hilo_E;
	alu_out_sel_E 	<= alu_out_sel_E;
	shift_mux_sel_E <= shift_mux_sel_E;
	jal_E 			<= jal_E;
	hilo_sel_E 		<= hilo_sel_E;
	reg_jump_E 		<= reg_jump_E;
	jump_E 			<= jump_E;
	dm2reg_E 		<= dm2reg_E;
	we_dm_E 		<= we_dm_E;
	branch_E 		<= branch_E;
	alu_src_E 		<= alu_src_E;
	alu_ctrl_E 		<= alu_ctrl_E;
	we_reg_E		<= we_reg_E;
end
else
begin
	// Hazard
	rs1E <= instr_D[25:21];
	rs2E <= instr_D[20:16];

	rd1out_E 	<= rd1out_D;
	wd_dm_E 	<= wd_dm_D;
	sext_imm_E 	<= sext_imm_D;
	pc_plus4_E  <= pc_plus4_D;
	rf_wa_E		<= rf_wa;
	
	//CONTROL UNIT
	we_hilo_E 		<= we_hilo;
	alu_out_sel_E 	<= alu_out_sel;
	shift_mux_sel_E <= shift_mux_sel;
	jal_E 			<= jal;
	hilo_sel_E 		<= hilo_sel;
	reg_jump_E 		<= reg_jump;
	jump_E 			<= jump;
	dm2reg_E 		<= dm2reg;
	we_dm_E 		<= we_dm;
	branch_E 		<= branch;
	alu_src_E 		<= alu_src;
	alu_ctrl_E 		<= alu_ctrl;
	we_reg_E		<= we_reg;
end
end
endmodule