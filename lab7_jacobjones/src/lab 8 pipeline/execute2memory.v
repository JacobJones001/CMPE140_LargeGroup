module execute2memory(
	input wire clk, rst,
	input wire zero_E,
	input wire pc_plus4_E,
	input wire [31:0] alu_out_E, wd_dm_E, 
	input wire [63:0] hilo_d_E, 

	output reg 	zero_M,	pc_plus4_M,
	output reg [31:0]	alu_out_M,	wd_dm_M,
	output reg [63:0]	hilo_d_M,
		
	
	//control unit signals
	input wire 
		we_hilo_E, 
		alu_out_sel_E,
		jal_E,
		hilo_sel_E,
		reg_jump_E,
		jump_E,
		dm2reg_E,
		we_dm_E,
		branch_E,
		
	
	output reg 
		we_hilo_M,
		alu_out_sel_M,
		jal_M,
		hilo_sel_M,
		reg_jump_M,
		jump_M,
		dm2reg_M,
		we_dm_M,
		branch_M
		
);
	always @ (posedge clk, rst) 
		begin 
		if (rst)
		begin 
			
			zero_M 		<= 0;
			pc_plus4_M 	<= 0;
			alu_out_M 	<= 0;
			hilo_d_M 	<= 0;
			wd_dm_M 	<= 0;
			
			//CU
			we_hilo_M 		<= 0;
			alu_out_sel_M 	<= 0;
			jal_M 			<= 0;
			hilo_sel_M 		<= 0;
			reg_jump_M 		<= 0;
			jump_M 			<= 0;
			dm2reg_M 		<= 0;
			we_dm_M 		<= 0;
			branch_M 		<= 0;
		end 
			
		else
		begin 
			zero_M 		<= zero_E;
			pc_plus4_M <= pc_plus4_E;
			alu_out_M 	<= alu_out_E;
			hilo_d_M 	<= hilo_d_E;
			wd_dm_M 	<= wd_dm_E;
			
			//CU
			we_hilo_M 		<= we_hilo_E;
			alu_out_sel_M 	<= alu_out_sel_E;
			jal_M 			<= jal_E;
			hilo_sel_M 		<= hilo_sel_E;
			reg_jump_M 		<= reg_jump_E;
			jump_M 			<= jump_E;
			dm2reg_M 		<= dm2reg_E;
			we_dm_M 		<= we_dm_E;
			branch_M 		<= branch_E;
		end
	end 
endmodule 	