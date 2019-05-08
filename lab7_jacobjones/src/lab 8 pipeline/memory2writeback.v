module memory2writeback (
    input wire stall_m2wb,

	input wire alu_out_sel_M,
	input wire jal_M,
	input wire reg_jump_M,
	input wire jump_M,
	input wire dm2reg_M,
	input wire pc_src,
    input wire we_reg_M,
	input wire [31:0] rd_dm,
	input wire [31:0] hilo_mux_out,
    input wire [31:0] alu_out_M,
    input wire [4:0]  rf_wa_M,
    input wire [31:0] pc_plus4_M, 
	input wire rst, clk,

	output reg alu_out_sel_WB,
	output reg jal_WB,
	output reg reg_jump_WB,
	output reg jump_WB,
	output reg dm2reg_WB,
	output reg pc_src_WB,
    output reg we_reg_WB,
	output reg [31:0] rd_dm_WB,
	output reg [31:0] hilo_mux_out_WB,
    output reg [31:0] alu_out_WB,
    output reg [4:0]  rf_wa_WB,
    output reg [31:0] pc_plus4_WB
);

always @(posedge clk, posedge rst) 
    begin
        if(rst)
            begin
                alu_out_sel_WB 	<= 0;
                jal_WB 			<= 0;
                reg_jump_WB 		<= 0;
                jump_WB 			<= 0;
                dm2reg_WB 		<= 0;
                pc_src_WB 			<= 0;
                we_reg_WB           <= 0;
                rd_dm_WB 			<= 0;
                hilo_mux_out_WB       <=0;
                alu_out_WB <= 0;
                rf_wa_WB <=0;
                pc_plus4_WB <=0;
            end
        else if(stall_m2wb)
            begin
                alu_out_sel_WB  <= 	alu_out_sel_WB; 
                jal_WB          <=	jal_WB;
                reg_jump_WB	  	<=	reg_jump_WB; 
                jump_WB 	  	<=	jump_WB;
                dm2reg_WB	  	<=	dm2reg_WB;
                pc_src_WB       <=	pc_src_WB;
                we_reg_WB       <=  we_reg_WB;
                rd_dm_WB 		<= 	rd_dm_WB;
                hilo_mux_out_WB  <= hilo_mux_out_WB;
                alu_out_WB <= alu_out_WB;
                rf_wa_WB <= rf_wa_WB;
                pc_plus4_WB <= pc_plus4_WB;
            end
        else
            begin
                alu_out_sel_WB  <= 	alu_out_sel_M; 
                jal_WB          <=	jal_M;
                reg_jump_WB	  	<=	reg_jump_M; 
                jump_WB 	  	<=	jump_M;
                dm2reg_WB	  	<=	dm2reg_M;
                pc_src_WB       <=	pc_src;
                we_reg_WB       <=  we_reg_M;
                rd_dm_WB 		<= 	rd_dm;
                hilo_mux_out_WB  <= hilo_mux_out;
                alu_out_WB <= alu_out_M;
                rf_wa_WB <= rf_wa_M;
                pc_plus4_WB <= pc_plus4_M;
            end   
    end
endmodule