module hazard_unit(
    input wire test,
    input wire clk,

    // alu data forwarding
    input wire [4:0] rs1E,
    input wire [4:0] rs2E,
    input wire [4:0] waM,
    input wire we_reg_M,
    input wire [4:0] waWB,
    input wire we_reg_WB,
    output wire [1:0] alu_data_forward_rd1,
    output wire [1:0] alu_data_forward_rd2,

    // lw data forwarding
    input wire [4:0] rs1D,
    input wire [4:0] rs2D,
    input wire [1:0] dm2reg_E,

    // beq data forwarding
    input wire [4:0] waE,
    input wire we_reg_E,
    output wire [1:0] beq_rd1_sel,
    output wire [1:0] beq_rd2_sel,

    // branch/jump stall
    input wire [31:0] instr_F,
    input wire [31:0] instr_D,

    output wire stall_pc,
    output wire stall_f2d,
    output wire stall_d2e,
    output wire stall_e2m,
    output wire stall_m2wb,
    output wire flush_f2d,
    output wire flush_d2e
);
    // reg [4:0] ctrl;    
    // assign {stall_m2wb, stall_e2m, stall_d2e, stall_f2d, stall_pc} = ctrl;

    // wire alu_data_forward;
    wire alu_data_forward_rd1_M;
    wire alu_data_forward_rd2_M;
    wire alu_data_forward_rd1_WB;
    wire alu_data_forward_rd2_WB;
    assign alu_data_forward_rd1_M = ((rs1E != 0) & (rs1E == waM) & we_reg_M);
    assign alu_data_forward_rd2_M = ((rs2E != 0) & (rs2E == waM) & we_reg_M);
    assign alu_data_forward_rd1_WB = ((rs1E != 0) & (rs1E == waWB) & we_reg_WB);
    assign alu_data_forward_rd2_WB = ((rs2E != 0) & (rs2E == waWB) & we_reg_WB);

    assign alu_data_forward_rd1 = {alu_data_forward_rd1_WB, alu_data_forward_rd1_M};
    assign alu_data_forward_rd2 = {alu_data_forward_rd2_WB, alu_data_forward_rd2_M};

    wire beq_rd1_sel_E, beq_rd1_sel_M, beq_rd1_sel_WB;
    wire beq_rd2_sel_E, beq_rd2_sel_M, beq_rd2_sel_WB;
    assign beq_rd1_sel_E = (rs1D != 0) & (rs1D == waE) & we_reg_E;
    assign beq_rd2_sel_E = (rs2D != 0) & (rs2D == waE) & we_reg_E;
    assign beq_rd1_sel_M = (rs1D != 0) & (rs1D == waM) & we_reg_M;
    assign beq_rd2_sel_M = (rs2D != 0) & (rs2D == waM) & we_reg_M;
    assign beq_rd1_sel_WB = (rs1D != 0) & (rs1D == waWB) & we_reg_WB;
    assign beq_rd2_sel_WB = (rs2D != 0) & (rs2D == waWB) & we_reg_WB;
    assign beq_rd1_sel = beq_rd1_sel_WB ? 2'b11 : {beq_rd1_sel_M, beq_rd1_sel_E};
    assign beq_rd2_sel = beq_rd2_sel_WB ? 2'b11 : {beq_rd2_sel_M, beq_rd2_sel_E};

    wire jump_branch_stall;
    reg jump_branch_stall_reg;
    assign jump_branch_stall = //(instr_D == 32'b0) ? 1'b0 :
                (jump_branch_stall_reg) ? 0'b0 :
                (instr_F[31:26] == 6'h4) | // beq
                (instr_F[31:26] == 6'h2) | // j
                (instr_F[31:26] == 6'h3) | // jal
                ((instr_F[31:26] == 6'h0) & (instr_F[5:0] == 6'h8)) ; // jr
                

    always @ (posedge clk) begin
        jump_branch_stall_reg <= jump_branch_stall;
    end
    // assign flush_f2d = jump_branch_stall;
    assign flush_f2d = 0'b0;

    wire lw_stall;
    assign lw_stall = ((rs1D == rs2E) | (rs2D == rs2E)) & dm2reg_E;
    assign stall_pc = lw_stall | jump_branch_stall;
    // assign stall_pc = lw_stall | 0'b0;
    assign stall_f2d = lw_stall;
    assign flush_d2e = lw_stall | jump_branch_stall;
    assign stall_d2e = 1'b0;
    assign stall_e2m = 1'b0;
    assign stall_m2wb = 1'b0;

    // always @ (test) begin
    //     if(test) begin
    //         // ctrl = 'b1_1_1_1_1;
    //         ctrl = 'b0_0_0_0_0;
    //     end
    //     else begin
    //         ctrl = 'b0_0_0_0_0;
    //     end
    // end


endmodule
