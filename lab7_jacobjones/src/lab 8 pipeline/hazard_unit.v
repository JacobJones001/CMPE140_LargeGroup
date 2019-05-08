module hazard_unit(
    input wire test,

    // alu data forwarding
    input wire [4:0] rs1E,
    input wire [4:0] rs2E,
    input wire [4:0] waM,
    input wire we_reg_M,
    input wire [4:0] waWB,
    input wire we_reg_WB,
    output wire [1:0] alu_data_forward_rd1,
    output wire [1:0] alu_data_forward_rd2,

    output wire stall_pc,
    output wire stall_f2d,
    output wire stall_d2e,
    output wire stall_e2m,
    output wire stall_m2wb
);
    reg [4:0] ctrl;    
    assign {stall_m2wb, stall_e2m, stall_d2e, stall_f2d, stall_pc} = ctrl;

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

    always @ (test) begin
        if(test) begin
            // ctrl = 'b1_1_1_1_1;
            ctrl = 'b0_0_0_0_0;
        end
        else begin
            ctrl = 'b0_0_0_0_0;
        end
    end


endmodule
