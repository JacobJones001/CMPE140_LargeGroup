module mips_pipelined (
        input  wire        clk,
        input  wire        rst,
        input  wire [4:0]  ra3,
        input  wire [31:0] instr,
        input  wire [31:0] rd_dm,
        output wire        we_dm,
        output wire [31:0] pc_current,
        output wire [31:0] alu_out,
        output wire [31:0] wd_dm,
        output wire [31:0] rd3,
        output wire [31:0] wd_rf,
        output wire [4:0] rf_wa
    );

    // F2D edit
    wire [31:0] instr_D;
    
    wire       jal;
    wire       branch;
    wire       jump;
    wire       reg_jump;
    wire       reg_dst;
    wire       we_reg;
    wire       alu_src;
    wire       dm2reg;
    wire       we_hilo;
    wire       hilo_sel;
    wire       alu_out_sel;
    wire       shift_mux_sel;
    wire       we_dm_D;
    wire [2:0] alu_ctrl;

    datapath_pipelined dp (
            .clk            (clk),
            .rst            (rst),
            .jal            (jal),
            .branch         (branch),
            .jump           (jump),
            .reg_jump       (reg_jump),
            .reg_dst        (reg_dst),
            .we_reg         (we_reg),
            .alu_src        (alu_src),
            .dm2reg         (dm2reg),
            .hilo_sel       (hilo_sel),
            .alu_out_sel    (alu_out_sel),
            .we_hilo        (we_hilo),
            .shift_mux_sel  (shift_mux_sel),
            .alu_ctrl       (alu_ctrl),
            .ra3            (ra3),
            .instr          (instr),
            .rd_dm          (rd_dm),
            .pc_current     (pc_current),
            .alu_mux_out    (alu_out),
            .wd_dm          (wd_dm),
            .rd3            (rd3),
            .wd_rf          (wd_rf),
            .rf_wa          (rf_wa),
            .instr_D        (instr_D),

            .we_dm_D(we_dm_D),
            .we_dm_M(we_dm)
        );

    controlunit cu (
            .opcode         (instr_D[31:26]),
            .funct          (instr_D[5:0]),
            .jal            (jal),
            .branch         (branch),
            .jump           (jump),
            .reg_jump       (reg_jump),
            .reg_dst        (reg_dst),
            .we_reg         (we_reg),
            .alu_src        (alu_src),
            .we_dm          (we_dm_D),
            .dm2reg         (dm2reg),
            .hilo_sel       (hilo_sel),
            .alu_out_sel    (alu_out_sel),
            .we_hilo        (we_hilo),
            .shift_mux_sel  (shift_mux_sel),
            .alu_ctrl       (alu_ctrl)
        );

endmodule