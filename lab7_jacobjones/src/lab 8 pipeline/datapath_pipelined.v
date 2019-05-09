module datapath_pipelined (
        input  wire        clk,
        input  wire        rst,
        input  wire        jal,
        input  wire        branch,      
        input  wire        jump,
        input  wire        reg_jump,
        input  wire        reg_dst,
        input  wire        we_reg,
        input  wire        alu_src,
        input  wire        dm2reg,
        input  wire        we_hilo,
        input  wire        alu_out_sel,
        input  wire        hilo_sel,
        input  wire        shift_mux_sel,
        input  wire [2:0]  alu_ctrl,
        input  wire [4:0]  ra3,
        input  wire [31:0] instr,
        input  wire [31:0] rd_dm,
        output wire [31:0] pc_current,
        output wire [31:0] alu_mux_out,
        output wire [31:0] wd_dm,
        output wire [31:0] rd3,
        output wire [31:0] wd_rf,
        output wire [4:0] rf_wa,
        output wire [31:0] instr_D,

        input wire we_dm_D,
        output wire we_dm_M,

        output wire [31:0] alu_out_M,
        output wire [31:0] wd_dm_M       
    );

        // hazard unit addition
        wire stall_pc, stall_f2d, stall_d2e, stall_e2m, stall_m2wb;
            // DF
        wire [1:0] alu_DF_sel_rd1, alu_DF_sel_rd2;
        wire [4:0] rs1E, rs2E; 
        wire [31:0] alu_DF_rd1_out;    
        wire [31:0] alu_DF_rd2_out;
        wire flush_d2e;

        // lw and sw fix
        wire [31:0] rd1_shift_full_out;
        wire lw_sw_zero_sel_E;

        // F2D edit
        wire [31:0] pc_plus4_D;
        // wire [31:0] instr_D;

        // we_reg edit
        wire we_reg_E, we_reg_M, we_reg_WB;

        //rf edit
        wire [4:0] rf_wa_E, rf_wa_M, rf_wa_WB;

        //clarence edit
        // wire we_dm_D;
         wire zero_E;        
        // wire alu_out_E;      
       wire [63:0] hilo_d_E; 
    
       wire zero_M;        
        wire [31:0] pc_plus4_M;     
        // wire [31:0] alu_out_M;      
        wire [31:0] alu_out_WB;      
        // wire [31:0] wd_dm_M;       
        wire [63:0] hilo_d_M;       
        wire hilo_sel_out;
        //
        wire we_hilo_E;
        wire alu_out_sel_E;
        wire shift_mux_sel_E;
        wire jal_E;
        wire hilo_sel_E;
        wire reg_jump_E;
        wire jump_E;
        wire dm2reg_E;
        wire we_dm_E;
        wire branch_E;
        wire [2:0] alu_ctrl_E;
        wire alu_src_E;

        wire we_hilo_M;
        wire alu_out_sel_M;
        wire jal_M;
        wire hilo_sel_M;
        wire reg_jump_M;
        wire jump_M;
        wire dm2reg_M;
        // wire we_dm_M;
        wire branch_M;

        wire alu_out_sel_WB;
        wire jal_WB;
        wire reg_jump_WB;
        wire jump_WB;
        wire dm2reg_WB;
        wire pc_src_WB;
        wire [31:0] rd_dm_WB;
        wire [31:0] hilo_mux_out_WB;
        wire [31:0] pc_plus4_WB;

    wire [4:0]  reg_addr;
    // wire [4:0]  rf_wa;
    wire        pc_src;
    wire [31:0] pc_plus4;
    wire [31:0] pc_pre_WB;
    wire [31:0] pc_next;
    wire [31:0] pc_rj_plus4;
    // wire [31:0] sext_imm;
    wire [31:0] ba;
    wire [31:0] bta;
    wire [31:0] jta;
    wire [31:0] rd1_out;
    wire [31:0] alu_pa;
    wire [31:0] alu_pb;
    wire [31:0] alu_mem_out;
    // wire [31:0] rd1out_D;
    // wire [31:0] wd_dm_D;
    wire [31:0] sext_imm_D;    
    // wire [31:0] pc_plus4_D;
    wire [31:0] rd1out_E;
    wire [31:0] wd_dm_E;
    wire [31:0] sext_imm_E;    
    wire [31:0] pc_plus4_E;
    // wire [31:0] wd_rf;
    wire        zero;
    wire [64-1:0] hilo_d, hilo_q;
    wire [32-1:0] hi_q, lo_q;
    wire [32-1:0] alu_out_hi;
    wire [32-1:0] hilo_mux_out;
    wire [32-1:0] alu_out;
    // wire [31:0] shift_mul_mux_out;
    wire [4:0] shift_rd1_out;
    
    assign pc_src = branch_E & zero_E;
    assign ba = {sext_imm_E[29:0], 2'b00};
    assign jta = {pc_plus4_D[31:28], instr_D[25:0], 2'b00};
    
    // --- PC Logic --- //
    dreg_pipelined pc_reg (
            .stall          (stall_pc),
            .clk            (clk),
            .rst            (rst),
            .d              (pc_next),
            .q              (pc_current)
        );

    adder pc_plus_4 (
            .a              (pc_current),
            .b              (32'd4),
            .y              (pc_plus4)
        );

    adder pc_plus_br (
            .a              (pc_plus4_E),
            .b              (ba),
            .y              (bta)
        );

    mux2 #(32) pc_reg_jmp_mux (
            .sel            (reg_jump), //change to reg_jump_WB
            .a              (pc_plus4),
            .b              (rd1_out),
            .y              (pc_rj_plus4)
        );

    mux2 #(32) pc_src_mux (
            .sel            (pc_src),
            .a              (pc_rj_plus4), // pc_pre
            .b              (bta), //
            .y              (pc_pre_WB)
        );

    mux2 #(32) pc_jmp_mux (
            .sel            (jump),
            .a              (pc_pre_WB),
            .b              (jta),
            .y              (pc_next)//pc jump + 4 on diagram
        );

    // --- RF Logic --- //
    mux2 #(5) rf_wa_mux (
            .sel            (reg_dst),
            .a              (instr_D[20:16]),
            .b              (instr_D[15:11]),
            .y              (reg_addr)
        );

    mux2 #(5) reg_addr_mux (
            .sel            (reg_jump),
            .a              (reg_addr),
            .b              (5'h1F),    //11111
            .y              (rf_wa)
        );

    regfile rf (
            .clk            (clk),
            .we             (we_reg_WB),
            .ra1            (instr_D[25:21]),
            .ra2            (instr_D[20:16]),
            .ra3            (ra3),
            .wa             (rf_wa_WB),
            .wd             (wd_rf),
            .rd1            (rd1_out),
            .rd2            (wd_dm),
            .rd3            (rd3)
        );

    signext se (
            .a              (instr_D[15:0]),
            .y              (sext_imm_D)
        );

    // --- ALU Logic --- //
    mux4 #(.WIDTH(32)) alu_DF_rd2_mux (
            .sel        (alu_DF_sel_rd2),
            .in0        (wd_dm_E),
            .in1        (alu_out_M),
            .in2        (wd_rf),
            .in3        (wd_dm_E),
            .out        (alu_DF_rd2_out)
        );

    mux2 #(32) alu_pb_mux (
            .sel            (alu_src_E),
            .a              (alu_DF_rd2_out),
            .b              (sext_imm_E),
            .y              (alu_pb)
        );

    mux4 #(.WIDTH(32)) alu_DF_rd1_mux (
            .sel        (alu_DF_sel_rd1),
            .in0        (rd1out_E),
            .in1        (alu_out_M),
            .in2        (wd_rf),
            .in3        (rd1out_E),
            .out        (alu_DF_rd1_out)
        );   
    
    mux2 #(5) shift_rd1_mux (
        .sel    (shift_mux_sel_E),
        .a      (alu_DF_rd1_out[4:0]),
        .b      (instr_D[10:6]),
        .y      (shift_rd1_out)
    );

    assign rd1_shift_full_out = {alu_DF_rd1_out[31:5], shift_rd1_out};
   
    assign lw_sw_zero_sel_E = we_dm_E | dm2reg_E;
    mux2 #(32) lw_sw_zero_mux (
        .sel    (lw_sw_zero_sel_E),
        .a      (rd1_shift_full_out),
        .b      (32'b0),
        .y      (alu_pa)
    );

    alu alu (
            .op             (alu_ctrl_E),
            .a              (alu_pa),
            .b              (alu_pb),
            .zero           (zero_E),
            .y              (alu_out),
            .y_hi           (alu_out_hi)
        );

    // --- MEM Logic --- //
    mux2 #(32) alu_mem_mux (
            .sel            (dm2reg_WB),
            .a              (alu_mux_out),
            .b              (rd_dm_WB),
            .y              (alu_mem_out)
        );

    mux2 #(32) rf_wd_mux (
            .sel            (jal_WB),
            .a              (alu_mem_out),
            .b              (pc_plus4_WB),
            .y              (wd_rf)
        );

    // HiLo Register & logic
    assign {hi_q, lo_q} = hilo_q;
    assign hilo_d_E = {alu_out_hi, alu_out};
    flopenr #(64) hilo_reg (
        .clk    (clk),
        .reset  (rst),
        .en     (we_hilo_M),
        .d      (hilo_d_M),
        .q      (hilo_q)
    );

    mux2 #(32) hilo_out_mux (
        .sel    (hilo_sel_M),
        .a      (lo_q),
        .b      (hi_q),
        .y    (hilo_mux_out)
    );

    mux2 #(32) alu_out_mux (
        .sel    (alu_out_sel_WB),
        .a      (alu_out_WB),
        .b      (hilo_mux_out_WB),
        .y    (alu_mux_out)
    );


wire test;
assign test = 0;

hazard_unit hazard_unit(
    .test               (test),

    // DF
    .rs1E                (rs1E),
    .rs2E                (rs2E),
    .waM                 (rf_wa_M),
    .we_reg_M            (we_reg_M),
    .waWB                 (rf_wa_WB),
    .we_reg_WB            (we_reg_WB),
    .alu_data_forward_rd1   (alu_DF_sel_rd1),
    .alu_data_forward_rd2   (alu_DF_sel_rd2),
    
    // lw DF
    .rs1D                   (instr_D[25:21]),
    .rs2D                   (instr_D[20:16]),
    .dm2reg_E               (dm2reg_E),

    // stall/flush
    .stall_pc           (stall_pc),
    .stall_f2d          (stall_f2d),
    .stall_d2e          (stall_d2e),
    .flush_d2e          (flush_d2e),
    .stall_e2m          (stall_e2m),
    .stall_m2wb         (stall_m2wb)
);

fetch2decode fetch2decode(
    .stall_f2d          (stall_f2d),

    .clk(clk),
    .rst(rst),
    .instr(instr),
    .pc_plus4(pc_plus4),
    .instr_D(instr_D),
    .pc_plus4_D(pc_plus4_D)
);

decode2execute decode2execute(
    .stall_d2e          (stall_d2e),
    .flush_d2e          (flush_d2e),
    // ALU DF
    .instr_D            (instr_D),
    .rs1E               (rs1E),
    .rs2E               (rs2E),
    
    .clk            (clk),
    .rst            (rst),
    .rd1out_D       (rd1_out),
    .wd_dm_D        (wd_dm),
    .sext_imm_D     (sext_imm_D),    
    .pc_plus4_D     (pc_plus4_D),
    .rf_wa          (rf_wa),
    

    .rd1out_E       (rd1out_E), 
    .wd_dm_E        (wd_dm_E),
    .sext_imm_E     (sext_imm_E),
    .pc_plus4_E     (pc_plus4_E),
    .rf_wa_E        (rf_wa_E),
    
    //control unit signals
    .we_hilo        (we_hilo),
    .alu_out_sel    (alu_out_sel),
    .shift_mux_sel  (shift_mux_sel),
    .jal            (jal),
    .hilo_sel       (hilo_sel),
    .reg_jump       (reg_jump),
    .jump           (jump),
    .dm2reg         (dm2reg),
    .we_dm          (we_dm_D),
    .branch         (branch),
    .alu_src        (alu_src),
    .alu_ctrl       (alu_ctrl),
    .we_reg         (we_reg),
    
    .we_hilo_E      (we_hilo_E),
    .alu_out_sel_E  (alu_out_sel_E),
    .shift_mux_sel_E(shift_mux_sel_E),
    .jal_E          (jal_E),
    .hilo_sel_E     (hilo_sel_E),
    .reg_jump_E     (reg_jump_E),
    .jump_E         (jump_E),
    .dm2reg_E       (dm2reg_E),
    .we_dm_E        (we_dm_E),
    .branch_E       (branch_E),
    .alu_src_E      (alu_src_E),
    .alu_ctrl_E     (alu_ctrl_E),
    .we_reg_E       (we_reg_E)

);
 
execute2memory execute2memory(
    .stall_e2m          (stall_e2m),
    
    .clk            (clk),
    .rst            (rst),
    .zero_E         (zero_E),
    .pc_plus4_E     (pc_plus4_E),
    .alu_out      (alu_out),
    .wd_dm_E        (wd_dm_E), 
    .hilo_d_E       (hilo_d_E),
    .rf_wa_E          (rf_wa_E),

    .zero_M         (zero_M),
    .pc_plus4_M     (pc_plus4_M),
    .alu_out_M      (alu_out_M), 
    .wd_dm_M        (wd_dm_M),
    .hilo_d_M       (hilo_d_M),
    .rf_wa_M        (rf_wa_M),
    
    //control unit signals
    .we_hilo_E      (we_hilo_E), 
    .alu_out_sel_E  (alu_out_sel_E),
    .jal_E          (jal_E),
    .hilo_sel_E     (hilo_sel_E),
    .reg_jump_E     (reg_jump_E),
    .jump_E         (jump_E),
    .dm2reg_E       (dm2reg_E),
    .we_dm_E        (we_dm_E),
    .branch_E       (branch_E),
    .we_reg_E       (we_reg_E),

    .we_hilo_M      (we_hilo_M),
    .alu_out_sel_M  (alu_out_sel_M),
    .jal_M          (jal_M),
    .hilo_sel_M     (hilo_sel_M),
    .reg_jump_M     (reg_jump_M),
    .jump_M         (jump_M),
    .dm2reg_M       (dm2reg_M),
    .we_dm_M        (we_dm_M),
    .branch_M       (branch_M),
    .we_reg_M       (we_reg_M)
);
 
memory2writeback memory2writeback(
    .stall_m2wb         (stall_m2wb),
    
    .rst            (rst),
    .clk            (clk),
    
    // data
        // in
    .hilo_mux_out   (hilo_mux_out),
    .rd_dm          (rd_dm),
    .alu_out_M      (alu_out_M), 
    .rf_wa_M        (rf_wa_M),
    .pc_plus4_M     (pc_plus4_M),
        // out
    .rd_dm_WB       (rd_dm_WB),
    .hilo_mux_out_WB    (hilo_mux_out_WB),
    .alu_out_WB      (alu_out_WB), 
    .rf_wa_WB        (rf_wa_WB),
    .pc_plus4_WB    (pc_plus4_WB),

    // cu
        // in 
    .alu_out_sel_M  (alu_out_sel_M),
    .jal_M          (jal_M),
    .reg_jump_M     (reg_jump_M),
    .jump_M         (jump_M),
    .dm2reg_M       (dm2reg_M),
    .pc_src         (pc_src),
    .we_reg_M       (we_reg_M),
        // out
    .alu_out_sel_WB (alu_out_sel_WB),
    .jal_WB         (jal_WB),
    .reg_jump_WB    (reg_jump_WB),
    .jump_WB        (jump_WB),
    .dm2reg_WB      (dm2reg_WB),
    .pc_src_WB      (pc_src_WB),
    .we_reg_WB       (we_reg_WB)
);
endmodule