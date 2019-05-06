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
        input  wire [1:0]  dm2reg,
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
        output wire [4:0] rf_wa
    );

        //clarence edit
        wire we_dm_D;
         wire zero_E;        
        wire alu_out_E;      
       wire hilo_d_E; 
    
       wire zero_M;        
        wire pc_plus4_M;     
        wire alu_out_M;      
        wire wd_dm_M;       
        wire hilo_d_M;       
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
        wire alu_ctrl_E;
        wire alu_src_E;

        wire we_hilo_M;
        wire alu_out_sel_M;
        wire jal_M;
        wire hilo_sel_M;
        wire reg_jump_M;
        wire jump_M;
        wire dm2reg_M;
        wire we_dm_M;
        wire branch_M;

        wire alu_out_sel_WB;
        wire jal_WB;
        wire reg_jump_WB;
        wire jump_WB;
        wire dm2reg_WB;
        wire pc_src_WB;
        wire rd_dm_WB;
        wire hilo_mux_out_WB;
        wire pc_plus4_WB;

    wire [4:0]  reg_addr;
    // wire [4:0]  rf_wa;
    wire        pc_src;
    wire [31:0] pc_plus4;
    wire [31:0] pc_pre_WB;
    wire [31:0] pc_next;
    wire [31:0] pc_rj_plus4;
    wire [31:0] sext_imm;
    wire [31:0] ba;
    wire [31:0] bta;
    wire [31:0] jta;
    wire [31:0] rd1_out;
    wire [31:0] alu_pa;
    wire [31:0] alu_pb;
    wire [31:0] alu_mem_out;
    wire [31:0] rd1out_D;
    wire [31:0] wd_dm_D;
    wire [31:0] sext_imm_D;    
    wire [31:0] pc_plus4_D;
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
    
    assign pc_src = branch_M & zero_M;
    assign ba = {sext_imm[29:0], 2'b00};
    assign jta = {pc_plus4[31:28], instr[25:0], 2'b00};
    
    // --- PC Logic --- //
    dreg pc_reg (
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
            .sel            (reg_jump_WB), //change to reg_jump_WB
            .a              (pc_plus4),
            .b              (rd1_out),
            .y              (pc_rj_plus4)
        );

    mux2 #(32) pc_src_mux (
            .sel            (pc_src_WB),
            .a              (pc_rj_plus4), // pc_pre
            .b              (bta), //
            .y              (pc_pre_WB)
        );

    mux2 #(32) pc_jmp_mux (
            .sel            (jump_WB),
            .a              (pc_pre_WB),
            .b              (jta),
            .y              (pc_next)//pc jump + 4 on diagram
        );

    // --- RF Logic --- //
    mux2 #(5) rf_wa_mux (
            .sel            (reg_dst),
            .a              (instr[20:16]),
            .b              (instr[15:11]),
            .y              (reg_addr)
        );

    mux2 #(5) reg_addr_mux (
            .sel            (reg_jump_E),
            .a              (reg_addr),
            .b              (5'h1F),    //11111
            .y              (rf_wa)
        );

    regfile rf (
            .clk            (clk),
            .we             (we_reg),
            .ra1            (instr[25:21]),
            .ra2            (instr[20:16]),
            .ra3            (ra3),
            .wa             (rf_wa),
            .wd             (wd_rf),
            .rd1            (rd1_out),
            .rd2            (wd_dm),
            .rd3            (rd3)
        );

    signext se (
            .a              (instr[15:0]),
            .y              (sext_imm_D)
        );

    // --- ALU Logic --- //
    mux2 #(32) alu_pb_mux (
            .sel            (alu_src_E),
            .a              (wd_dm_E),
            .b              (sext_imm_E),
            .y              (alu_pb)
        );
    
    assign alu_pa = {rd1_out[31:5], shift_rd1_out};
    // assign alu_pa = {rd1_out[31:5], rd1_out[4:0]};
    // assign alu_pa = (shift_mux_sel) ? instr : rd1_out;
    
    mux2 #(5) shift_rd1_mux (
        .sel    (shift_mux_sel_E),
        .a      (rd1_out[4:0]),
        .b      (instr[10:6]),
        .y      (shift_rd1_out)
    );

    // mux2 #(32) shift_rd1_mux (
    //         .sel            (shift_mux_sel),
    //         .a              (rd1_out),
    //         .b              (instr),
    //         .y              (alu_pa)
    // );

    alu alu (
            .op             (alu_ctrl_E),
            .a              (alu_pa),
            .b              (alu_pb),
            .zero           (zero_E),
            .y              (alu_out_E),
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
        .a      (alu_out),
        .b      (hilo_mux_out_WB),
        .y    (alu_mux_out)
    );

 
decode2execute decode2execute(
    .clk            (clk),
    .rst            (rst),
    .rd1out_D       (rd1out_D),
    .wd_dm_D        (wd_dm_D),
    .sext_imm_D     (sext_imm_D),    
    .pc_plus4_D     (pc_plus4_D),
    

    .rd1out_E       (rd1out_E), 
    .wd_dm_E        (wd_dm_E),
    .sext_imm_E     (sext_imm_E),
    .pc_plus4_E     (pc_plus4_E),
    
    
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
    .alu_ctrl_E     (alu_ctrl_E)
);
 
execute2memory execute2memory(
    .clk            (clk),
    .rst            (rst),
    .zero_E         (zero_E),
    .pc_plus4_E     (pc_plus4_E),
    .alu_out_E      (alu_out_E),
    .wd_dm_E        (wd_dm_E), 
    .hilo_d_E       (hilo_d_E), 

    .zero_M         (zero_M),
    .pc_plus4_M     (pc_plus4_M),
    .alu_out_M      (alu_out_M), 
    .wd_dm_M        (wd_dm_M),
    .hilo_d_M       (hilo_d_M),
        
    
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
        
    .we_hilo_M      (we_hilo_M),
    .alu_out_sel_M  (alu_out_sel_M),
    .jal_M          (jal_M),
    .hilo_sel_M     (hilo_sel_M),
    .reg_jump_M     (reg_jump_M),
    .jump_M         (jump_M),
    .dm2reg_M       (dm2reg_M),
    .we_dm_M        (we_dm_M),
    .branch_M       (branch_M)
);
 
memory2writeback memory2writeback(
    .alu_out_sel_M  (alu_out_sel_M),
    .jal_M          (jal_M),
    .reg_jump_M     (reg_jump_M),
    .jump_M         (jump_M),
    .dm2reg_M       (dm2reg_M),
    .pc_src         (pc_src),
    .rd_dm          (rd_dm),
    .hilo_mux_out   (hilo_mux_out),
    .rst            (rst),
    .clk            (clk),

    .alu_out_sel_WB (alu_out_sel_WB),
    .jal_WB         (jal_WB),
    .reg_jump_WB    (reg_jump_WB),
    .jump_WB        (jump_WB),
    .dm2reg_WB      (dm2reg_WB),
    .pc_src_WB      (pc_src_WB),
    .rd_dm_WB       (rd_dm_WB),
    .hilo_mux_out_WB    (hilo_mux_out_WB)
);
endmodule