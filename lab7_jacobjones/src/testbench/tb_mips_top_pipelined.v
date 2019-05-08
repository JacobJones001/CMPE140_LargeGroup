module tb_mips_top_pipelined;

    reg         clk;
    reg         rst;
    wire        we_dm;
    wire [31:0] pc_current;
    wire [31:0] instr;
    wire [31:0] alu_mux_out;
    wire [31:0] wd_dm;
    wire [31:0] rd_dm;
    wire [31:0] DONT_USE;
    wire [31:0] wd_rf;
    wire [4:0]  rf_wa;
    reg [31:0]  gpi0;
    reg [31:0]  gpi1;
    wire [31:0] gpo0;
    wire [31:0] gpo1;
    
    // extra debug
    wire [31:0]  pc_next = DUT.mips.dp.pc_next;

    // Debug
    // --- F2D --- //
    wire [31:0]  instr_D = DUT.mips.dp.instr_D;
    wire [31:0]  pc_plus4_D = DUT.mips.dp.pc_plus4_D;
    // --- D2E --- //
        // -- data -- //
    wire [31:0]  rd1_out = DUT.mips.dp.rd1_out;
    wire [31:0]  wd_dm_D = DUT.mips.dp.wd_dm;
    wire [31:0]  sext_imm_D = DUT.mips.dp.sext_imm_D;
    wire [31:0]  pc_plus4 = DUT.mips.dp.pc_plus4;
    wire [31:0]  rd1out_E = DUT.mips.dp.rd1out_E;
    wire [31:0]  wd_dm_E = DUT.mips.dp.wd_dm_E;
    wire [31:0]  sext_imm_E = DUT.mips.dp.sext_imm_E;
    wire [31:0]  pc_plus4_E = DUT.mips.dp.pc_plus4_E;
    wire [4:0]  rf_wa_E = DUT.mips.dp.rf_wa_E;
        // -- cu -- //
    wire we_hilo = DUT.mips.dp.we_hilo;
    wire alu_out_sel = DUT.mips.dp.alu_out_sel;
    wire shift_mux_sel = DUT.mips.dp.shift_mux_sel;
    wire jal = DUT.mips.dp.jal;
    wire hilo_sel = DUT.mips.dp.hilo_sel;
    wire reg_jump = DUT.mips.dp.reg_jump;
    wire jump = DUT.mips.dp.jump;
    wire [1:0] dm2reg = DUT.mips.dp.dm2reg;
    wire we_dm_D = DUT.mips.dp.we_dm_D;
    // wire we_dm = DUT.mips.dp.we_dm; // From mips_top
    wire branch = DUT.mips.dp.branch;
    wire alu_src = DUT.mips.dp.alu_src;
    wire [2:0] alu_ctrl = DUT.mips.dp.alu_ctrl;
    wire we_reg = DUT.mips.dp.we_reg;
    wire we_hilo_E = DUT.mips.dp.we_hilo_E;
    wire alu_out_sel_E = DUT.mips.dp.alu_out_sel_E;
    wire shift_mux_sel_E = DUT.mips.dp.shift_mux_sel_E;
    wire jal_E = DUT.mips.dp.jal_E;
    wire hilo_sel_E = DUT.mips.dp.hilo_sel_E;
    wire reg_jump_E = DUT.mips.dp.reg_jump_E;
    wire jump_E = DUT.mips.dp.jump_E;
    wire [1:0] dm2reg_E = DUT.mips.dp.dm2reg_E;
    wire we_dm_E = DUT.mips.dp.we_dm_E;
    wire branch_E = DUT.mips.dp.branch_E;
    wire alu_src_E = DUT.mips.dp.alu_src_E;
    wire [2:0] alu_ctrl_E = DUT.mips.dp.alu_ctrl_E;
    wire we_reg_E = DUT.mips.dp.we_reg_E;
    // --- E2M --- //
        // -- data -- //
    wire [31:0]  alu_pa = DUT.mips.dp.alu_pa;
    wire [31:0]  alu_pb = DUT.mips.dp.alu_pb;
    wire [31:0]  alu_out = DUT.mips.dp.alu_out;
    wire [31:0]  alu_out_M = DUT.mips.dp.alu_out_M;
    wire [63:0]  hilo_d_E = DUT.mips.dp.hilo_d_E;
    wire [63:0]  hilo_d_M = DUT.mips.dp.hilo_d_M;
    wire [4:0]   rf_wa_M = DUT.mips.dp.rf_wa_M;
    wire [31:0]  pc_plus4_M = DUT.mips.dp.pc_plus4_M;
    wire [31:0]  wd_dm_M = DUT.mips.dp.wd_dm_M;
        // -- cu -- //
    wire zero_E = DUT.mips.dp.zero_E;
    wire zero_M = DUT.mips.dp.zero_M;
    wire we_hilo_M = DUT.mips.dp.we_hilo_M;
    wire alu_out_sel_M = DUT.mips.dp.alu_out_sel_M;
    wire jal_M = DUT.mips.dp.jal_M;
    wire hilo_sel_M = DUT.mips.dp.hilo_sel_M;
    wire reg_jump_M = DUT.mips.dp.reg_jump_M;
    wire jump_M = DUT.mips.dp.jump_M;
    wire [1:0] dm2reg_M = DUT.mips.dp.dm2reg_M;
    wire we_dm_M = DUT.mips.dp.we_dm_M;
    wire branch_M = DUT.mips.dp.branch_M;
    wire we_reg_M = DUT.mips.dp.we_reg_M;
    // --- M2W --- //
        // -- data -- //
    wire [31:0]  hilo_mux_out_WB = DUT.mips.dp.hilo_mux_out_WB;
    wire [31:0]  rd_dm_WB = DUT.mips.dp.rd_dm_WB;
    wire [31:0]  alu_out_WB = DUT.mips.dp.alu_out_WB;
    wire [4:0]   rf_wa_WB = DUT.mips.dp.rf_wa_WB;
    wire [31:0]  pc_plus4_WB = DUT.mips.dp.pc_plus4_WB;
        // -- cu -- //
    wire pc_src = DUT.mips.dp.pc_src;
    wire pc_src_WB = DUT.mips.dp.pc_src_WB;
    wire alu_out_sel_WB = DUT.mips.dp.alu_out_sel_WB;
    wire jal_WB = DUT.mips.dp.jal_WB;
    wire reg_jump_WB = DUT.mips.dp.reg_jump_WB;
    wire jump_WB = DUT.mips.dp.jump_WB;
    wire [1:0] dm2reg_WB = DUT.mips.dp.dm2reg_WB;
    wire we_reg_WB = DUT.mips.dp.we_reg_WB;

    // // Original
    // wire shift_mux_sel = DUT.mips.shift_mux_sel;
    // wire alu_out_sel = DUT.mips.alu_out_sel;
    // wire hilo_sel = DUT.mips.hilo_sel;
    // wire dm2reg = DUT.mips.dm2reg;
    // wire we_hilo = DUT.mips.we_hilo;
    // wire jal = DUT.mips.jal;
    // wire we_reg = DUT.mips.we_reg;
    // wire reg_jump = DUT.mips.reg_jump;
    // wire reg_dst = DUT.mips.reg_dst;
    // wire alu_src = DUT.mips.alu_src;
    // wire jump = DUT.mips.jump;
    // wire branch = DUT.mips.branch;
    // wire [63:0] hilo_d = DUT.mips.dp.hilo_d;
    // wire [63:0] hilo_q = DUT.mips.dp.hilo_q;
    // wire [31:0] alu_pa = DUT.mips.dp.alu_pa;
    // wire [31:0] alu_out = DUT.mips.dp.alu_out;
    // wire [31:0] pc_plus4 = DUT.mips.dp.pc_plus4;
    // wire [31:0] rd1_out = DUT.mips.dp.rd1_out;
    // wire [2:0] alu_ctrl = DUT.mips.dp.alu_ctrl;
    // wire [4:0] ra1 = instr[25:21];
    // // wire [31:0] instr = DUT.mips.dp.instr;
    // // wire [4:0] shift_rd1_out = DUT.mips.dp.shift_rd1_out;
    // // wire [31:0] shift_rd1_mux_a = DUT.mips.dp.shift_rd1_mux.a;
    // // wire [31:0] shift_rd1_mux_b = DUT.mips.dp.shift_rd1_mux.b;
    // // wire [31:0] shift_rd1_mux_y = DUT.mips.dp.shift_rd1_mux.y;
    // // wire pc_next = DUT.mips.pc_next;
    // wire [31:0] pc_next = DUT.mips.dp.pc_next;
    // wire [31:0] alu_out_hi = DUT.mips.dp.alu_out_hi;


    // // --- SOC -- //
    // wire WE1 = DUT.WE1;
    // wire WE2 = DUT.WE2;
    // wire WEM = DUT.WEM;
    // wire [1:0] RdSel = DUT.RdSel;
    // wire [31:0] RdGPIO = DUT.RdGPIO;
    // wire [31:0] RdFA = DUT.RdFA;
    // // --- GPIO --- //
    // wire gpio_WE1 = DUT.gpio.WE1;
    // wire gpio_WE2 = DUT.gpio.WE2;
    // wire [1:0] gpio_RdSel = DUT.gpio.RdSel;
    // // --- Factorial --- //
    // wire [31:0] fact_product = DUT.factorial_accelerator.product;
    // wire [3:0] fact_n_out = DUT.factorial_accelerator.n_out;
    // wire x_gt_1 = DUT.factorial_accelerator.factorial.x_gt_1;
    // wire [31:0] out_CNT = DUT.factorial_accelerator.factorial.FACT_DP.out_CNT;
    // wire [31:0] n = DUT.factorial_accelerator.factorial.FACT_DP.n;
    // wire GoPulseOut = DUT.factorial_accelerator.GoPulseOut;
    // wire Done = DUT.factorial_accelerator.Done;
    // wire Err = DUT.factorial_accelerator.Err;
    // wire sel_MUX = DUT.factorial_accelerator.factorial.sel_MUX;
    // wire [2:0] fact_cs = DUT.factorial_accelerator.factorial.FACT_CU.CS;


    integer error_count = 0;
    integer a, b;
    reg [63:0] expected_product;
    integer expected_jmp_value; // JR and JAL check
    integer reg1_value;

    
    mips_top_pipelined DUT (
            .clk            (clk),
            .rst            (rst),
            .we_dm          (we_dm),
            .ra3            (5'b00000),
            .gpi0           (gpi0),
            .gpi1           (gpi1),
            .gpo0           (gpo0),
            .gpo1           (gpo1),
            .pc_current     (pc_current),
            .instr          (instr),
            .alu_out        (alu_mux_out),
            .wd_dm          (wd_dm),
            .rd_dm          (rd_dm),
            .rd3            (DONT_USE),
            .wd_rf          (wd_rf),
            .rf_wa          (rf_wa)
        );
    
    task tick; 
    begin 
        clk = 1'b0; #5;
        clk = 1'b1; #5;
    end
    endtask

    task reset;
    begin 
        rst = 1'b0; #5;
        rst = 1'b1; #5;
        rst = 1'b0;
    end
    endtask

    // task CHECK_MULTU;
    // begin
    //     a = 2;
    //     b = 4;
    //     expected_product = a*b;
    //     if({alu_out_hi, alu_mux_out} != expected_product) begin
    //         error_count = error_count + 1;
    //         $display("Error: MULTU - Product");
    //     end 
    // end
    // endtask

    // task CHECK_MFHI;
    // begin
    //     if(wd_rf[31:0] != expected_product[63:32]) begin
    //         error_count = error_count + 1;
    //         $display("Error: MFHI - Value");
    //     end 
    // end
    // endtask

    // task CHECK_MFLO;
    // begin
    //     if(wd_rf[31:0] != expected_product[31:0]) begin
    //         error_count = error_count + 1;
    //         $display("Error: MFLO - Value");
    //     end 
    // end
    // endtask

    // task CHECK_JR;
    // begin
    //     expected_jmp_value = 'h1c;
    //     if(pc_next != expected_jmp_value) begin
    //         error_count = error_count + 1;
    //         $display("Error: CHECK_JR - Value");
    //     end 
    // end
    // endtask

    // task CHECK_JAL;
    // begin
    //     if(pc_next[27:2] != instr[25:0]) begin
    //         error_count = error_count + 1;
    //         $display("Error: CHECK_JAL - Value");
    //     end 
    // end
    // endtask

    // task CHECK_SLL;
    // begin
    //     reg1_value = 1;
    //     if(alu_mux_out != (reg1_value << instr[10:6])) begin
    //         error_count = error_count + 1;
    //         $display("Error: CHECK_SLL - Value");
    //     end 
    // end
    // endtask

    // task CHECK_SRL;
    // begin
    //     reg1_value = 8;
    //     if(alu_mux_out != (reg1_value >> instr[10:6])) begin
    //         error_count = error_count + 1;
    //         $display("Error: CHECK_SRL - Value");
    //     end 
    // end
    // endtask

    // task run_test;
    // begin
    //     tick;
    //     tick;
    //     tick;
    //     CHECK_MULTU;
    //     tick; 
    //     CHECK_MFLO;
    //     tick;
    //     CHECK_JR;
    //     tick;
    //     CHECK_JAL;
    //     tick; // addi
    //     tick;
    //     CHECK_SLL;
    //     tick;
    //     CHECK_SRL;
    //     tick;
        
    // end
    // endtask

    
    initial begin
        reset;
        gpi0 = 32'b1010;
        gpi1 = 32'b0110;
        // run_test;
        while(pc_current != 32'h140) 
        begin
            tick;
        end
        $display("Error Count: ", error_count);
        $finish;
    end


endmodule