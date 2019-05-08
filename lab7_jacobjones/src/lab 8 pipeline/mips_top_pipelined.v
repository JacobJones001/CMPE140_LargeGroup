module mips_top_pipelined (
        input  wire        clk,
        input  wire        rst,
        input  wire [4:0]  ra3,
        input  wire [31:0]  gpi0,
        input  wire [31:0]  gpi1,
        output wire [31:0]  gpo0,
        output wire [31:0]  gpo1,
        output wire        we_dm,
        output wire [31:0] pc_current,
        output wire [31:0] instr,
        output wire [31:0] alu_out,
        output wire [31:0] wd_dm,
        output wire [31:0] rd_dm,
        output wire [31:0] rd3,
        output wire [31:0] wd_rf,
        output wire [4:0] rf_wa
    );

    // Data Memory Edit
    wire [31:0] alu_out_M;
    wire [31:0] wd_dm_M;      

    // wire [31:0] gpi0;
    // wire [31:0] gpi1;
    // assign gpi0 = {28'b0, gpi0_in};
    // assign gpi1 = {28'b0, gpi1_in};

    wire [31:0] DONT_USE;
    wire WE1;
    wire WE2;
    wire WEM;
    wire [1:0] RdSel;
    wire [31:0] RdDM; 
    wire [31:0] RdFA; 
    wire [31:0] RdGPIO; 



    mips_pipelined mips (
            .clk            (clk),
            .rst            (rst),
            .ra3            (ra3),
            .instr          (instr),
            .rd_dm          (rd_dm),
            .we_dm          (we_dm),
            .pc_current     (pc_current),
            .alu_out        (alu_out),
            .wd_dm          (wd_dm),
            .rd3            (rd3),
            .wd_rf          (wd_rf),
            .rf_wa          (rf_wa),
            .alu_out_M      (alu_out_M),
            .wd_dm_M        (wd_dm_M)
        );

    imem_pipelined imem (
            .a              (pc_current[7:2]),
            .y              (instr)
        );

    dmem dmem (
            .clk            (clk),
            .we             (WEM),
            .a              (alu_out_M[7:2]),
            .d              (wd_dm_M),
            .q              (RdDM)
        );

    gpio gpio (
        .clk(clk),
        .reset(rst),
        .A(alu_out_M[3:2]),
        .WE(WE2),
        .WD(wd_dm_M),
        .gpi0(gpi0),
        .gpi1(gpi1),
        .gpo0(gpo0),
        .gpo1(gpo1),
        .RD(RdGPIO)
    );

    factorial_accelerator factorial_accelerator (
        .clk(clk),
        .reset(rst),
        .A(alu_out_M[3:2]),
        .WE(WE1),
        .WD(wd_dm_M[3:0]),
        .RD(RdFA)
    );

    soc_addr_dec soc_addr_dec (
        .WE(we_dm),
        .A(alu_out_M),
        .WE1(WE1),
        .WE2(WE2),
        .WEM(WEM),
        .RdSel(RdSel)
    );

    mux4 #(32) read_data_mux(
        .sel(RdSel),
        .in0(RdDM),
        .in1(RdDM),
        .in2(RdFA),
        .in3(RdGPIO),
        .out(rd_dm)
    );

endmodule