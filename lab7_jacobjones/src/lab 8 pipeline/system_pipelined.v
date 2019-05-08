module system_pipelined(
        input wire clk,
        input wire rst,
        input wire [31:0] gpi0,
        input wire [31:0] gpi1,
        output wire [31:0] gpo0,
        output wire [31:0] gpo1
    );

    // wire [4:0]  ra3,
    // wire [31:0]  gpi0,
    // wire [31:0]  gpi1,
    wire        we_dm;
    wire [31:0] pc_current;
    wire [31:0] instr;
    wire [31:0] alu_out;
    wire [31:0] wd_dm;
    wire [31:0] rd_dm;
    wire [31:0] rd3;
    wire [31:0] wd_rf;
    wire [4:0] rf_wa;

    mips_top_pipelined mips_top (
        .clk            (clk),
        .rst            (rst),
        .ra3            (5'b00000),
        .gpi0           (gpi0),
        .gpi1           (gpi1),
        .gpo0           (gpo0),
        .gpo1           (gpo1),
        .we_dm          (we_dm),
        .pc_current     (pc_current),
        .instr          (instr),
        .alu_out        (alu_mux_out),
        .wd_dm          (wd_dm),
        .rd_dm          (rd_dm),
        .rd3            (rd3),
        .wd_rf          (wd_rf),
        .rf_wa          (rf_wa)
    );


endmodule