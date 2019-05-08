module tb_system_pipelined;

    reg         clk;
    reg         rst;
    reg [31:0]  gpi0;
    reg [31:0]  gpi1;
    wire [31:0] gpo0;
    wire [31:0] gpo1;

    // For Testing
    wire [31:0] pc_current = DUT.pc_current;
    // wire [3:0] n_out = DUT.mips_top.factorial_accelerator.n_out;
    // wire done = DUT.mips_top.factorial_accelerator.Done;
    // wire [31:0] product = DUT.mips_top.factorial_accelerator.product;
    // wire dmem_we = DUT.mips_top.dmem.we;
    // wire [5:0] dmem_addr = DUT.mips_top.dmem.a;
    // wire [31:0] dmem_data = DUT.mips_top.dmem.d;



    integer error_count = 0;

    system_pipelined DUT(
        .clk(clk),
        .rst(rst),
        .gpi0(gpi0),
        .gpi1(gpi1),
        .gpo0(gpo0),
        .gpo1(gpo1)
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

    initial begin
        reset;
        gpi0 = 32'b1010;
        gpi1 = 32'b0000;
        // run_test;
        while(pc_current != 32'h140) 
        begin
            tick;
        end
        $display("Error Count: ", error_count);
        $finish;
    end

endmodule
