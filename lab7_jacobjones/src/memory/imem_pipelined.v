module imem_pipelined (
        input  wire [5:0]  a,
        output wire [31:0] y
    );

    reg [31:0] rom [0:63];

    initial begin
        $readmemh ("lab8_pipeline_memfile.dat", rom);
    end

    assign y = rom[a];
    
endmodule