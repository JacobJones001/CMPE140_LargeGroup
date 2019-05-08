module system_fpga (
        input  wire       clk,
        input  wire       rst,
        input  wire       button,
        input  wire [4:0] switches,
        output wire [1:0] SINGLE_LED,
        output wire [3:0] LEDSEL,
        output wire [7:0] LEDOUT
    );

    wire [31:0] gpo0;
    wire [31:0] gpo1;
    wire [31:0] gpi0_in;

    reg  [15:0] reg_hex;
    wire        clk_sec;
    wire        clk_5KHz;
    wire        clk_pb;

    wire [7:0]  digit0;
    wire [7:0]  digit1;
    wire [7:0]  digit2;
    wire [7:0]  digit3;

    assign SINGLE_LED[0] = switches[4];
    assign SINGLE_LED[1] = 1'b0;

    clk_gen clk_gen (
            .clk100MHz          (clk),
            .rst                (rst),
            .clk_4sec           (clk_sec),
            .clk_5KHz           (clk_5KHz)
        );

    button_debouncer bd (
            .clk                (clk_5KHz),
            .button             (button),
            .debounced_button   (clk_pb)
        );

    assign gpi0_in = {28'b0, switches[3:0]};

    system_pipelined system(
        .clk(clk_pb),
        .rst(rst),
        .gpi0(gpi0_in),
        .gpi1(gpo1), // looped
        .gpo0(gpo0),
        .gpo1(gpo1)
    );

    // mux2 #(32) gpo0_mux (
    //         .sel            (switches[4]),
    //         .a              (gpo0[15:0]),
    //         .b              (gpo0[31:16]),
    //         .y              (reg_hex)
    //     );

    always@ (posedge clk) begin
        case(switches[4])
            1'b0: reg_hex = gpo0[15:0];
            1'b1: reg_hex = gpo0[31:16];
            default: reg_hex = 'hFFFF;
        endcase
    end

    hex_to_7seg hex3 (
            .HEX                (reg_hex[15:12]),
            .s                  (digit3)
        );

    hex_to_7seg hex2 (
            .HEX                (reg_hex[11:8]),
            .s                  (digit2)
        );

    hex_to_7seg hex1 (
            .HEX                (reg_hex[7:4]),
            .s                  (digit1)
        );

    hex_to_7seg hex0 (
            .HEX                (reg_hex[3:0]),
            .s                  (digit0)
        );

    led_mux led_mux (
            .clk                (clk_5KHz),
            .rst                (rst),
            .LED3               (digit3),
            .LED2               (digit2),
            .LED1               (digit1),
            .LED0               (digit0),
            .LEDSEL             (LEDSEL),
            .LEDOUT             (LEDOUT)
        );


endmodule