`default_nettype none

module tb;

  // DUT interface signals
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  wire [7:0] uo_out;
  reg [7:0] uio_in;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Instantiate the DUT
  tt_um_mips16_single_cycle dut (
    .clk(clk),
    .rst_n(rst_n),
    .ena(ena),
    .ui_in(ui_in),
    .uo_out(uo_out),
    .uio_in(uio_in),
    .uio_out(uio_out),
    .uio_oe(uio_oe)
  );

  // Clock generation
  initial clk = 0;
  always #5 clk = ~clk;  // 10ns clock period (100MHz)

  // Initial stimulus
  initial begin
    rst_n = 0;
    ena = 1;
    ui_in = 8'b0;
    uio_in = 8'b0;
    #20;
    rst_n = 1;
  end

endmodule
