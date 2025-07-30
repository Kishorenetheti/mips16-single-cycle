`default_nettype none
`timescale 1ns / 1ps

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
  
  // Clock generation - slower for gate-level compatibility
  initial clk = 0;
  always #50 clk = ~clk;  // 20ns clock period (50MHz) - slower for gate-level
  
  // Initial stimulus
  initial begin
    // Initialize all inputs to known values
    rst_n = 0;
    ena = 1;
    ui_in = 8'b0;
    uio_in = 8'b0;
    
    // Hold reset for sufficient time
    #1000;
    rst_n = 1;
    
    // Let the design run
    #5000;
    
    $display("Testbench completed successfully");
    $finish;
  end
  
  // Optional: Monitor outputs (helpful for debugging)
  initial begin
    $monitor("Time: %0t, rst_n: %b, ena: %b, uo_out: %02h, uio_out: %02h, uio_oe: %02h", 
             $time, rst_n, ena, uo_out, uio_out, uio_oe);
  end
  
  // Generate VCD dump for waveform viewing
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
  end

endmodule

`default_nettype wire