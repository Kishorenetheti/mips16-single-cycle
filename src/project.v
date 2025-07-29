`default_nettype none

module tt_um_mips16_minimal (
    input  wire clk,
    input  wire rst_n,
    output wire [7:0] uo_out
);

  wire rst = ~rst_n;

  wire [15:0] alu_out;

  // Simple CPU instance
  mips_cpu_minimal cpu(
    .clk(clk),
    .rst(rst),
    .alu_out(alu_out)
  );

  assign uo_out = alu_out[7:0];

endmodule

// Program Counter
module PC(
  input clk,
  input rst,
  input jump,
  input [3:0] jump_addr,
  output reg [3:0] pc_out
);

  always @(posedge clk or posedge rst) begin
    if (rst)
      pc_out <= 4'd0;
    else if (jump)
      pc_out <= jump_addr;
    else
      pc_out <= pc_out + 1;
  end

endmodule

// Instruction Memory: 8 instructions only
module instruction_memory(
  input [3:0] addr,
  output reg [15:0] instruction
);

  always @(*) begin
    case(addr)
      4'd0: instruction = 16'h0123; // ADD example
      4'd1: instruction = 16'h1234; // SUB example
      4'd2: instruction = 16'h2345; // ADDI example
      4'd3: instruction = 16'h3456; // LW example
      4'd4: instruction = 16'h4567; // SW example
      4'd5: instruction = 16'h5678; // OR example
      4'd6: instruction = 16'h6789; // XOR example
      4'd7: instruction = 16'h7000; // JUMP to 0
      default: instruction = 16'h0000;
    endcase
  end

endmodule

// Register File: only 4 registers
module reg_file(
  input clk,
  input rst,
  input [1:0] rs,
  input [1:0] rt,
  input [1:0] rd,
  input [15:0] write_data,
  input reg_write,
  output [15:0] read_data1,
  output [15:0] read_data2
);

  reg [15:0] regs [0:3];
  integer i;

  // reset all registers
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      for (i=0; i<4; i=i+1)
        regs[i] <= 16'd0;
    end else if (reg_write) begin
      regs[rd] <= write_data;
    end
  end

  assign read_data1 = regs[rs];
  assign read_data2 = regs[rt];

endmodule

// ALU: only ADD and SUB for now
module ALU(
  input [15:0] A,
  input [15:0] B,
  input alu_op, // 0=ADD, 1=SUB
  output reg [15:0] alu_out
);

  always @(*) begin
    alu_out = alu_op ? (A - B) : (A + B);
  end

endmodule

// Minimal CPU module
module mips_cpu_minimal(
  input clk,
  input rst,
  output [15:0] alu_out
);

  wire [3:0] pc;
  wire [15:0] instruction;
  wire [1:0] rs, rt, rd;
  wire alu_op;
  wire reg_write;

  wire [15:0] reg_read1, reg_read2;
  wire [15:0] alu_input_b;

  // Program counter instance
  PC pc_inst(
    .clk(clk),
    .rst(rst),
    .jump(1'b0),
    .jump_addr(4'd0),
    .pc_out(pc)
  );

  // Instruction memory instance
  instruction_memory imem(
    .addr(pc),
    .instruction(instruction)
  );

  // Simple instruction decode:
  // instruction format: [15:12]=opcode, [11:10]=rd, [9:8]=rs, [7:6]=rt, [5:0]=imm (ignored here)
  assign alu_op = instruction[15];   // MSB as ALU op: 0=ADD, 1=SUB
  assign rd = instruction[11:10];
  assign rs = instruction[9:8];
  assign rt = instruction[7:6];
  assign reg_write = 1'b1; // always write back for simplicity

  // Register file
  reg_file rf(
    .clk(clk),
    .rst(rst),
    .rs(rs),
    .rt(rt),
    .rd(rd),
    .write_data(alu_out),
    .reg_write(reg_write),
    .read_data1(reg_read1),
    .read_data2(reg_read2)
  );

  assign alu_input_b = reg_read2;

  // ALU
  ALU alu(
    .A(reg_read1),
    .B(alu_input_b),
    .alu_op(alu_op),
    .alu_out(alu_out)
  );

endmodule
