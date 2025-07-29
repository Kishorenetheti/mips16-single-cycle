`default_nettype none

// Top-level TinyTapeout Module for 16-bit MIPS Processor
module tt_um_mips16_single_cycle (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  wire rst = !rst_n;
  
  wire [15:0] ALU_out;

  mips_single_cycle cpu (
    .clk(clk),
    .rst(rst),
    .ALU_out(ALU_out)
  );

  assign uo_out = ALU_out[7:0];
  assign uio_out = ALU_out[15:8];
  assign uio_oe = 8'hFF;
  
  wire _unused = &{ui_in, uio_in, ena, 1'b0};

endmodule

// Program Counter Module
module PC(
  input clk,
  input rst,
  input jump,
  input [15:0] jump_address,
  output reg [15:0] pc_out
);

  reg [15:0] p_c;
  wire [15:0] pc_next;

  assign pc_next = (jump) ? jump_address : (p_c + 2);

  always @(posedge clk) begin
    if (rst)
      p_c <= 16'd0;
    else if (p_c >= 16'd30)
      p_c <= 16'd0;
    else
      p_c <= pc_next;
  end

  always @(*) begin
    pc_out = p_c;
  end

endmodule

// Instruction Memory Module
module instruction_memory(
  input [15:0] p_in,
  output reg [15:0] instruction
);

  reg [15:0] rom [0:15];  
  
  initial begin
    rom[0]  = 16'b0000_0001_0010_0011;
    rom[1]  = 16'b0001_0010_0011_0100;
    rom[2]  = 16'b0010_0011_0100_0101;
    rom[3]  = 16'b0011_0100_0101_0011;
    rom[4]  = 16'b0100_0101_0100_0011;
    rom[5]  = 16'b0110_0100_0011_0011;
    rom[6]  = 16'b0111_0000_0000_0011;
    rom[7]  = 16'b0010_0010_0010_1111;
    rom[8]  = 16'b0000_0001_0001_0001;
    rom[9]  = 16'b0001_0011_0001_0000;
    rom[10] = 16'b0011_0001_0010_0100;
    rom[11] = 16'b0100_0010_0011_0100;
    rom[12] = 16'b0010_0011_0000_0001;
    rom[13] = 16'b0001_0001_0000_0001;
    rom[14] = 16'b0111_0000_0101_0100;
    rom[15] = 16'b0101_0000_0000_0000;
  end

  /* verilator lint_off WIDTHEXPAND */
  always @(*) begin
    if ((p_in[3:0]) < 16)
      instruction = rom[p_in[3:0]];
    else
      instruction = 16'b0;
  end
  /* verilator lint_on WIDTHEXPAND */

endmodule

// Decoder Module
module decode(
  input [15:0] instruction_in,
  output reg [3:0] rs, rt, opcode, rd, im,
  output reg [11:0] jump
);

  always @(*) begin
    rs = 4'b0000;
    rt = 4'b0000;
    rd = 4'b0000;
    im = 4'b0000;
    jump = 12'b0;
    opcode = instruction_in[15:12];   

    case(opcode)
      4'b0000: begin
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end
      4'b0001: begin
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end
      4'b0010: begin
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        im = instruction_in[3:0];
      end
      4'b0011: begin
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        im = instruction_in[3:0];
      end
      4'b0100: begin
        rt = instruction_in[11:8];
        rs = instruction_in[7:4];
        im = instruction_in[3:0];
      end
      4'b0101: begin
        jump = instruction_in[11:0];
      end
      4'b0110: begin
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end
      4'b0111: begin
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end
      default: begin
        // Default no operation
      end
    endcase
  end
endmodule

// Control Unit Module
module control_unit(
    input [3:0] opcode,
    output reg RegDst, ALUsrc, MemtoReg, MemWrite, MemRead, RegWrite, jump,
    output reg [3:0] ALUOp
);

  always @(*) begin
    RegDst = 0; ALUsrc = 0; MemtoReg = 0; RegWrite = 0;
    MemWrite = 0; MemRead = 0; jump = 0; ALUOp = 4'b0;
    
    case (opcode)
      4'b0000: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=4'b0000; end
      4'b0001: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=4'b0001; end
      4'b0010: begin RegDst=0; ALUsrc=1; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=4'b0010; end
      4'b0011: begin RegDst=0; ALUsrc=1; MemtoReg=1; RegWrite=1; MemWrite=0; MemRead=1; jump=0; ALUOp=4'b0011; end
      4'b0100: begin RegDst=0; ALUsrc=1; MemtoReg=0; RegWrite=0; MemWrite=1; MemRead=0; jump=0; ALUOp=4'b0100; end
      4'b0101: begin RegDst=0; ALUsrc=0; MemtoReg=0; RegWrite=0; MemWrite=0; MemRead=0; jump=1; ALUOp=4'b0000; end
      4'b0110: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=4'b0110; end
      4'b0111: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=4'b0111; end
      default: begin end
    endcase
  end
endmodule

// ALU Module
module ALU(
  input [15:0] A, B,
  input [3:0] ALUOp,
  output reg [15:0] ALU_out
);

  always @(*) begin
    case(ALUOp)
      4'b0000: ALU_out = A + B;      // ADD
      4'b0001: ALU_out = A - B;      // SUB
      4'b0010: ALU_out = A + B;      // ADDI
      4'b0011: ALU_out = A + B;      // LW
      4'b0100: ALU_out = A + B;      // SW
      4'b0110: ALU_out = A ^ B;      // XOR
      4'b0111: ALU_out = A | B;      // OR
      default: ALU_out = 16'b0;
    endcase
  end
endmodule

// Data Memory Module
module data_memory(
  input clk,
  input MemWrite, MemRead,
  input [5:0] address, // 6-bit address to reduce size
  input [15:0] write_data,
  output reg [15:0] read_data
);

  reg [15:0] mem [0:63];
  integer i;

  initial begin
    mem[0] = 16'h1234;
    mem[1] = 16'h5678;
    mem[2] = 16'h9ABC;
    mem[3] = 16'hDEF0;
    for (i = 4; i < 64; i = i + 1)
      mem[i] = 16'h0;
  end

  always @(posedge clk) begin
    if (MemWrite)
      mem[address] <= write_data;
  end

  always @(*) begin
    if (MemRead)
      read_data = mem[address];
    else
      read_data = 16'b0;
  end
endmodule

// MIPS Single Cycle CPU Module
module mips_single_cycle(
  input clk,
  input rst,
  output [15:0] ALU_out
);

  wire [15:0] instruction;
  wire [15:0] pc;
  wire RegDst, ALUsrc, MemtoReg, MemWrite, MemRead, RegWrite, jump;
  wire [15:0] mem_read_data, alu_input_b;
  wire [3:0] write_reg;
  wire [15:0] write_data_final;
  wire [15:0] Read_data1, Read_data2;
  wire [15:0] sign_ext_immediate;
  wire [3:0] rs, rt, rd, im;
  wire [3:0] ALUOp;

  PC pc_inst(
    .clk(clk),
    .rst(rst),
    .jump(jump),
    .jump_address({4'b0, instruction[11:0]}),
    .pc_out(pc)
  );

  instruction_memory imem(
    .p_in(pc),
    .instruction(instruction)
  );

  decode dec(
    .instruction_in(instruction),
    .rs(rs),
    .rt(rt),
    .rd(rd),
    .opcode(),
    .im(im),
    .jump()
  );

  control_unit cu(
    .opcode(instruction[15:12]),
    .RegDst(RegDst),
    .ALUsrc(ALUsrc),
    .MemtoReg(MemtoReg),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .RegWrite(RegWrite),
    .jump(jump),
    .ALUOp(ALUOp)
  );

  reg [15:0] reg_file [0:15];
  integer j;
  initial begin
    reg_file[0] = 16'h0; reg_file[1] = 16'h1; reg_file[2] = 16'h2;
    reg_file[3] = 16'h3; reg_file[4] = 16'h4; reg_file[5] = 16'h5;
    for (j = 6; j < 16; j = j + 1)
      reg_file[j] = 16'h0;
  end

  assign Read_data1 = reg_file[rs];
  assign Read_data2 = reg_file[rt];

  assign sign_ext_immediate = {{12{im[3]}}, im};

  assign alu_input_b = ALUsrc ? sign_ext_immediate : Read_data2;

  ALU alu_inst(
    .A(Read_data1),
    .B(alu_input_b),
    .ALUOp(ALUOp),
    .ALU_out(ALU_out)
  );

  // Use lower 6 bits of ALU_out for memory address (reduce size)
  wire [5:0] mem_addr = ALU_out[5:0];

  data_memory dmem(
    .clk(clk),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .address(mem_addr),
    .write_data(Read_data2),
    .read_data(mem_read_data)
  );

  assign write_reg = RegDst ? rd : rt;

  assign write_data_final = MemtoReg ? mem_read_data : ALU_out;

  integer k;
  always @(posedge clk) begin
    if (rst) begin
      reg_file[0] <= 16'h0;
      reg_file[1] <= 16'h1;
      reg_file[2] <= 16'h2;
      reg_file[3] <= 16'h3;
      reg_file[4] <= 16'h4;
      reg_file[5] <= 16'h5;
      for (k = 6; k < 16; k = k + 1)
        reg_file[k] <= 16'h0;
    end else if (RegWrite) begin
      reg_file[write_reg] <= write_data_final;
    end
  end

endmodule
