`default_nettype none

// Top-level TinyTapeout Module for 8-bit MIPS Processor
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
  
  wire [7:0] ALU_out;
  wire [7:0] pc_out;

  mips_single_cycle cpu (
    .clk(clk),
    .rst(rst),
    .ALU_out(ALU_out),
    .pc_out(pc_out)
  );

  assign uo_out = ALU_out;
  assign uio_out = {4'b0, pc_out[3:0]};  // Show PC in lower 4 bits
  assign uio_oe = 8'hFF;
  
  wire _unused = &{ui_in, uio_in, ena, 1'b0};

endmodule

// Program Counter Module - 8-bit
module PC(
  input clk,
  input rst,
  input jump,
  input [7:0] jump_address,
  output reg [7:0] pc_out
);

  reg [7:0] p_c;
  wire [7:0] pc_next;

  assign pc_next = jump ? jump_address : (p_c + 1);

  always @(posedge clk) begin
    if (rst)
      p_c <= 8'd0;
    else if (p_c >= 8'd15)  // Only 16 instructions
      p_c <= 8'd0;
    else
      p_c <= pc_next;
  end

  always @(*) begin
    pc_out = p_c;
  end

endmodule

// Instruction Memory Module - 8-bit instructions
module instruction_memory(
  input [3:0] p_in,
  output reg [7:0] instruction
);

  reg [7:0] rom [0:15];  
  
  initial begin
    rom[0]  = 8'b0000_0001;  // ADD R0, R1
    rom[1]  = 8'b0001_0010;  // SUB R0, R2  
    rom[2]  = 8'b0010_0011;  // ADDI R0, 3
    rom[3]  = 8'b0011_0100;  // LW R0, [4]
    rom[4]  = 8'b0100_0101;  // SW R0, [5]
    rom[5]  = 8'b0110_0100;  // XOR R0, R0
    rom[6]  = 8'b0111_0000;  // OR R0, R0
    rom[7]  = 8'b0010_0010;  // ADDI R0, 2
    rom[8]  = 8'b0000_0001;  // ADD R0, R1
    rom[9]  = 8'b0001_0011;  // SUB R1, R1
    rom[10] = 8'b0011_0001;  // LW R0, [1]
    rom[11] = 8'b0100_0010;  // SW R0, [2]
    rom[12] = 8'b0010_0011;  // ADDI R0, 1
    rom[13] = 8'b0001_0001;  // SUB R0, R1
    rom[14] = 8'b0111_0000;  // OR R0, R0
    rom[15] = 8'b0101_0000;  // JUMP 0
  end

  always @(*) begin
    instruction = rom[p_in];
  end

endmodule

// Decoder Module - 8-bit
module decode(
  input [7:0] instruction_in,
  output reg [1:0] rs, rt, rd,
  output reg [3:0] im
);

  wire [3:0] opcode = instruction_in[7:4];

  always @(*) begin
    rs = 2'b00;
    rt = 2'b00;
    rd = 2'b00;
    im = 4'b0000;

    case(opcode)
      4'b0000, 4'b0001, 4'b0110, 4'b0111: begin  // R-type
        rd = instruction_in[3:2];
        rs = instruction_in[1:0];
        rt = instruction_in[1:0];
      end
      4'b0010: begin  // ADDI
        rd = instruction_in[3:2];
        im = instruction_in[3:0];
      end
      4'b0011, 4'b0100: begin  // LW/SW
        rd = instruction_in[3:2];
        im = instruction_in[3:0];
      end
      4'b0101: begin  // JUMP
        // Jump uses immediate field
        im = instruction_in[3:0];
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
    output reg [2:0] ALUOp
);

  always @(*) begin
    RegDst = 0; ALUsrc = 0; MemtoReg = 0; RegWrite = 0;
    MemWrite = 0; MemRead = 0; jump = 0; ALUOp = 3'b000;
    
    case (opcode)
      4'b0000: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=3'b000; end
      4'b0001: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=3'b001; end
      4'b0010: begin RegDst=1; ALUsrc=1; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=3'b000; end
      4'b0011: begin RegDst=1; ALUsrc=1; MemtoReg=1; RegWrite=1; MemWrite=0; MemRead=1; jump=0; ALUOp=3'b000; end
      4'b0100: begin RegDst=0; ALUsrc=1; MemtoReg=0; RegWrite=0; MemWrite=1; MemRead=0; jump=0; ALUOp=3'b000; end
      4'b0101: begin RegDst=0; ALUsrc=0; MemtoReg=0; RegWrite=0; MemWrite=0; MemRead=0; jump=1; ALUOp=3'b000; end
      4'b0110: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=3'b010; end
      4'b0111: begin RegDst=1; ALUsrc=0; MemtoReg=0; RegWrite=1; MemWrite=0; MemRead=0; jump=0; ALUOp=3'b011; end
      default: begin end
    endcase
  end
endmodule

// ALU Module - 8-bit
module ALU(
  input [7:0] A, B,
  input [2:0] ALUOp,
  output reg [7:0] ALU_out
);

  always @(*) begin
    case(ALUOp)
      3'b000: ALU_out = A + B;      // ADD/ADDI/LW/SW
      3'b001: ALU_out = A - B;      // SUB
      3'b010: ALU_out = A ^ B;      // XOR
      3'b011: ALU_out = A | B;      // OR
      default: ALU_out = 8'b0;
    endcase
  end
endmodule

// Data Memory Module - Much smaller
module data_memory(
  input clk,
  input MemWrite, MemRead,
  input [2:0] address,    // Only 8 memory locations
  input [7:0] write_data,
  output reg [7:0] read_data
);

  reg [7:0] mem [0:7];    // Only 8 locations

  initial begin
    mem[0] = 8'h12;
    mem[1] = 8'h34;
    mem[2] = 8'h56;
    mem[3] = 8'h78;
    mem[4] = 8'h9A;
    mem[5] = 8'hBC;
    mem[6] = 8'hDE;
    mem[7] = 8'hF0;
  end

  always @(posedge clk) begin
    if (MemWrite)
      mem[address] <= write_data;
  end

  always @(*) begin
    if (MemRead)
      read_data = mem[address];
    else
      read_data = 8'b0;
  end
endmodule

// MIPS Single Cycle CPU Module - 8-bit with minimal register file
module mips_single_cycle(
  input clk,
  input rst,
  output [7:0] ALU_out,
  output [7:0] pc_out
);

  wire [7:0] instruction;
  wire [1:0] rs, rt, rd;
  wire [3:0] im;
  wire RegDst, ALUsrc, MemtoReg, MemWrite, MemRead, RegWrite, jump;
  wire [7:0] Read_data1, Read_data2;
  wire [7:0] sign_ext_immediate;
  wire [7:0] alu_input_b;
  wire [7:0] mem_read_data;
  wire [1:0] write_reg;
  wire [7:0] write_data_final;
  wire [2:0] ALUOp;

  PC pc_inst(
    .clk(clk),
    .rst(rst),
    .jump(jump),
    .jump_address({4'b0, im}),
    .pc_out(pc_out)
  );

  instruction_memory imem(
    .p_in(pc_out[3:0]),
    .instruction(instruction)
  );

  decode dec(
    .instruction_in(instruction),
    .rs(rs),
    .rt(rt),
    .rd(rd),
    .im(im)
  );

  control_unit cu(
    .opcode(instruction[7:4]),
    .RegDst(RegDst),
    .ALUsrc(ALUsrc),
    .MemtoReg(MemtoReg),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .RegWrite(RegWrite),
    .jump(jump),
    .ALUOp(ALUOp)
  );

  // Minimal 4-register file instead of 16
  reg [7:0] reg_file [0:3];
  
  initial begin
    reg_file[0] = 8'h00;
    reg_file[1] = 8'h01;
    reg_file[2] = 8'h02;
    reg_file[3] = 8'h03;
  end

  assign Read_data1 = reg_file[rs];
  assign Read_data2 = reg_file[rt];

  assign sign_ext_immediate = {{4{im[3]}}, im};

  assign alu_input_b = ALUsrc ? sign_ext_immediate : Read_data2;

  ALU alu_inst(
    .A(Read_data1),
    .B(alu_input_b),
    .ALUOp(ALUOp),
    .ALU_out(ALU_out)
  );

  // Use lower 3 bits for memory address (8 locations)
  wire [2:0] mem_addr = ALU_out[2:0];

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

  always @(posedge clk) begin
    if (rst) begin
      reg_file[0] <= 8'h00;
      reg_file[1] <= 8'h01;
      reg_file[2] <= 8'h02;
      reg_file[3] <= 8'h03;
    end else if (RegWrite) begin
      reg_file[write_reg] <= write_data_final;
    end
  end

endmodule