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

  // Convert reset to positive logic
  wire rst = !rst_n;
  
  // Internal signals from MIPS processor
  wire [15:0] ALU_out;
  wire [3:0] ALUOp;
  wire [15:0] Read_data1;
  wire [15:0] Read_data2;
  wire [15:0] sign_ext_immediate;
  wire [3:0] rs, rt, rd;
  wire [3:0] Write_reg_out;

  // Instantiate the MIPS processor
  mips_single_cycle cpu (
    .clk(clk),
    .rst(rst),
    .ALU_out(ALU_out),
    .ALUOp(ALUOp),
    .Read_data1(Read_data1),
    .Read_data2(Read_data2),
    .sign_ext_immediate(sign_ext_immediate),
    .rs(rs),
    .rt(rt),
    .rd(rd),
    .Write_reg_out(Write_reg_out)
  );

  // Map outputs to TinyTapeout pins
  assign uo_out = ALU_out[7:0];    // Lower 8 bits of ALU output
  assign uio_out = ALU_out[15:8];  // Upper 8 bits of ALU output
  assign uio_oe = 8'hFF;           // All bidirectional pins as outputs
  
  // Unused inputs (can be ignored or used later)
  wire _unused = &{ui_in, uio_in, ena, 1'b0};

endmodule

// Program Counter Module
module PC(
  input clk,
  input rst,
  input jump,
  input [15:0] jump_address,
  output wire [15:0] pc_out
);

  reg [15:0] p_c;
  wire [15:0] pc_next;

  assign pc_next = (jump) ? jump_address : (p_c + 2);

  always @(posedge clk or posedge rst) begin
    if (rst)
      p_c <= 16'd0;
    else if (p_c >= 16'd30)  // wrap at last instruction (15*2 = 30)
      p_c <= 16'd0;
    else
      p_c <= pc_next;
  end

  assign pc_out = p_c;

endmodule

// Instruction Memory Module
module instruction_memory(
  input [15:0] p_in,
  output wire [15:0] instruction
);
  reg [15:0] instruction1;
  reg [15:0] rom [0:15];  // Fixed array declaration
  
  initial begin
    rom[0]  = 16'b0000_0001_0010_0011;  // ADD
    rom[1]  = 16'b0001_0010_0011_0100;  // SUB
    rom[2]  = 16'b0010_0011_0100_0101;  // ADDI
    rom[3]  = 16'b0011_0100_0101_0011;  // LW
    rom[4]  = 16'b0100_0101_0100_0011;  // SW
    rom[5]  = 16'b0110_0100_0011_0011;  // XOR
    rom[6]  = 16'b0111_0000_0000_0011;  // OR
    rom[7]  = 16'b0010_0010_0010_1111;  // ADDI
    rom[8]  = 16'b0000_0001_0001_0001;  // ADD
    rom[9]  = 16'b0001_0011_0001_0000;  // SUB
    rom[10] = 16'b0011_0001_0010_0100;  // LW
    rom[11] = 16'b0100_0010_0011_0100;  // SW
    rom[12] = 16'b0010_0011_0000_0001;  // ADDI
    rom[13] = 16'b0001_0001_0000_0001;  // SUB
    rom[14] = 16'b0111_0000_0101_0100;  // OR
    rom[15] = 16'b0101_0000_0000_0000;  // JUMP (to 0)
  end
  
  always @(*) begin
    if ((p_in >> 1) < 16)
      instruction1 = rom[p_in >> 1];
    else
      instruction1 = 16'b0000_0000_0000_0000;
  end
  
  assign instruction = instruction1;
endmodule

// Decoder Module
module decode(
  input [15:0] instruction_in,
  output reg [3:0] rs, rt, opcode, rd, im,
  output reg [11:0] jump
);

  always @(*) begin
    // Default values
    rs = 4'b0000;
    rt = 4'b0000;
    rd = 4'b0000;
    im = 4'b0000;
    jump = 12'b000000000000;
    opcode = instruction_in[15:12];   

    case(opcode)
      4'b0000: begin  // ADD
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end

      4'b0001: begin  // SUB
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end

      4'b0010: begin  // ADDI
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        im = instruction_in[3:0];
      end

      4'b0011: begin  // LW
        rd = instruction_in[11:8]; // destination register to load into
        rs = instruction_in[7:4];  // base register
        im = instruction_in[3:0];  // offset
      end

      4'b0100: begin  // SW
        rt = instruction_in[11:8]; // source register to store FROM 
        rs = instruction_in[7:4];  // base register
        im = instruction_in[3:0];  // offset
      end

      4'b0101: begin  // JUMP
        jump = instruction_in[11:0];
      end

      4'b0110: begin  // XOR
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end

      4'b0111: begin  // OR
        rd = instruction_in[11:8];
        rs = instruction_in[7:4];
        rt = instruction_in[3:0];
      end

      default: begin
        rs = 4'b0000;
        rt = 4'b0000;
        rd = 4'b0000;
        im = 4'b0000;
        jump = 12'b000000000000;
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
    // Default values
    RegDst = 1'b0; ALUsrc = 1'b0; MemtoReg = 1'b0; RegWrite = 1'b0;
    MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0000;
    
    case (opcode)
      4'b0000: begin  // ADD
        RegDst = 1'b1; ALUsrc = 1'b0; MemtoReg = 1'b0; RegWrite = 1'b1;
        MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0000;
      end

      4'b0001: begin  // SUB
        RegDst = 1'b1; ALUsrc = 1'b0; MemtoReg = 1'b0; RegWrite = 1'b1;
        MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0001;
      end

      4'b0010: begin  // ADDI
        RegDst = 1'b0; ALUsrc = 1'b1; MemtoReg = 1'b0; RegWrite = 1'b1;
        MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0010;
      end

      4'b0011: begin  // LW
        RegDst = 1'b0; ALUsrc = 1'b1; MemtoReg = 1'b1; RegWrite = 1'b1;
        MemWrite = 1'b0; MemRead = 1'b1; jump = 1'b0; ALUOp = 4'b0011;
      end

      4'b0100: begin  // SW
        RegDst = 1'b0; ALUsrc = 1'b1; MemtoReg = 1'b0; RegWrite = 1'b0;
        MemWrite = 1'b1; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0100;
      end

      4'b0101: begin  // JUMP
        RegDst = 1'b0; ALUsrc = 1'b0; MemtoReg = 1'b0; RegWrite = 1'b0;
        MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b1; ALUOp = 4'b0000;
      end

      4'b0110: begin  // XOR
        RegDst = 1'b1; ALUsrc = 1'b0; MemtoReg = 1'b0; RegWrite = 1'b1;
        MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0110;
      end

      4'b0111: begin  // OR
        RegDst = 1'b1; ALUsrc = 1'b0; MemtoReg = 1'b0; RegWrite = 1'b1;
        MemWrite = 1'b0; MemRead = 1'b0; jump = 1'b0; ALUOp = 4'b0111;
      end
    endcase
  end

endmodule

// ALU Module
module ALU(
  input [15:0] A, B,
  input [3:0] ALUOp,
  output reg [15:0] ALU_out,
  output Zero
);

  always @(*) begin
    case(ALUOp)
      4'b0000: ALU_out = A + B;      // ADD
      4'b0001: ALU_out = A - B;      // SUB
      4'b0010: ALU_out = A + B;      // ADDI
      4'b0011: ALU_out = A + B;      // LW (address calculation)
      4'b0100: ALU_out = A + B;      // SW (address calculation)
      4'b0110: ALU_out = A ^ B;      // XOR
      4'b0111: ALU_out = A | B;      // OR
      default: ALU_out = 16'b0;
    endcase
  end

  assign Zero = (ALU_out == 16'b0);

endmodule

// Data Memory Module
module data_memory(
  input clk,
  input MemWrite, MemRead,
  input [15:0] address,
  input [15:0] write_data,
  output reg [15:0] read_data
);

  reg [15:0] mem [0:63]; // Fixed array declaration - 64 words of memory
  integer i;

  initial begin
    // Initialize memory with some test values
    mem[0] = 16'h1234;
    mem[1] = 16'h5678;
    mem[2] = 16'h9ABC;
    mem[3] = 16'hDEF0;
    // Initialize rest to 0
    for (i = 4; i < 64; i = i + 1) begin
      mem[i] = 16'h0000;
    end
  end

  always @(posedge clk) begin
    if (MemWrite && address < 64)
      mem[address] <= write_data;
  end

  always @(*) begin
    if (MemRead && address < 64)
      read_data = mem[address];
    else
      read_data = 16'b0;
  end

endmodule

// MIPS Single Cycle CPU Module
module mips_single_cycle(
  input clk,
  input rst,
  output [15:0] ALU_out,
  output [3:0] ALUOp,
  output [15:0] Read_data1,
  output [15:0] Read_data2,
  output [15:0] sign_ext_immediate,
  output [3:0] rs,
  output [3:0] rt,
  output [3:0] rd,
  output [3:0] Write_reg_out
);

  wire [15:0] instruction;
  wire [15:0] pc;
  wire RegDst, ALUsrc, MemtoReg, MemWrite, MemRead, RegWrite, jump;
  wire [15:0] write_data, mem_read_data, alu_input_b;
  wire [3:0] write_reg;
  wire [15:0] write_data_final;

  // Program Counter
  PC pc_inst(
    .clk(clk),
    .rst(rst),
    .jump(jump),
    .jump_address({4'b0000, instruction[11:0]}), // extend jump address to 16 bits
    .pc_out(pc)
  );

  // Instruction Memory
  instruction_memory imem(
    .p_in(pc),
    .instruction(instruction)
  );

  // Decode instruction
  wire [3:0] opcode, im;
  wire [11:0] jump_addr;
  decode dec(
    .instruction_in(instruction),
    .rs(rs),
    .rt(rt),
    .rd(rd),
    .opcode(opcode),
    .im(im),
    .jump(jump_addr)
  );

  // Control Unit
  control_unit cu(
    .opcode(opcode),
    .RegDst(RegDst),
    .ALUsrc(ALUsrc),
    .MemtoReg(MemtoReg),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .RegWrite(RegWrite),
    .jump(jump),
    .ALUOp(ALUOp)
  );

  // Register File
  reg [15:0] reg_file [0:15];  // Fixed array declaration
  integer j;
  
  // Initialize register file
  initial begin
    reg_file[0] = 16'h0000;  // $zero
    reg_file[1] = 16'h0001;  // $at
    reg_file[2] = 16'h0002;  // $v0
    reg_file[3] = 16'h0003;  // $v1
    reg_file[4] = 16'h0004;  // $a0
    reg_file[5] = 16'h0005;  // $a1
    for (j = 6; j < 16; j = j + 1) begin
      reg_file[j] = 16'h0000;
    end
  end

  // Register file read
  assign Read_data1 = reg_file[rs];
  assign Read_data2 = reg_file[rt];

  // Sign extend immediate
  assign sign_ext_immediate = {{12{im[3]}}, im};

  // ALU input B mux
  assign alu_input_b = ALUsrc ? sign_ext_immediate : Read_data2;

  // ALU
  ALU alu_inst(
    .A(Read_data1),
    .B(alu_input_b),
    .ALUOp(ALUOp),
    .ALU_out(ALU_out),
    .Zero()
  );

  // Data Memory
  data_memory dmem(
    .clk(clk),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .address(ALU_out[5:0]), // Use lower 6 bits for address
    .write_data(Read_data2),
    .read_data(mem_read_data)
  );

  // Write register mux
  assign write_reg = RegDst ? rd : rt;
  assign Write_reg_out = write_reg;

  // Write data mux
  assign write_data_final = MemtoReg ? mem_read_data : ALU_out;

  // Register file write
  integer k;
  always @(posedge clk) begin
    if (rst) begin
      reg_file[0] <= 16'h0000;  // $zero always 0
      reg_file[1] <= 16'h0001;
      reg_file[2] <= 16'h0002;
      reg_file[3] <= 16'h0003;
      reg_file[4] <= 16'h0004;
      reg_file[5] <= 16'h0005;
      for (k = 6; k < 16; k = k + 1) begin
        reg_file[k] <= 16'h0000;
      end
    end else if (RegWrite && write_reg != 4'b0000) begin // Don't write to $zero
      reg_file[write_reg] <= write_data_final;
    end
  end

endmodule