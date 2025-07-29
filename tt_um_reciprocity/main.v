# Tiny Tapeout project information
project:
  title:        "MIPS16 Single Cycle Processor"      # Project title
  author:       "Kishore Netheti"                    # Your name
  discord:      ""                                   # Your discord username, for communication and automatically assigning you a Tapeout role (optional)
  description:  "A 16-bit MIPS single-cycle processor with 8 instructions: ADD, SUB, ADDI, LW, SW, XOR, OR, JUMP"      # Short description of what your project does
  language:     "Verilog"                           # other examples include SystemVerilog, Amaranth, VHDL, etc
  clock:        "10000"                             # Clock frequency in Hz (or 0 if not applicable)

# Keep a track of the submission yaml
yaml_version: 6

# As everyone will have access to all designs, try to make it easy for someone new to your design to know what
# it does and how to operate it. This info will be automatically collected and used to make a datasheet for the chip.
documentation:
  inputs:
    - none (processor runs automatically)
    - none
    - none
    - none
    - none
    - none
    - none
    - none
  outputs:
    - ALU output bit 0
    - ALU output bit 1
    - ALU output bit 2
    - ALU output bit 3
    - ALU output bit 4
    - ALU output bit 5
    - ALU output bit 6
    - ALU output bit 7
  bidirectional:
    - ALU operation bit 0
    - ALU operation bit 1
    - ALU operation bit 2
    - ALU operation bit 3
    - source register bit 0
    - source register bit 1
    - target register bit 0
    - target register bit 1

# The following fields are optional
  tag:          "processor, mips, cpu"      # comma separated list of tags: test, encryption, experiment, clock, animation, utility, industrial, pwm, fpga, alu, microprocessor, risc, ethernet, flash, fft, filter, music, bcd, sound, serial, timer, random, maze, fpga, cpu, gui, game
  external_hw:  ""                          # Describe any external hardware needed
  discord:      ""                          # Your discord handle, used for communication and automatically assigning tapeout role after a submission
  doc_link:     ""                          # URL to longer form documentation, eg the README.md in your repository
  clock_hz:     10000                       # Clock frequency in Hz (or 0 if not applicable)

  # How many tiles your design occupies? A single tile is about 167x108 uM.
  tiles: "1x1"    # Valid values: 1x1, 1x2, 2x2, 3x2, 4x2 or 8x2

# Do not change!
pinout:
  # Inputs
  ui[0]: ""
  ui[1]: ""
  ui[2]: ""
  ui[3]: ""
  ui[4]: ""
  ui[5]: ""
  ui[6]: ""
  ui[7]: ""

  # Outputs
  uo[0]: "ALU_out[0]"
  uo[1]: "ALU_out[1]"
  uo[2]: "ALU_out[2]"
  uo[3]: "ALU_out[3]"
  uo[4]: "ALU_out[4]"
  uo[5]: "ALU_out[5]"
  uo[6]: "ALU_out[6]"
  uo[7]: "ALU_out[7]"

  # Bidirectional pins
  uio[0]: "ALUOp[0]"
  uio[1]: "ALUOp[1]"
  uio[2]: "ALUOp[2]"
  uio[3]: "ALUOp[3]"
  uio[4]: "rs[0]"
  uio[5]: "rs[1]"
  uio[6]: "rt[0]"
  uio[7]: "rt[1]"

# Do not change!
yaml_version: 6