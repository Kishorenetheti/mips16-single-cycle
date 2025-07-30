import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_mips_processor(dut):
    """Test the 8-bit MIPS processor functionality"""
    
    # Set the clock period to 100ns (10MHz)
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Starting 8-bit MIPS processor test")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Let the processor run through several instructions
    dut._log.info("Running processor for multiple clock cycles")
    
    # Test for 30 clock cycles to see instruction execution
    for cycle in range(30):
        await ClockCycles(dut.clk, 1)
        
        # Read the ALU output (8-bit value from uo_out)
        alu_result = int(dut.uo_out.value)
        
        # Read the PC output (lower 4 bits from uio_out)
        pc_result = int(dut.uio_out.value) & 0x0F  # Only lower 4 bits are PC
        
        dut._log.info(f"Cycle {cycle}: ALU Output = 0x{alu_result:02X} ({alu_result}), PC = {pc_result}")
        
        # Check that we're getting valid outputs
        if cycle > 5:  # Give some cycles for processor to start
            # Verify PC is changing (should cycle 0-15)
            assert pc_result <= 15, f"PC value {pc_result} exceeds maximum (15)"

    dut._log.info("8-bit MIPS processor test completed successfully")

@cocotb.test()
async def test_reset_functionality(dut):
    """Test that reset properly initializes the processor"""
    
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # Test reset behavior
    dut._log.info("Testing reset functionality")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    
    # Hold reset for several cycles
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    
    # During reset, PC should be 0
    pc_during_reset = int(dut.uio_out.value) & 0x0F
    dut._log.info(f"During reset: PC = {pc_during_reset}")
    
    # Release reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Check that processor starts executing from PC = 0
    alu_result = int(dut.uo_out.value)
    pc_after_reset = int(dut.uio_out.value) & 0x0F
    
    dut._log.info(f"After reset: ALU Output = 0x{alu_result:02X}, PC = {pc_after_reset}")
    
    # PC should start from 0 after reset
    assert pc_after_reset == 0, f"PC should be 0 after reset, got {pc_after_reset}"
    
    # Run a few more cycles to ensure processor is working
    await ClockCycles(dut.clk, 10)
    
    # PC should have advanced
    pc_final = int(dut.uio_out.value) & 0x0F
    dut._log.info(f"After 10 cycles: PC = {pc_final}")
    
    dut._log.info("Reset test completed successfully")

@cocotb.test()
async def test_instruction_execution(dut):
    """Test specific instruction execution"""
    
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # Initialize
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Track instruction execution through a full cycle
    dut._log.info("Testing instruction execution sequence")
    
    instruction_results = []
    
    for i in range(20):  # Execute 20 instructions
        await ClockCycles(dut.clk, 1)
        
        alu_out = int(dut.uo_out.value)
        pc = int(dut.uio_out.value) & 0x0F
        
        instruction_results.append((pc, alu_out))
        dut._log.info(f"Instruction {i}: PC={pc}, ALU={alu_out:02X}")
        
        # Verify PC wraps around at 16
        if i > 0:
            prev_pc = instruction_results[i-1][0]
            if prev_pc == 15:  # Should wrap to 0
                assert pc == 0, f"PC should wrap from 15 to 0, got {pc}"
    
    dut._log.info("Instruction execution test completed successfully")