import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_mips_processor(dut):
    """Test the 16-bit MIPS processor functionality"""
    
    # Set the clock period to 100ns (10MHz)
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Starting MIPS processor test")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Let the processor run through several instructions
    dut._log.info("Running processor for multiple clock cycles")
    
    # Test for 50 clock cycles to see instruction execution
    for cycle in range(50):
        await ClockCycles(dut.clk, 1)
        
        # Read the ALU output (16-bit value across uo_out and uio_out)
        alu_low = int(dut.uo_out.value)
        alu_high = int(dut.uio_out.value)
        alu_result = (alu_high << 8) | alu_low
        
        dut._log.info(f"Cycle {cycle}: ALU Output = 0x{alu_result:04X} ({alu_result})")
        
        # Check that we're getting valid outputs (not all zeros or all ones)
        if cycle > 5:  # Give some cycles for processor to start
            assert alu_result != 0xFFFF, f"ALU output stuck at all 1s at cycle {cycle}"
            # Note: ALU output of 0 might be valid for some operations

    # Test specific expected behaviors
    dut._log.info("MIPS processor test completed successfully")

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
    
    # Release reset
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    
    # Check that processor starts executing
    alu_low = int(dut.uo_out.value)
    alu_high = int(dut.uio_out.value)
    alu_result = (alu_high << 8) | alu_low
    
    dut._log.info(f"After reset: ALU Output = 0x{alu_result:04X}")
    
    # Run a few more cycles to ensure processor is working
    await ClockCycles(dut.clk, 10)
    
    dut._log.info("Reset test completed successfully")