import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

async def load_x(dut, x_val):
    dut.ui_in.value = x_val & 0xFF
    dut.uio_in.value = 1 
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0
    await RisingEdge(dut.clk)

async def load_d_and_start(dut, d_val):
    dut.ui_in.value = d_val & 0xFF
    dut.uio_in.value = 3
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0

async def read_output(dut, sel):
    dut.ui_in.value = sel & 0x07
    await ClockCycles(dut.clk, 1)
    val = int(dut.uo_out.value)
    return val - 256 if val > 127 else val

@cocotb.test()
async def test_project(dut):
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)
    assert int(dut.uio_out.value) & 0x10 == 0, "Busy no debería estar activo tras el reset"

    await load_x(dut, 0)
    await load_x(dut, -5 & 0xFF)
    await load_x(dut, 20)
    await load_x(dut, 10)

    await load_d_and_start(dut, 50)
    
    await RisingEdge(dut.clk)
    busy_flag = (int(dut.uio_out.value) >> 4) & 1
    assert busy_flag == 1, "La FSM debería estar en estado Busy"
    
    for _ in range(9):
        await RisingEdge(dut.clk)
        
    uio_val = int(dut.uio_out.value)
    done_flag = (uio_val >> 5) & 1
    busy_flag = (uio_val >> 4) & 1
    
    assert done_flag == 1, "La FSM no emitió el pulso DONE en el ciclo 10"
    assert busy_flag == 0, "La FSM no bajó el flag BUSY en S_DONE"

    await RisingEdge(dut.clk)
    
    y_hat = await read_output(dut, 0)
    e = await read_output(dut, 1)
    w0 = await read_output(dut, 2)
    
    assert y_hat == 0, f"Error: y_hat esperado 0, obtenido {y_hat}"
    assert e == 50, f"Error: e esperado 50, obtenido {e}"
    assert w0 == 0, f"Error: w0 esperado 0, obtenido {w0}"
    
    dut._log.info("Prueba de FSM y datapath completada con éxito.")