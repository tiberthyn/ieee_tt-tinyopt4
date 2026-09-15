import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


# ============================================================
#  Helpers
# ============================================================

async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def load_x(dut, x_val):
    """Shift a new sample x[n] into the DUT delay line (2 cycles)."""
    dut.ui_in.value = x_val & 0xFF
    dut.uio_in.value = 1          # start=1, data_sel=0, cfg_sel=0
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0
    await RisingEdge(dut.clk)


async def load_d_and_start(dut, d_val):
    """Latch d[n] and trigger the 11-state processing path (1 cycle)."""
    dut.ui_in.value = d_val & 0xFF
    dut.uio_in.value = 3          # start=1, data_sel=1, cfg_sel=0
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0


async def configure_mu(dut, mu_s):
    """Write the learning-rate exponent s (mu = 2^-s) via cfg_sel (2 cycles)."""
    dut.ui_in.value = mu_s & 0x07
    dut.uio_in.value = 5          # start=1, data_sel=0, cfg_sel=1
    await RisingEdge(dut.clk)
    dut.uio_in.value = 0
    await RisingEdge(dut.clk)


async def read_output(dut, sel):
    """Read uo_out using ui_in[2:0] as the register selector."""
    dut.ui_in.value = sel & 0x07
    await ClockCycles(dut.clk, 1)
    val = int(dut.uo_out.value)
    return val - 256 if val > 127 else val


def _sat_q17(v):
    """Saturate an integer to signed 8-bit Q1.7 range [-128, +127]."""
    return max(-128, min(127, v))


# ============================================================
#  Test 1 — Smoke test (FSM handshake + datapath single iteration)
# ============================================================

@cocotb.test()
async def test_project(dut):
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)
    assert int(dut.uio_out.value) & 0x10 == 0, "Busy should not be active after reset"

    await load_x(dut, 0)
    await load_x(dut, -5 & 0xFF)
    await load_x(dut, 20)
    await load_x(dut, 10)

    await load_d_and_start(dut, 50)

    await RisingEdge(dut.clk)
    busy_flag = (int(dut.uio_out.value) >> 4) & 1
    assert busy_flag == 1, "The FSM should be in Busy state"

    for _ in range(9):
        await RisingEdge(dut.clk)

    uio_val = int(dut.uio_out.value)
    done_flag = (uio_val >> 5) & 1
    busy_flag = (uio_val >> 4) & 1

    assert done_flag == 1, "The FSM did not issue the DONE pulse at cycle 10"
    assert busy_flag == 0, "The FSM did not lower the BUSY flag in S_DONE"

    await RisingEdge(dut.clk)

    y_hat = await read_output(dut, 0)
    e = await read_output(dut, 1)
    w0 = await read_output(dut, 2)

    assert y_hat == 0, f"Error: expected y_hat 0, got {y_hat}"
    assert e == 50, f"Error: expected e 50, got {e}"
    assert w0 == 0, f"Error: expected w0 0, got {w0}"

    dut._log.info("FSM and datapath test completed successfully.")


# ============================================================
#  Test 2 — 500-sample closed-loop convergence test
# ============================================================

@cocotb.test()
async def test_convergence(dut):
    """500-sample system identification against W* = [64, -32, 16, -8]."""
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Configure learning rate: mu = 2^-3  ->  shift = 7 + 3 = 10
    await configure_mu(dut, 3)

    W_star = [64, -32, 16, -8]
    random.seed(0xC0FFEE)

    # x_hist[0]=x[n-1], x_hist[1]=x[n-2], x_hist[2]=x[n-3], x_hist[3]=x[n-4]
    x_hist = [0, 0, 0, 0]

    for _ in range(1500):
        x_new = random.randint(-64, 63)

        # Reference signal produced by the target plant.
        # After load_x, the DUT delay line becomes
        # [x_new, x_hist[0], x_hist[1], x_hist[2]].
        d_raw = (W_star[0] * x_new +
                 W_star[1] * x_hist[0] +
                 W_star[2] * x_hist[1] +
                 W_star[3] * x_hist[2]) >> 7
        d_val = _sat_q17(d_raw)

        # Feed sample and reference to the DUT, then trigger the FSM.
        await load_x(dut, x_new & 0xFF)
        await load_d_and_start(dut, d_val & 0xFF)

        # FSM needs 11 edges total (1 trigger + 10 processing).
        # Wait 12 to be safe and to allow return to S_IDLE.
        await ClockCycles(dut.clk, 12)

        # Update the Python-side shadow of the delay line.
        x_hist = [x_new, x_hist[0], x_hist[1], x_hist[2]]

    # Read back the four weights from the DUT.
    w0 = await read_output(dut, 2)
    w1 = await read_output(dut, 3)
    w2 = await read_output(dut, 4)
    w3 = await read_output(dut, 5)
    w_hw = [w0, w1, w2, w3]

    dut._log.info(f"Converged weights: w0={w0}, w1={w1}, w2={w2}, w3={w3}")
    dut._log.info(f"Target weights:    w0={W_star[0]}, w1={W_star[1]}, w2={W_star[2]}, w3={W_star[3]}")

    for i, (hw, star) in enumerate(zip(w_hw, W_star)):
        diff = abs(hw - star)
        assert diff <= 10, (
            f"w{i}: hardware={hw}, target={star}, |diff|={diff} > 10 LSB"
        )

    dut._log.info("500-sample convergence test passed.")