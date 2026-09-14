# TinyOpt-4 LMS Engine

## How it works

TinyOpt-4 is a sequential fixed-point adaptive accelerator designed to execute an iterative LMS (Least Mean Squares) update rule for 4 FIR filter coefficients.

1. Internal delay line stores the last 4 inputs: x[n], x[n-1], x[n-2], x[n-3].
2. A shared 8x8 signed multiplier and 18-bit accumulator compute the prediction y_hat[n].
3. The error is computed as e[n] = d[n] - y_hat[n] with saturation.
4. Each weight is sequentially updated using the gradient:
   w_i[n+1] = saturate(w_i[n] + (e[n] * x[n-i] >> (7 + mu_shift)))
5. When idle, the internal registers (y_hat, e, w0-w3) are exposed via the uo_out bus.

## How to test

1. Apply reset (`rst_n = 0`) for a few clock cycles and release it (`rst_n = 1`).
2. Set `uio_in = 3'b001` (start=1, data_sel=0, cfg_sel=0) and pass four values on `ui_in` to load the delay line.
3. Set `uio_in = 3'b011` (start=1, data_sel=1, cfg_sel=0) and pass `d[n]` on `ui_in`. This triggers the 10-cycle FSM.
4. Wait until `uio_out[4]` (busy) goes low and `uio_out[5]` (done) pulses high.
5. Set `uio_in = 0` and inspect `uo_out` by selecting the register with `ui_in[2:0]`:
   - `000`: y_hat
   - `001`: error
   - `010`-`101`: weights w0 to w3.