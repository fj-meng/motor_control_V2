# PS CAN0 MIO10 Test

This is the first bench test for the AX7Z010B and the H10010 motor. It uses
the Zynq PS CAN0 peripheral, not a PL CAN IP.

## Source layout

- `can0_config.h` contains the CAN clock timing, IDs, frame size, and safety
  switch.
- `can0_driver.h` contains the public CAN function declarations.
- `can0_driver.c` contains all CAN function definitions and the receive loop.
- `main.c` contains only the application entry point and startup flow.

Add these four files to the SDK application's `src` directory. Do not add the
old `can0_ps_mio10_sdk2017_test.c` or `can0_ps_mio10_sdk2017_test.h` files if
they are still present in an older SDK workspace. Also do not add
`can0_ps_mio10_sdk2017_test2.c`, because it contains another `main()`.

## Vivado 2017.4

1. Open the Zynq-7000 Processing System configuration.
2. Enable CAN 0.
3. Keep CAN0 on the MIO mapping used by the board:
   - `PS_MIO10` = CAN0 RX
   - `PS_MIO11` = CAN0 TX
4. Enable UART1 and keep the board mapping:
   - `PS_MIO48` = UART1 TX
   - `PS_MIO49` = UART1 RX
5. Confirm the PS CAN clock. The header assumes the CAN CLK shown in the
   Zynq PS configuration is 100 MHz. It uses BRPR=4, actual TSEG1=16 TQ,
   actual TSEG2=3 TQ, and SJW=1 TQ, giving exactly 1 Mbps with an 85% sample
   point. If the clock differs, recalculate all four timing constants.
6. Generate the bitstream and export the hardware to SDK.

## SDK 2017.4

1. Create a Standalone BSP and application project.
2. Set `ps7_uart_1` as `stdin` and `stdout`, at 115200 baud.
3. Add `can0_config.h`, `can0_driver.h`, `can0_driver.c`, and `main.c` to the
   application. Only `main.c` provides `main()`.
4. Build and run with the motor powered and the CAN bus correctly terminated.

The initial build is receive-only. It prints the raw PS CAN identifier, DLC,
the PS CAN hardware receive timestamp, and up to eight data bytes. The
timestamp is the low 16 bits returned in `RxFrame[1]`; it is not the time of
the SDK interrupt. It wraps, so a later version should extend it to 32 or 64
bits in software.

The timing calculation is
`100 MHz / ((4 + 1) * (1 + (15 + 1) + (2 + 1))) = 1 Mbit/s`.
The values passed to `XCanPs_SetBitTiming` are the controller's encoded
segment values, so the actual segments are one count larger.

`SEND_SAFE_PROBE` is deliberately `0`. The optional probe frame is included
for a later controlled test, but the motor's stored operating mode determines
how a standard CAN command is interpreted. Do not enable it until the rotor is
secured and the no-load test procedure is ready.

## Expected first result

The UART should first show `CAN0 ready`. It will then print `RX` lines when a
CAN frame arrives. A response from the motor may use host ID 100, so inspect
the printed raw identifier rather than assuming it is 1.

If no frame is received, check these in order: motor power, CAN_H/CAN_L
polarity, common GND, exactly two 120 ohm terminations, CAN bitrate, motor ID,
and whether the motor is in CAN2.0 rather than CANFD mode.
