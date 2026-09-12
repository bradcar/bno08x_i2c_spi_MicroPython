# Offline timestamp regression tests

Run from the repository root with CPython 3.8 or later; no packages or hardware:

```sh
python tests/test_timestamps.py
```

To verify the regressions against a trusted earlier driver checkout:

```sh
python tests/test_timestamps.py --source /path/to/old/lib/bno08x.py
```

The tests execute the actual `update_sensors`, `_process_report` and
`_process_control_report` bodies, extracted with Python AST. Native decorators
are removed; transport, the non-wrapping clock and `uctypes` sensor-field access
are replaced with small fixtures. All packet bytes are synthetic. This does
not import the full driver or initialize GPIO, serial ports or a sensor.

The contract comes from the [Hillcrest SH-2 Reference Manual v1.2, sections
7.2.1–7.2.2](https://cdn.sparkfun.com/assets/4/d/9/3/8/SH-2-Reference-Manual-v1.2.pdf#page=81).
In milliseconds it is `host_interrupt - signed_base * 0.1 + sum(signed_rebases) * 0.1 + report_delay * 0.1`,
relative to the driver's host epoch. Its example expects 3500ms from a 5000ms
interrupt, 40000 base ticks, +15000 rebase ticks and 10000 report-delay ticks.
The [CEVA reference parser](https://github.com/ceva-dsp/sh2/blob/b514b1e2586ddc195e553dac89fc94c637b25298/sh2.c#L644)
uses the same signed, cumulative calculation.

Coverage includes positive/zero controls, negative base and rebase values,
multiple rebases, replacement bases, reports on each side of a rebase, and a
second packet. Both normal report parsing and the cached packet fast path are
exercised; sensor values and unread counts are checked alongside timestamps.
A synthetic interrupt arriving during the rebase handler verifies that reports
in progress retain the packet's original host time anchor.

These tests do not validate MicroPython native compilation, hardware clock
wrapping, transport framing, precision on a device, physical heading, drift,
calibration, or navigation performance. The reserved base value `0x7FFFFFFF`
is outside these valid-delta cases; this patch does not define a new policy for
that overflow sentinel. Bench validation remains separate.
