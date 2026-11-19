# AKP over simulated ELRS — 2026-11-19

Scope: the Python/AKP tunnel, simulated CRSF/ELRS byte link, AKP device
responder, RC/telemetry multiplexing and firmware-over-radio flight. No
hardware, serial port or RF transmitter was used.

## Command

```bash
ROS_DOMAIN_ID=61 AKP_SIM=true FLIGHT_CONTROLLER=firmware \
  scripts/run_radio_sil_mission.sh fpv_quad_5in <loss> 8.0 150.0 <tag>
```

The AKP probe queries `HELLO`, `STATUS` and parameter index 0 through
`AkClient(CrsfAkpTunnel(...))`. RC continues at 150 Hz; MSP envelopes use a
separate 40 Hz simulated multiplex lane.

## Results

| run | mission result | goal error | AKP result | AKP latencies |
| --- | --- | ---: | --- | --- |
| 0 % loss | landed and disarmed | 0.32 m | hello/status/parameter OK | 71.0 / 70.1 / 69.6 ms |
| 10 % loss | landed and disarmed | 0.32 m | hello/status/parameter OK, 7 total attempts | 70.3 / 71.4 / 71.3 ms |
| link cut | failsafe, receiver disabled, landed | - | AKP completed before cut | 72.5 / 70.5 / 70.1 ms |

Evidence:

- `radio-sil-akp-firmware-loss0-20261118-220638.{json,akp.json,log}`
- `radio-sil-akp-firmware-loss10-20261118-221123.{json,akp.json,log}`
- `radio-sil-akp-firmware-link-loss-20261118-221607.{json,akp.json,log}`

The link-cut probe recorded the cut at 2.00 m, receiver disable, and final
altitude 0.02 m. `last_positions_m` in the successful mission JSONs show
liftoff/landing resets from the MuJoCo plant; the flight itself reached within
0.32 m of the goal and ended disarmed.
