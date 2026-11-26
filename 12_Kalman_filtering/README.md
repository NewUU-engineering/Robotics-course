# Kalman Filter Tutorial: Odometry + Gyroscope on a TRIK Robot

A TRIK robot estimates its pose with an Extended Kalman Filter (EKF) that fuses the wheel encoders and the gyroscope.
It drives **the last two digits of the student's ID** as seven-segment digits on a floor marker grid.
Deliberate **interventions** then break the filter's assumptions one at a time, following the process-noise examples of
[kalmanfilter.net, "1D Kalman Filter with process noise"](https://kalmanfilter.net/kalman1d_pn.html).

## 1. Robot and tools

| Item | Value |
|---|---|
| Motors / encoders | Right `M3` / `E3`; left `M4` / `E4` |
| Geometry | `r = 0.028 m` (wheel radius), `B = 0.154 m` (track width) |
| Gyroscope | Yaw rate in mdeg/s |
| Robot program | Written by students in the TRIK Studio visual language |
| `kf_replay.py` | Offline replay of a log: trajectories, innovation, P22 and gain plots |

## 2. Theory

```text
state         x = [x, y, θ]ᵀ                 pose in the start frame
input         Δs = r·(Δφ_r + Δφ_l)/2,  Δθ = r·(Δφ_r − Δφ_l)/B          (encoders)
measurement   z  = Σ (ω_gyro − b)·Δt                                  (gyro heading)

motion        x⁺ = x + Δs·cos θ,   y⁺ = y + Δs·sin θ,   θ⁺ = θ + Δθ
              z  = θ + v

F = | 1  0  −Δs·sin θ |     G = | cos θ  0 |     H = | 0  0  1 |
    | 0  1   Δs·cos θ |         | sin θ  0 |
    | 0  0   1        |         | 0      1 |

predict:  x⁻ = f(x, u)                P⁻ = F·P·Fᵀ + Q,   Q = G·Σ_u·Gᵀ
update:   ν = wrap(z − θ⁻)            S  = P22⁻ + R
          K = [P02⁻, P12⁻, P22⁻]ᵀ / S
          x = x⁻ + K·ν                P  = (I − K·H)·P⁻
```

Because `H = [0 0 1]`, only the third column of P is needed:

```text
P02⁻ = P02 − Δs·sin θ·P22 + cos θ·σ_sθ        P02 = P02⁻ − P02⁻·P22⁻/S
P12⁻ = P12 + Δs·cos θ·P22 + sin θ·σ_sθ        P12 = P12⁻ − P12⁻·P22⁻/S
P22⁻ = P22 + σ_θ²                             P22 = P22⁻ − P22⁻·P22⁻/S
```

**Consistency check.** If Q and R are right, about 95 % of the innovations satisfy

```text
|ν| ≤ 2·sqrt(S)
```

## 3. Baseline Q, R and P0

```text
Q  (wheel slip):    σ_θ² = k²·(|d_r| + |d_l|) / B²,   σ_sθ = k²·(|d_r| − |d_l|) / (2·B),   k² = 5e-5 m
R  (gyro drift):    R(t) = N²·t + σ_b²·t²,             N² = 3.0e-8 rad²/s,   σ_b² = 7.6e-7 rad²/s²
P0 (placement):     P02 = P12 = 0,   P22 = 3e-4 rad²
```

| Value | Motivation |
|---|---|
| `k²` | Rubber wheels on a smooth floor: about 7 mm of random distance error per wheel per metre. Q grows with wheel travel and is zero at rest. |
| `N`, `σ_b` | Typical MEMS gyro, with 0.05 °/s of bias left after averaging 2 s at rest. The integrated error grows with time, so R grows too. |
| `P22 = 3e-4` | The heading is aligned on the start marker within about 1°. |

## 4. Field and digit routes

The 12 markers form two digit cells 0.5 × 1.0 m with a 0.2 m gap between them:

```text
 y=1.0  E ── a ── F     E ── a ── F
        f         b     f         b
 y=0.5  D ── g ── C     D ── g ── C
        e         c     e         c
 y=0.0  A ── d ── B     A ── d ── B
       x=0       0.5   0.7       1.2
       └ cell 1 (tens) ┘└ cell 2 (units) ┘
```

- **Start:** the robot's axle centre on A of cell 1, at (0, 0), facing +x.
- **Cell 2** has the same nodes shifted by +0.7 m in x.
- **Rows:** measure the row spacing (assumed 0.5 m) before the first run.

## 5. Program requirements

Students build the program in the TRIK Studio visual language. It must:

1. **Turn the ID into waypoints.** Drive the tens digit in cell 1 and the units digit in cell 2, following the table in Section 4.
2. **Calibrate the gyro.** Stand still for 2 s, average the gyro rate, and subtract it as the bias.
3. **Run the EKF from Section 2.** Steer to each waypoint until the robot is within 5 cm of it.
4. **Keep all filter settings in one initialisation block,** so each intervention changes only variables:

   | Variable | Meaning |
   |---|---|
   | `k2` or a constant `q` | Process noise |
   | `R` mode and values | Constant `R`, or `N²`, `σ_b²` for growing R |
   | `P22_0` | Initial heading variance |
   | `theta0` | Initial heading estimate |
   | Bias switch, `extra_bias` | Bias calibration on/off; artificial bias added to the gyro |
   | `pause_node` | The node where the robot stops for 5 s |

5. **Log one line per step:** `theta_l, theta_r, gyro, t, x, y, course, P22, innov`.
6. **Stop the motors** at the end.

## 6. Interventions

First drive the **baseline**: the values of Section 3, 2 runs. Then apply one intervention at a time.

Each intervention imitates one situation from kalmanfilter.net. Interventions marked **replay** can be applied to a baseline log, without driving again.

| # | Intervention | kalmanfilter.net analogue | How to apply | Expected result |
|---|---|---|---|---|
| T1 | **Wrong initial guess** | Example 6: initial guess 60 °C, true 50 °C | `theta0 = 10°`, constant `R = 7e-4`. Compare `P22_0 = 1e-6` (overconfident) with `P22_0 = 3e-2` (σ ≈ 10°). Robot or replay. | Large P22_0: the gain starts near 1 and the error vanishes in a few steps. Small P22_0: slow convergence, and the first leg is driven skewed. |
| T2 | **Wrong motion model** | Example 7: the liquid heats, but the model says constant | Wrap 3 layers of electrical tape (≈ 0.45 mm) around the right wheel, and keep `r` unchanged. Odometry then reports a false turn of about 6°/m. Run `k2 = 5e-7`, `5e-5` and `5e-3`. Robot only. | Small q: **lag**. The estimate follows the wrong odometry, the path bends, and innovations sit outside ±2·sqrt(S), so the filter is overconfident. |
| T2b | **Larger process noise** | Example 8: q raised to 0.15 | Same as T2 with `k2 = 5e-3`. | No lag: the estimate follows the gyro and the path is right again, but the heading is noisier because there is less smoothing. |
| T3 | **Biased measurement** | Model mismatch on the measurement side | Switch off bias calibration and set `extra_bias = 0.5 °/s`. Compare `R = 2.5e-5`, `R = 7e-4` and the growing R. Robot or replay. | The heading drifts about 30° per minute with every R. Tuning R cannot remove a bias, because the filter treats the gyro heading as the truth in the long run. |
| T4 | **Sudden manoeuvre** | Unpredictable change, which calls for larger q | While the robot pauses at `pause_node`, lift it and rotate it about 20° by hand. The gyro sees the rotation; the encoders do not. Compare `k2 = 5e-7` with constant `q = 4e-4`. Robot only. | Small q: the innovation jumps far beyond ±2·sqrt(S) and recovery is slow. Constant q: recovery within a few steps. Travel-scaled Q adds nothing at rest, which is exactly the wrong assumption here. |
| T5 | **Sensor dropout** | Prediction without updates | Switch off the gyro update for 20 s in the middle of the route. Replay. | P22 grows by Q at every step. When updates resume, a large innovation is followed by a jump in K and a fast correction. |

**What to record for every run:**

- The final-pose error against the last marker (ruler for position, a tape strip on the chassis for heading).
- The share of innovations inside ±2·sqrt(S).
- The plots of P22 and K over time.

## 7. Offline replay

`kf_replay.py` reads the first four log columns and plots two things:

- **The trajectories** of odometry-only, gyro-only and EKF estimates, over the route and the markers.
- **The innovation** with its ±2·sqrt(S) bound, P22 and the gain K over time. It also prints the share of innovations inside the bound.

| Option | Use |
|---|---|
| `--id` | Student ID; draws the reference route |
| `--q`, `--r`, `--p22` | Constant Q, constant R, initial P22 |
| `--k2`, `--n2`, `--sb2` | Travel-scaled Q and growing R (baseline) |
| `--theta0` | T1: initial heading estimate, deg |
| `--add-bias` | T3: artificial gyro bias, mdeg/s |
| `--dropout T1 T2` | T5: no gyro update between T1 and T2 seconds |

## 8. Report and questions

**Report:** for the baseline and each intervention, the route plot, the final-pose error, the share of innovations inside the bound, and one sentence explaining the result.

**Questions:**

1. **T2 and T2b:** larger q removed the lag. Why is recalibrating the right wheel radius still the better fix? Compare with the conclusion of kalmanfilter.net.
2. **T3:** why can no value of R fix a biased gyro? Which extra state would?
3. **T4:** travel-scaled Q is physically right for slip but fails for a manual rotation. How would you design Q for both?
4. **T1:** what does P0 express, and what happens when it is too small?
5. Which digits make odometry errors largest, and which make gyro errors largest? Consider the number of turns versus the run time.
