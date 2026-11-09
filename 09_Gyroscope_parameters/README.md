# Lab 09 — Gyroscope Sensor Parameters

## Lecture

**Topic:** Sensors for state estimation — proprioceptive vs. exteroceptive, sensor performance characteristics, inertial sensors (MEMS gyroscope, accelerometer), IMU/INS and why an INS drifts without aiding.

**Key ideas**

- A MEMS vibratory gyro measures angular **rate** ω through the Coriolis force acting on a driven proof mass. It never measures angle directly — heading is obtained by integrating ω, so every error in ω becomes a growing error in θ.
- Sensor model used in this lab (yaw axis) — every parameter you identify plugs into it, so the result is a gyro you can **simulate**:

  $$\omega_{meas}(t_k) = Q_{\Delta}\Big(\mathrm{sat}_{\pm\omega_{max}}\big[(1+s)\,\omega_{true}(t_k) + b_0 + d\,t_k + n_k\big]\Big), \quad n_k \sim \mathcal{N}(0,\sigma^2)$$

  | Symbol | Parameter | Section |
  |---|---|---|
  | $b_0$ | offset (bias) | §1 |
  | $d$ | bias drift rate | §2 |
  | $\omega_{max}$ | measurement range (saturation) | §3 |
  | $\Delta$ | resolution (quantisation step) | §4 |
  | $\sigma$ | noise standard deviation | §5 |
  | $s$ | scale-factor error | §6 |

- Integrating the model: a constant bias gives a heading error growing **linearly** in time ($\theta_{err} = b_0 t$); white noise gives an error growing as **√t**. This is the "unlimited drift" of an INS from the lecture.
- Error classes from the lecture: *systematic* (offset, scale factor — can be calibrated out) vs. *non-systematic* (noise — can only be characterised statistically and handled by a filter in Labs 11–12).

**Reading**

- Lecture slides: *16-311 Introduction to Robotics, Lecture 15 — Sensors (for State Estimation) 1*, G. A. Di Caro, CMU-Q (slides 3–19).
- Panasonic, *Basic Knowledge of Inertial Sensors — Gyro and Acceleration Sensors* — Coriolis principle and vibratory gyro mechanism: <https://industrial.panasonic.com/ww/ds/ss/technical/b14>
- O. Woodman, *An Introduction to Inertial Navigation*, Cambridge UCAM-CL-TR-696 — gyro error sources and how they propagate into heading: <https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf>
- VectorNav Inertial Navigation Primer — *IMU Specifications* (how to read a datasheet): <https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs>

---

## Goal

Identify the parameters of the TRIK controller's built-in gyroscope (yaw axis), build a simulated gyro from them, and use it in Lab 10 (gyro-based localization), Lab 11 (complementary filter) and Lab 12 (Kalman filter — σ becomes your **R**).

Every parameter must be reported **with units**, the **method** used, and the **raw log** it was computed from.

### TRIK gyroscope API

Docs: <https://help.trikset.com/trik/programming-code/object-brick/class-gyroscope> · runtime reference: <https://dl.trikset.com/docs/runtime/html/classtrikControl_1_1GyroSensorInterface.html>

| Call | Returns |
|---|---|
| `brick.gyroscope().readRawData()` | 3 angular rates, **uncalibrated** (units not documented — see §6) |
| `brick.gyroscope().read()` | 7 values: `[0..2]` rates in mdeg/s, `[3]` timestamp in µs, `[4..6]` tilt angles in mdeg — bias-corrected **only after** `calibrate()` |
| `brick.gyroscope().calibrate(ms)` | built-in bias estimation by averaging at rest (docs recommend 10–20 s); resets angles |
| `brick.gyroscope().getCalibrationValues()` | 6 values: `[0..2]` gyro bias, `[3..5]` mean accelerometer at rest |

> Use `readRawData()` for all experiments — otherwise you measure the residual after TRIK's own correction, not the sensor.
> On-robot code: JavaScript / pure Python only (no NumPy). Log to CSV on the robot, analyse offline in Colab.
> Before every static measurement: robot powered on and still for **2 min** (warm-up), on a rigid surface, motors off.
> **One 3-min static log covers §1, §2, §4 and §5.** Record it once and reuse it.

---

## Parameters to identify

### 1. Offset (bias) — $b_0$

**What:** the mean output when the robot is not rotating, in deg/s.

**How (proposed method — static averaging):**
1. Take the first **T = 30 s** of the static log.
2. $b_0 = \frac{1}{N}\sum_{k=1}^{N}\omega_k$. Report also the standard error $\sigma/\sqrt{N}$.
3. Recompute $b_0$ from the first 1, 2, 5, 10, 20, 30 s and plot $b_0(T)$. Where the curve flattens is your **recommended calibration time**.
4. Compare with `getCalibrationValues()[2]` after `calibrate(10000)`.

**On-robot version:** a running mean (Welford's algorithm — no array needed) so the robot can self-calibrate at start-up.

Links:
- MakersPortal, *Gyroscope and Accelerometer Calibration with Raspberry Pi* — static-offset calibration in Python: <https://makersportal.com/blog/calibration-of-an-inertial-measurement-unit-imu-with-raspberry-pi-part-ii>
- Lecture slide 7 — systematic vs. non-systematic errors.

---

### 2. Bias drift — $d$

**What:** how fast the offset itself changes during a run, in deg/s per min; and the resulting heading error, in deg/min.

**How:**
1. Use the full **3-min** static log.
2. Split the log into 30-s windows, compute the mean rate in each, fit a line → slope $d$.
3. Integrate (trapezoid rule with the **real timestamps**):
   - (a) raw rate → heading drift dominated by $b_0$, ~linear; report slope in deg/min;
   - (b) rate minus $b_0$ → residual drift; report heading error after 1, 2, 3 min.
4. Check: does (a)'s slope match $b_0 \cdot 60$?

Links:
- Lecture slides 15, 17 — INS drift tables and "unlimited drift".
- Analog Devices, *Gyro Mechanical Performance: The Most Important Parameter* — why real drift is worse than the datasheet: <https://www.analog.com/en/resources/technical-articles/gyro-mechanical-performance.html>

---

### 3. Measurement range — $\omega_{max}$

**What:** the maximum rate the sensor reports before clipping, in ±deg/s.

**How:** spin the robot in place at max motor power (opposite wheel commands), then faster by hand on a turntable / office chair. Plot the rate — saturation shows as a flat top. Report both + and − limits.

Links:
- Lecture slide 5 — dynamic range.
- Datasheet example for comparison (range, noise rows) — VectorNav VN-100: <https://www.vectornav.com/docs/default-source/product-brief/vn-100-product-brief.pdf>

---

### 4. Resolution — $\Delta$

**What:** the smallest change in output (quantisation step), in deg/s.

**How:** from the static log, sort the distinct raw values and take the smallest non-zero difference. Convert to deg/s with the factor from §6.

Links: Lecture slide 5 — resolution and A/D conversion.

---

### 5. Noise — $\sigma$

**What:** the random part of the output at rest, in deg/s.

**How** (3-min static log, $b_0$ subtracted):
- Standard deviation σ (deg/s). This is the measurement noise **R** for Lab 12 at your sample rate.
- Histogram with fitted Gaussian. Is the noise Gaussian? Are there outliers or spikes?

Links:
- Lecture slide 7 — precision, random errors.
- VectorNav primer (above) — how σ relates to datasheet noise specs.

---

### 6. Scale factor — $s$ *(added)*

**What:** the gain of the sensor: the raw → deg/s conversion and the scale-factor error $s$ (how much the gyro over- or under-reports a real rotation). The sign of positive rotation is checked here too.

**How:**
1. **Sign:** rotate the robot slowly counter-clockwise (seen from above). Is the z-rate positive? It must match the heading convention in your odometry. Recall the encoder sign bug in `trajectory_blocks.js`.
2. **Raw → deg/s:** log `readRawData()` and `read()[2]` together during motion. After bias removal, the ratio gives the conversion factor.
3. **Scale-factor error:** rotate the robot exactly **N = 3 full turns** (3 × 360°) by hand against a reference mark on the floor or a turntable, CCW then CW. Integrate the bias-corrected rate:
   $$s = \frac{\theta_{integrated}}{\theta_{true}} - 1$$
   Report $s$ in % for CW and CCW separately. Asymmetry is common.
4. Repeat at a second speed to check that $s$ does not depend on speed.

Links:
- Lecture slide 6 — sensitivity and the accuracy formula.
- Woodman TR-696 (above) — calibration errors.

---

## Deliverables

Submit to `CS413 - Robotics/Lab9/`:

1. **Logger program** (`.js` / `.qrs`). It logs `readRawData()` + timestamp to CSV and runs the on-robot running-mean offset calibration.
2. **Raw logs** (CSV): static 3 min, saturation spin, 3-turn scale-factor runs (CW and CCW).
3. **Colab notebook** with the analysis and plots: $b_0(T)$, drift curves, saturation plot, noise histogram, scale-factor runs.
4. **Simulated gyro.** A Python function `gyro_sim(omega_true, t)` implementing the model from the Lecture section with your parameters. Feed it a recorded rotation and overlay the simulated output on the real log.
5. **Parameter table:**

| Parameter | Symbol | Value | Unit | Method |
|---|---|---|---|---|
| Offset (bias) | $b_0$ | | deg/s | static mean, T = … |
| Bias drift | $d$ | | deg/s/min | 30-s window means |
| Heading drift, uncorrected | | | deg/min | slope of θ(t) |
| Measurement range | ±$\omega_{max}$ | | deg/s | saturation test |
| Resolution | $\Delta$ | | deg/s | quantisation step |
| Noise std | $\sigma$ | | deg/s | static, 3 min |
| Raw → deg/s factor | | | — | §6 |
| Scale-factor error CCW / CW | $s$ | | % | 3 turns |
| Positive rotation direction | | CCW / CW | — | §6 |

6. **Short analysis** (≤ 1 page). Which error source dominates heading error after 10 s, 1 min and 3 min of driving? Is TRIK's built-in `calibrate()` good enough, and how long should it run?
