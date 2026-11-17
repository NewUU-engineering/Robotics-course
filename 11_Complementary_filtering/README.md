## 1. Lecture

### Two sensors, one course

In Week 10 the robot drove the same trajectory twice, once with the course from the encoders and once with the course from the gyro. Each sensor fails in its own way:

| Source | Good at | Bad at |
|---|---|---|
| `course_odom` (encoders) | long term: it does not drift while the robot stands still | short term: wheel slip, bumps, being pushed or turned by hand, a wrong `B` |
| `course + gyro_inc` (gyro) | short term: it sees every real rotation, even when the wheels slip | long term: bias `b0` and scale factor `s` integrate into drift |

A **complementary filter** takes the high-frequency part from the gyro and the low-frequency part from the odometry, and adds them into one estimate.

### Complementary filter for the robot course

The robot uses the same variables as in Weeks 6–10:

```
theta_l     = encoder E4 · π / 180
theta_r     = encoder E3 · π / 180
prev_path   = path
path        = (theta_r + theta_l) · r / 2
course_odom = (theta_r − theta_l) · r / B
gyro_inc    = (gz − b0) / (1 + s) · dt                 (in rad)

course      = K · (course + gyro_inc) + (1 − K) · course_odom

x           = x + (path − prev_path) · cos(course)
y           = y + (path − prev_path) · sin(course)
```

- `K = 1` means the course comes from the gyro only (Week 10, run B).
- `K = 0` means the course comes from the encoders only (Week 10, run A).
- Values in between blend the two.

### What `K` really means: the time constant

`K` is applied in **every control cycle**, so its effect depends on the loop period `dt`. The filter has a time constant `τ`:

```
K = τ / (τ + dt)        ⇔        τ = K · dt / (1 − K)
```

- Rotations faster than `τ` are taken from the gyro.
- Changes slower than `τ` are pulled back to `course_odom`.

Example with `dt = 10 ms`:

| `K` | `τ` | Behaviour |
|---|---|---|
| 0.98 | ≈ 0.5 s | gyro for quick turns and knocks, odometry after about 1–2 s |
| 0.5 | ≈ 0.01 s | practically odometry only |
| 0.02 | ≈ 0.0002 s | odometry only |

Compute `τ` for **your** measured `dt` before the experiments, and predict how the robot will act in each run.

### Gyro calibration comes first

`gyro_inc` is only as good as `b0` and `s`. An uncalibrated bias of 1 °/s becomes 60° of course error in one minute. With large `K`, that error goes straight into the controller.

### Reading

- MIT 16.333 Lecture 15, *Inertial Sensors, Complementary Filtering, Simple Kalman Filtering* (S. Park, J. How). See the complementary filter section: the gyro is the high-pass path, the slow sensor is the low-pass path.
- P. Gui, L. Tang, S. Mukhopadhyay, *MEMS Based IMU for Tilting Measurement: Comparison of Complementary and Kalman Filter Based Data Fusion*, IEEE ICIEA 2015. Eq. (1) is the filter and Eq. (2) is `α = τ / (τ + dt)`. Here the accelerometer plays the role that odometry plays for us.
- S. Colton, *The Balance Filter*, MIT, 2007. This is the original one-line complementary filter.
- W. T. Higgins, *A Comparison of Complementary and Kalman Filtering*, IEEE Trans. AES, 1975. This is the bridge to Week 12.

---

## 2. Practical task — Lab 11

### Goal

Drive the trajectory of **the last two digits of your student ID** (the same waypoints `X_ref`, `Y_ref` as in Lab 10) three times, with three filter gains:

| Run | `K` (gyro) | `1 − K` (odometry) |
|---|---|---|
| 1 | 0.98 | 0.02 |
| 2 | 0.5 | 0.5 |
| 3 | 0.02 | 0.98 |

In every run, **disturb the robot by hand** at the same place. Then compare how the robot acts with each `K`.

### Step 1 — Gyro calibration (before any run)

Develop your own gyro calibration in the TRIK Studio program. Do not use the built-in gyro calibration.

1. At program start, the robot stands still for `T_cal` seconds. Choose `T_cal` from your Lab 9 `b0(T)` plot.
2. During `T_cal`, compute `b0` from the raw rate `gz`, using your method from Lab 9 (mean, median, outlier rejection…).
3. Use `s` and the raw-to-deg/s factor from Lab 9. Check the sign: CCW rotation must increase both `course_odom` and the gyro course.
4. **Check the calibration.** Run the program with the motors off and `K = 1`, and leave the robot still for 60 s after calibration. Report the course drift in °/min. It must be below 1 °/min; if it is not, improve the calibration before you continue.

### Step 2 — Robot program

- **Same program, controller and gains as Lab 10.** Only the course line changes to the complementary filter. `K` is a variable set at the start of the program.
- The loop period `dt` must be constant (≈ 10 ms). Measure it, because `τ` depends on it.
- **Logging.** Every cycle, write one line to the log file:

  ```
  time, theta_l, theta_r, gz, bias, course
  ```

  Log `theta_l`, `theta_r` and `gz` raw. Then `course_odom`, the pure gyro course and the filtered course for **any** `K` can all be recomputed offline from every run.
- Name the files `cf_098.txt`, `cf_050.txt` and `cf_002.txt`.

### Step 3 — Runs with a hand disturbance

Mark the start pose with tape, and start all three runs from it. Choose **one long straight segment** and disturb the robot there in every run, in the same way:

1. **Stop:** hold the robot firmly on the floor for about 3 s. The wheels keep spinning and slipping.
2. **Turn:** while holding it, rotate the robot by about 45° and let it go.

A partner times the disturbance and notes the waypoint index. Record each run on video with a phone.

Watch what the robot does after you let it go:
- Does it turn back toward the waypoint?
- Does it drive on in the wrong direction?
- Does it miss the rest of the digits?

### Step 4 — Analysis in Colab

1. **Reconstruct.** From each log, recompute three courses:
   - `course_odom` from `theta_l`, `theta_r`;
   - the pure gyro course `course_gyro` from `gz`, `b0`, `s`;
   - the filtered `course` with that run's `K`.

   Check that the recomputed `course` matches the logged one.
2. **Course plots.** For each run, plot `course_odom`, `course_gyro` and `course` over time, and mark the disturbance interval.
   - How large is the jump in `course_gyro` when you turned the robot?
   - Does `course_odom` see the turn?
   - How fast does `course` return to `course_odom`? Compare that time with your `τ`.
3. **Trajectories.** Plot `X_ref, Y_ref` and the reconstructed `x, y` of all three runs on one x–y plot.
4. **Comparison with the ideal trajectory.** Compute `distance` and `courseAngle` to the ideal segment, as in Labs 6–7. Report for each run:
   - max |`distance`| and RMS `distance`;
   - mean |`courseAngle`|;
   - the final error to the last waypoint.
5. **What-if.** Use the log of run 1 and recompute the course offline with other `K` values (for example 0.9, 0.99, 0.999). Which `K` keeps the hand-turn in the estimate for at least 5 s? What `τ` is that?

### Deliverables

Create a repo `CS413-Robotics/Lab11/<content>`:

1. **`complementary_filter.qrs`** — the program with your gyro calibration and the filter, `K` as a variable.
2. **Calibration check** — the 60 s static log and the measured course drift in °/min.
3. **Logs** `cf_098.txt`, `cf_050.txt`, `cf_002.txt`.
4. **Videos** of the three runs, or links to them.
5. **Colab notebook (`.ipynb`)** with the plots and the summary table from Step 4:

   | Run | `K` | `τ` (s) | max \|`distance`\| (cm) | RMS `distance` (cm) | mean \|`courseAngle`\| (°) | final error (cm) | behaviour after the disturbance |
   |---|---|---|---|---|---|---|---|
   | 1 | 0.98 | | | | | | |
   | 2 | 0.5 | | | | | | |
   | 3 | 0.02 | | | | | | |

6. **Short analysis (6–10 sentences):**
   - How did the robot act after the disturbance with each `K`? Did it notice the turn, and did it correct it?
   - Why do runs 2 and 3 behave almost the same? Use `τ` to explain.
   - Holding the robot made the wheels slip. Which part of the estimate did that corrupt (`path` or `course`)? Can the complementary filter fix it?
   - Which `K` would you choose for this robot, and why? What does a fixed `K` miss, which the Kalman filter in Week 12 will adapt?
