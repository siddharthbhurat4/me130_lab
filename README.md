# ME130 pendulum — ROS 2 workspace

ROS 2 Jazzy. The control code is C++; the analysis is Python.

| node | hardware |
|---|---|
| `motor_node` | PWM ch0 (GPIO18), DIR GPIO25, PS GPIO24 |
| `encoder_node` | GPIO17, GPIO27 |
| `imu_node` | `/dev/i2c-1` |

`motor_node` is the only node that moves the motor.
## Git Fork, Clone & Build

### 1. Configure Git

Run once on your Raspberry Pi:

```bash
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
```

Use an email associated with your GitHub account.

### 2. Fork the Repository

1. Log in to your GitHub account.
2. Open the [ME130 Lab repository](https://github.com/siddharthbhurat4/me130_lab).
3. Click **Fork** → **Create fork**.

This creates your own copy of the repository under your GitHub account.

### 3. Clone Your Fork

#### Install gh (github authentication) by running

```bash
sudo apt update
sudo apt install gh
```

#### Log in to GitHub

```bash
gh auth login
```
Select:
```text
GitHub.com
HTTPS
Login with a web browser
```
Follow the displayed instructions to authenticate with your GitHub account.


Replace `YOUR_USERNAME` with your GitHub username:
```
git clone https://github.com/YOUR_USERNAME/me130_lab.git
```
```bash
source /opt/ros/jazzy/setup.bash
cd ~/me130_lab/ros2_ws
colcon build
source install/setup.bash
```


### 4. Add the Course Repository

Run once:

```bash
git remote add upstream https://github.com/siddharthbhurat4/me130_lab.git
```

Verify:

```bash
git remote -v
```

* `origin` → your fork
* `upstream` → course repository

### 5. Save Your Work

```bash
git add .
git commit -m "Describe your changes"
git push origin main
```

### 6. Get Course Updates

When updates are released:

```bash
git fetch upstream
git merge upstream/main
git push origin main
```

If Git reports a merge conflict, resolve the conflict before continuing.

## The four labs

### 1. Motor characterization — find the deadband

```bash
ros2 launch me130_pendulum motor_characterization.launch.py
ros2 run me130_pendulum plot_characterization.py
```

**Detach the pendulum first.** `deadband` is 0.0 here on purpose — this lab
*measures* the deadband, so the motor must be driven raw.

The number it prints is the `deadband:=` argument for labs 2, 3 and 4.

### 2. Step response — free and forced

```bash
ros2 launch me130_pendulum step_response.launch.py deadband:=0.065
ros2 run me130_pendulum step_response.py
```

Rod **hanging down**. Writes steps_<timestamp>.csv.

### 3. Frequency response

```bash
ros2 launch me130_pendulum frequency_response.launch.py deadband:=0.065
ros2 run me130_pendulum freq_response.py
```

Rod **hanging down**. Writes `sweep_<timestamp>.csv`.
Hold time per frequency is `skip_s + cycles/f`, so low frequencies are driven
longer. Keep `skip_s` equal to `SKIP_S` in `freq_response.py`.

### 4. Balance upright

```bash
ros2 launch me130_pendulum balance.launch.py deadband:=0.065
```

**The gains default to zero, so this does nothing until you set them.** That
is the lab.

Open a SECOND terminal and run:

```bash
ros2 run me130_pendulum keyboard_node
```
```
  z  zero theta here (hold the rod upright first)
  e  arm            x  disarm            q  quit
  t  telemetry on/off
  p<Kp>  d<Kd>  i<Ki>  w<integral limit>      e.g.  p10.0<Enter>
  f<deadband>
```
## Safety

- killing a test node stops the motor rather than leaving it driving.
- `controller_node` disarms itself past `theta_max_deg` (90°).
- Arming is refused outside `arm_window_deg` (5°) of the zero.
- Gains start at zero so a fresh checkout cannot lunge.

## Parameters worth knowing

| parameter | node | default | meaning |
|---|---|---|---|
| `deadband` | `motor_node` | 0.0 | from lab 1; 0.0 disables compensation |
| `motor_sign` | `motor_node` | 1.0 | flip to -1.0 if +duty spins the wrong way |
| `kp`, `ki`, `kd` | `controller_node` | 0.0 | students set these |
| `step_duties` | `step_sequence_node` | ±0.10…±0.25 | raise if the small steps stall |
| `frequencies_hz` | `sine_sweep_node` | 0.3…3.5 | centre on your resonance |
