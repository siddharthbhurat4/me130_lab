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
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/me130_lab/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
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
