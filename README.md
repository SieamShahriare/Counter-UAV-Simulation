# Counter-UAV Simulation

A multi-agent drone intercept simulation built with PX4 SITL, Gazebo, and MAVSDK-Python. An enemy UAV executes randomized evasive maneuvers while a defender drone autonomously pursues and intercepts it using GPS-based proportional navigation.

---

## Demo

Click to watch

[![Counter UAV Demo](https://img.youtube.com/vi/hm-dU5wbCms/0.jpg)](https://www.youtube.com/watch?v=hm-dU5wbCms)

## Architecture

```
counter_uav/
├── PX4-Autopilot/          # PX4 firmware + Gazebo models (submodule)
├── config/
├── logs/
├── models/
├── src/
│   ├── sim.py
├── worlds/
├── requirements.txt
└── README.md
```

### How It Works

The simulation runs two concurrent asyncio coroutines coordinated by an `asyncio.Event`:

1. **Enemy agent** (`x500`, Instance 0) waits for full preflight health checks, arms, takes off, then executes 15 randomized velocity vectors (3–10 m/s forward speed, ±30°/s yaw rate) over ~45 seconds to simulate evasive flight.
2. **Defender agent** (`x500_depth`, Instance 1) holds on the pad until the enemy signals it is airborne, then scrambles, climbs to altitude, and uses proportional navigation to close the distance at 15 m/s. Impact is declared at < 1 m separation.

---

## Prerequisites

### System Requirements

- macOS (Apple Silicon tested)
- Python 3.10+
- Conda (Miniconda recommended)
- PX4-Autopilot (built for SITL)
- Gazebo `gz-sim8`

### Install the PX4_Autopilot Submodule

```bash
cd counter_uav
git submodule update --init --recursive
```

#### or download manually from here

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
```

### Setup Python Environement

```bash
conda create -n Counter-UAV python=3.10
conda activate Counter-UAV
```

Install requirements:

```bash
pip install -r requirements.txt
```

---

## Running the Simulation

You need **4 terminal windows**. Run them in order and wait for each to finish booting before moving to the next.

***Make sure to activate the PX4 environment in every instance before running the commands***

```bash
source PX4-Autopilot/.venv/bin/activate
```

### Terminal 1 — Enemy UAV (x500, Instance 0)

```bash
cd PX4-Autopilot

export GZ_IP=127.0.0.1
export GZ_PARTITION=counter_uav
export HEADLESS=1
export PX4_GZ_WORLD_X=100
export PX4_GZ_WORLD_Y=0

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 ./build/px4_sitl_default/bin/px4 -i 0
```

> Wait for `INFO [commander] Ready for takeoff!`

### Terminal 2 — Defender UAV (x500_depth, Instance 1)

```bash
cd PX4-Autopilot

export GZ_IP=127.0.0.1
export GZ_PARTITION=counter_uav
export HEADLESS=1
export PX4_GZ_WORLD_X=0
export PX4_GZ_WORLD_Y=0

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4 -i 1
```

> Wait for `INFO [commander] Ready for takeoff!`

### Terminal 3 — Gazebo GUI

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=counter_uav

gz sim -g -v 4
```

> The 3D world will open. Both drones should be visible — the enemy at (100, 0) and the defender at (0, 0).

### Terminal 4 — Run the Simulation

```bash
conda activate DroneVis
python src/sim.py
```

---

## Expected Output

```
[Enemy] Waiting for connection...
[Defender] Waiting for connection...
[Enemy] Connected to MAVLink!
[Enemy] Waiting for EKF/GPS convergence and preflight checks...
[Defender] Connected to MAVLink!
[Defender] Waiting for EKF/GPS convergence and preflight checks...
[Enemy] Health checks passed! Ready to arm.
[Enemy] Arming and taking off...
[Enemy] Armed successfully!
[Defender] Health checks passed. Holding on pad for target launch...
[Enemy] Airborne. Signaling Defender...
[Defender] Target airborne! Scrambling...
[Defender] Armed successfully!
[Defender] Offboard started. Running intercept logic...
[Defender] Target Lock | Dist: 98.3m | Δalt: 2.1m
[Defender] Target Lock | Dist: 85.7m | Δalt: 1.8m
...
[Defender] Target Lock | Dist: 0.8m | Δalt: 0.1m
[Defender] *** KINETIC IMPACT !!! ***
```

---

## Key Design Decisions

| Decision | Reason |
|---|---|
| `asyncio.Event` for synchronization | Prevents the Defender from launching before the Enemy is airborne |
| `health.is_armable` check | Waits for all PX4 preflight checks, not just GPS lock |
| `arm_with_retry()` with 5 attempts | macOS SITL timing causes intermittent arming denials |
| Distinct gRPC ports (50051, 50052) | Prevents MAVSDK backend collision when two `System()` objects share a process |
| `VelocityBodyYawspeed` for enemy | Enables realistic forward-flight evasion relative to drone heading |
| Proportional navigation at 15 m/s | Guarantees the defender closes faster than the enemy can evade |
| Kinetic strike threshold at 1 m | Realistic for simulation; Gazebo collision detection fires at this range |
| `HEADLESS=1` on PX4 instances | Saves CPU — the GUI is launched separately via `gz sim -g` |

---

## Troubleshooting

**`COMMAND_DENIED` on arming**  
The EKF hasn't fully converged. The `arm_with_retry()` function handles this automatically with up to 5 retries.

**`Address already in use`**  
A previous crashed run left MAVSDK or PX4 processes alive. Run:

```bash
pkill -9 -f px4; pkill -9 -f gz; pkill -9 -f ruby; pkill -9 -f mavsdk
```

**Defender intercepts at 0 m immediately**  
The Enemy's EKF hasn't converged yet — its GPS defaults to the local origin. The `asyncio.Event` handshake prevents this — ensure both drones have passed health checks before the event fires.

**`gazebo already running world: default` in PX4 logs**  
A headless Gazebo server from a previous run survived. Kill all processes and relaunch.

---

## Known Warnings (Safe to Ignore)

- `Received ack for not-existing command: 176!` — Harmless MAVLink quirk when entering offboard mode
- `Other threads are currently calling into gRPC` — macOS gRPC fork warning; silence with `export GRPC_ENABLE_FORK_SUPPORT=0`
- `gz_frame_id` SDF warnings — Cosmetic only, sensors function correctly

---

## Roadmap

- [ ] OpenCV camera feed bridge from Gazebo topic
- [ ] Visual terminal guidance — switch from GPS to camera tracking inside 10 m
- [ ] Target detection using YOLOv8-nano or color thresholding
- [ ] HUD overlay with target lock indicator

## Author

Sieam Shahriare
