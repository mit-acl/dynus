# SANDO: Safe Autonomous Trajectory Planning for Dynamic Unknown Environments #

If you like this project, please consider starring ⭐ the repo!

### **Submitted to the IEEE Transactions on Robotics (T-RO)**

<table>
<tr>
<td width="50%"><b>Spatiotemporal SFC</b><br><video src="https://github.com/user-attachments/assets/7279e010-bde3-4061-9a28-07a145092276" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Hardware Heatmap</b><br><video src="https://github.com/user-attachments/assets/f9bf40c3-18ec-498c-80f7-e7ff92dcb2a6" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
<tr>
<td width="50%"><b>Sim Interactive</b><br><video src="https://github.com/user-attachments/assets/6718410a-aa0d-4726-a6ca-605a22e25b2c" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Sim Static</b><br><video src="https://github.com/user-attachments/assets/61e676a6-4b03-480a-90b3-4dafb19d7198" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
<tr>
<td width="50%"><b>Sim Dynamic</b><br><video src="https://github.com/user-attachments/assets/f672ad26-3e06-4cbf-99d2-fd6b2eaeac1d" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Hardware Single Dynamic</b><br><video src="https://github.com/user-attachments/assets/3a6a9805-078e-4b12-bc5e-d3421edce9cf" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
<tr>
<td width="50%"><b>Hardware Multiple Dynamic</b><br><video src="https://github.com/user-attachments/assets/aac54514-d8fb-4aa7-8304-63ffc71c2bad" width="100%" autoplay loop muted playsinline controls></video></td>
<td width="50%"><b>Hardware Dynamic + Static</b><br><video src="https://github.com/user-attachments/assets/175d183b-60ea-40a3-92ae-1ece3cdf40e6" width="100%" autoplay loop muted playsinline controls></video></td>
</tr>
</table>

## Paper

SANDO: Safe Autonomous Trajectory Planning for Dynamic Unknown Environments is available [https://arxiv.org/abs/2511.10822](https://arxiv.org/abs/2511.10822)!

```bibtex
@article{kondo2026sando,
      title={SANDO: Safe Autonomous Trajectory Planning for Dynamic Unknown Environments},
}
```

## Video

The full video is available [https://youtu.be/Pvb-VPUdLvg](https://youtu.be/Pvb-VPUdLvg).

## Interactive Demo

SANDO includes a built-in interactive mode where you can click goals in RViz and watch the drone navigate through dynamic obstacles in real time. See the [Run Simulation](#use-docker-recommended) section for instructions.

## Setup

SANDO has been tested on both Docker and native installations on Ubuntu 22.04 with ROS 2 Humble.

### Use Docker (Recommended)

1. **Install Docker:**  
   Follow the [official Docker installation guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

2. **Clone the Repository and Navigate to the Docker Folder:**
   ```bash
   mkdir -p ~/code/ws/src
   cd ~/code/ws/src
   git clone https://github.com/mit-acl/dynus.git sando
   cd sando/docker
   ```

3. **BUILD:**
    - Navigate to the docker folder in your sando repo (eg. `cd ~/code/ws/src/sando/docker/`) and run this
      ```bash
      make build
      ```

4. **Run Simulation**

    SANDO provides four simulation modes with three difficulty levels:

    | Mode | Description |
    |------|-------------|
    | `static` | Static forest obstacles (Gazebo) |
    | `dynamic` | Known dynamic obstacles (RViz-only, lightweight) |
    | `unknown_dynamic` | Unknown dynamic obstacles detected via pointcloud (Gazebo) |
    | `interactive` | Click-to-goal with obstacles in a 30x30m arena (RViz-only) |

    | Difficulty | Obstacles |
    |------------|-----------|
    | `easy` | 50 |
    | `medium` | 100 |
    | `hard` | 200 |

    **Demo modes** (goal is sent automatically):
    ```bash
    make run-demo SCENARIO=static_easy
    make run-demo SCENARIO=dynamic_hard
    make run-demo SCENARIO=unknown_dynamic_medium
    ```

    **Interactive mode** (click goals in RViz using "2D Nav Goal"):
    ```bash
    make run-interactive
    make run-interactive NUM_OBSTACLES=100   # customize obstacle count
    ```

    **Convenience aliases:**
    ```bash
    make run-static-easy
    make run-static-medium
    make run-static-hard
    make run-dynamic-easy
    make run-dynamic-medium
    make run-dynamic-hard
    make run-unknown-easy
    make run-unknown-medium
    make run-unknown-hard
    ```

    **Without GPU:**
    ```bash
    make run-demo SCENARIO=static_easy GPU=false
    ```

    **Debug shell:**
    ```bash
    make shell
    ```

<details>
  <summary><b>Useful Docker Commands</b></summary>

  - **Remove all caches:**
    ```bash
    docker builder prune
    ```

  - **Remove all containers:**
    ```bash
    docker rm $(docker ps -a -q)
    ```

  - **Remove all images:**
    ```bash
    docker rmi $(docker images -q)
    ```

</details>

### Native Installation

1. **Clone the Repository and Navigate to the Workspace Folder:**
   ```bash
   mkdir -p ~/code/ws
   cd ~/code/ws
   git clone https://github.com/mit-acl/dynus.git sando
   cd sando
   ```

2. **Run the Setup Script:**
   ```bash
   ./setup.sh
   ```
   This script will first install ROS 2 Humble, then SANDO and its dependencies. Please note that this script modifies your `~/.bashrc` file.

 3. **Run the Simulation**

    Source the workspace and run simulations using `run_sim.py`:
    ```bash
    cd ~/code/dynus_ws
    source install/setup.bash
    ```

    **Demo modes** (goal is sent automatically):
    ```bash
    # Static forest environments (Gazebo)
    python3 src/sando/scripts/run_sim.py -m static -d easy -s install/setup.bash
    python3 src/sando/scripts/run_sim.py -m static -d medium -s install/setup.bash
    python3 src/sando/scripts/run_sim.py -m static -d hard -s install/setup.bash

    # Known dynamic obstacles (RViz-only, lightweight)
    python3 src/sando/scripts/run_sim.py -m dynamic -d easy -s install/setup.bash
    python3 src/sando/scripts/run_sim.py -m dynamic -d hard -s install/setup.bash

    # Unknown dynamic obstacles (Gazebo + obstacle tracker)
    python3 src/sando/scripts/run_sim.py -m unknown_dynamic -d medium -s install/setup.bash
    ```

    **Interactive mode** (click goals in RViz using "2D Nav Goal"):
    ```bash
    python3 src/sando/scripts/run_sim.py -m interactive -s install/setup.bash

    # Customize obstacle count
    python3 src/sando/scripts/run_sim.py -m interactive --num-obstacles 100 -s install/setup.bash
    ```

## Benchmarking

This section describes the complete workflow for running local trajectory optimization benchmarks.

### Overview

The benchmarking pipeline consists of three main steps:
1. **Generate Safety Corridors**: Create standardized test cases with safe flight corridors
2. **Run Benchmarks**: Execute trajectory optimization benchmarks (standardized and variable elimination)
3. **Generate LaTeX Tables**: Process results and generate publication-ready tables

### Step 1: Generate Safety Corridors

Safety corridors define the collision-free space for trajectory optimization. Generate them once and reuse for all benchmarks.

```bash
# Build the workspace
cd ~/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando

# Source the workspace
source install/setup.bash

# Generate safety corridors
tmuxp load src/sando/launch/generate_sfc.yaml
```

**Output**: Safety corridor files (`.mysco2` format) saved to `src/sando/data/`

**What it does**:
- Launches simulator in background
- Generates random start/goal pairs in standardized environment
- Computes safe flight corridors using convex decomposition
- Saves corridors as binary files for reproducible benchmarks

### Step 2: Run Standardized Benchmarks

Run comprehensive benchmarks comparing SANDO (single/multi-threaded) and FASTER (original).

```bash
cd ~/code/dynus_ws/src/sando/benchmarking

# Run the full benchmark suite
python3 run_benchmark_suite.py
```

**Output**: CSV files in `benchmark_data/`:
- `single_thread/sando_N_benchmark.csv` (N=4,5,6)
- `single_thread/original_faster_N_benchmark.csv` (N=4,5,6)
- `multi_thread/sando_N_benchmark.csv` (N=4,5,6)

**What it does**:
- Tests multiple problem sizes (N=4,5,6 segments)
- Compares single-threaded vs multi-threaded optimization
- Measures computation time, success rate, trajectory quality, and constraint violations
- Runs 100+ test cases per configuration

**Options**:
```bash
# Factor determination mode (for tuning factor ranges)
python3 run_benchmark_suite.py --factor-determination
```

### Step 3: Run Variable Elimination Benchmarks

Compare SANDO with and without variable elimination to demonstrate its performance impact.

```bash
cd ~/code/dynus_ws/src/sando/benchmarking

# Run VE comparison benchmark
python3 run_benchmark_suite.py --ve-comparison
```

**Output**: CSV files in `benchmark_data/ve_benchmark/`:
- `sando_N_with_ve_benchmark.csv` (N=4,5,6)
- `sando_N_without_ve_benchmark.csv` (N=4,5,6)

**What it does**:
- Runs multi-threaded SANDO with variable elimination enabled
- Runs multi-threaded SANDO with variable elimination disabled
- Directly compares optimization speed for the same problem instances
- Highlights the computational benefit of variable elimination

### Step 4: Generate LaTeX Tables

Process benchmark results and generate publication-ready LaTeX tables.

```bash
cd ~/code/dynus_ws/src/sando/benchmarking

# Generate both standardized and VE benchmark tables
python3 generate_latex_table.py
```

**Output**: LaTeX files in `/home/kkondo/paper_writing/SANDO_v3/tables/`:
- `standardized_benchmark.tex` - Full comparison table
- `ve_benchmark.tex` - Variable elimination comparison table

**What it does**:
- Loads CSV benchmark data
- Computes statistics (mean, success rate, violations)
- Generates formatted LaTeX tables with best/worst highlighting
- Ready for direct inclusion in paper with `\input{}`

### Step 5: Analyze Results (Optional)

Use Jupyter notebook for detailed analysis and visualization.

```bash
cd ~/code/dynus_ws/src/sando/benchmarking

# Open Jupyter notebook
jupyter notebook local_traj_benchmark.ipynb
```

**Features**:
- Load and analyze benchmark CSV data
- Generate summary statistics tables
- Create plots comparing different configurations
- Export results for paper figures

**How to use the notebook**:

1. **Run the first cell** to load standardized benchmark data:
   ```python
   # The notebook automatically loads from:
   # - single_thread/sando_N_benchmark.csv
   # - single_thread/original_faster_N_benchmark.csv
   # - multi_thread/sando_N_benchmark.csv
   ```
   This generates a unified summary table with statistics.

2. **Run the VE benchmarking cell** (second cell) to load variable elimination data:
   ```python
   # Loads from ve_benchmark/ folder:
   # - sando_N_with_ve_benchmark.csv
   # - sando_N_without_ve_benchmark.csv
   ```
   This generates a comparison table showing VE impact.

3. **Run visualization cells** to create plots (optional):
   - Computation time vs N
   - Success rates
   - Trajectory quality metrics

**Note**: The notebook expects the benchmark data to exist. Run Steps 2-3 first to generate the CSV files.

### Step 6: Visualize Trajectories in RViz (Optional)

Visualize and compare trajectories from different planners side-by-side in RViz.

```bash
cd ~/code/dynus_ws

# Build if needed
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando

# Source the workspace
source install/setup.bash

# Launch visualization
tmuxp load src/sando/launch/visualize_local_trajs.yaml
```

**What it does**:
- Reads saved trajectory files from `traj_dump/` directories
- Loads the same test case for multiple planners
- Displays trajectories with color-coded velocity profiles
- Shows safe flight corridors and obstacle environment
- Allows step-by-step comparison

**Configuration**:
- Edit `visualize_local_trajs.yaml` to specify which trajectories to load
- Set `traj_dump_root_dirs_` to point to trajectory dump directories
- Trajectories are automatically generated during benchmarks if `traj_dump_enable: true`

**Controls in RViz**:
- Use markers to select different test cases
- Toggle trajectory visibility per planner
- Inspect velocity, acceleration, and jerk profiles
- Verify corridor constraints visually

### Directory Structure

```
sando/
├── benchmarking/
│   ├── run_benchmark_suite.py          # Main benchmark runner
│   ├── generate_latex_table.py         # Table generator
│   ├── local_traj_benchmark.ipynb      # Analysis notebook
│   └── ...
├── benchmark_data/
│   ├── single_thread/                  # Single-threaded results
│   ├── multi_thread/                   # Multi-threaded results
│   └── ve_benchmark/                   # Variable elimination results
└── data/                               # Safety corridor files (.mysco2)
```

### Tips

- **Build once**: Only rebuild when you modify C++ code
- **Reuse corridors**: Generate safety corridors once, reuse for all benchmarks
- **Run overnight**: Full benchmark suite takes several hours
- **Check results**: Use notebook to verify data quality before generating tables
- **VE comparison**: Run after standardized benchmarks to demonstrate algorithmic contribution

## Simulation Benchmarking (Dynamic & Static Environments)

This section describes how to run full end-to-end simulation benchmarks — launching the planner, flying through environments, and collecting metrics (success rate, computation time, travel time, path length, smoothness, constraint violations, collisions).

There are two modes:
- **Dynamic** (`rviz-only`): Procedurally generated obstacles (static + moving). Lightweight, no Gazebo.
- **Static** (`gazebo`): Pre-defined forest worlds (`easy_forest.world`, `medium_forest.world`, `hard_forest.world`). Requires Gazebo.

### Prerequisites

**Docker (recommended):**

```bash
# Build the Docker image
cd ~/code/ws/src/sando/docker
make build

# Run the container (GPU + display forwarding)
make run

# Inside the container, everything is already built. Rebuild if needed:
cd /home/kkondo/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando
source install/setup.bash
```

**Native installation:**

```bash
cd ~/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando
source install/setup.bash
```

### Dynamic Obstacle Benchmark

Runs in `rviz-only` mode with procedurally generated obstacles. Three difficulty cases:
- **Easy**: 50 obstacles
- **Medium**: 100 obstacles
- **Hard**: 200 obstacles

#### 1. Configure `sando.yaml`

Make sure `environment_assumption` is set to `"dynamic"`:

```bash
# In src/sando/config/sando.yaml, verify:
#   environment_assumption: "dynamic"
```

#### 2. Build

```bash
cd ~/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando
```

#### 3. Run benchmark

```bash
python3 src/sando/scripts/run_benchmark.py \
  --setup-bash install/setup.bash \
  --mode rviz-only \
  --cases easy medium hard \
  --config-name dynamic \
  --num-trials 10 \
  --start 0.0 0.0 2.0 \
  --goal 105.0 0.0 2.0 \
  --timeout 50
```

#### 4. Analyze and generate LaTeX table

```bash
python3 src/sando/scripts/analyze_dynamic_benchmark.py \
  --data-dir src/sando/benchmark_data/dynamic \
  --all-cases \
  --table-type dynamic \
  --latex-name dynamic_benchmark.tex
```

### Static Forest Benchmark

Runs in `gazebo` mode with pre-defined `.world` files. Three difficulty cases:
- **Easy**: `easy_forest.world`
- **Medium**: `medium_forest.world`
- **Hard**: `hard_forest.world`

#### 1. Configure `sando.yaml`

Set `environment_assumption` to `"static"`:

```bash
# In src/sando/config/sando.yaml, set:
#   environment_assumption: "static"
```

#### 2. Build

```bash
cd ~/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando
```

#### 3. Run benchmark

```bash
python3 src/sando/scripts/run_benchmark.py \
  --setup-bash install/setup.bash \
  --mode gazebo \
  --cases easy medium hard \
  --config-name static \
  --num-trials 10 \
  --start 0.0 0.0 2.0 \
  --goal 105.0 0.0 2.0 \
  --timeout 50
```

#### 4. Analyze and generate LaTeX table

```bash
python3 src/sando/scripts/analyze_dynamic_benchmark.py \
  --data-dir src/sando/benchmark_data/static \
  --all-cases \
  --table-type static \
  --latex-name static_benchmark.tex
```

### Where Data Goes

```
src/sando/
├── benchmark_data/
│   ├── dynamic/                          # Dynamic obstacle benchmark results
│   │   ├── easy_YYYYMMDD_HHMMSS/
│   │   │   ├── benchmark_dynamic_*.csv   # Trial metrics (success, time, collisions, etc.)
│   │   │   ├── benchmark_dynamic_*.json  # Same data in JSON format
│   │   │   ├── csv/
│   │   │   │   ├── num_0.csv             # Per-trial computation time breakdown
│   │   │   │   ├── num_1.csv
│   │   │   │   └── ...
│   │   │   └── bags/                     # ROS2 bag recordings per trial
│   │   │       ├── trial_0/
│   │   │       └── ...
│   │   ├── medium_YYYYMMDD_HHMMSS/
│   │   └── hard_YYYYMMDD_HHMMSS/
│   └── static/                           # Static forest benchmark results
│       ├── easy_YYYYMMDD_HHMMSS/
│       ├── medium_YYYYMMDD_HHMMSS/
│       └── hard_YYYYMMDD_HHMMSS/
```

**LaTeX tables** are written to: `/home/kkondo/paper_writing/SANDO_v3/tables/`
- `dynamic_benchmark.tex` — Dynamic obstacle results
- `static_benchmark.tex` — Static forest results

### Benchmark CLI Reference

**`run_benchmark.py`** options:

| Flag | Description | Default |
|------|-------------|---------|
| `--setup-bash` | Path to `install/setup.bash` (required) | — |
| `--mode` | `rviz-only` or `gazebo` | `rviz-only` |
| `--cases` | `easy`, `medium`, `hard`, or `all` | `all` |
| `--config-name` | Name for output directory | `default` |
| `--num-trials` | Trials per case | `5` |
| `--start` | Start position (x y z) | `0 0 2` |
| `--goal` | Goal position (x y z) | `105 0 2` |
| `--timeout` | Seconds per trial | `120` |
| `--env` | Override gazebo environment name | auto from case |
| `--visualize` | Show RViz during benchmark | off |

**`analyze_dynamic_benchmark.py`** options:

| Flag | Description | Default |
|------|-------------|---------|
| `--data-dir` | Path to benchmark data directory | — |
| `--all-cases` | Analyze all cases in directory | off |
| `--table-type` | `dynamic` or `static` | `dynamic` |
| `--latex-name` | Output `.tex` filename | `dynamic_benchmark.tex` |
| `--config-name` | Config name for table caption | `default` |

### Quick Reference

```bash
# ── Dynamic benchmark (full pipeline) ──
# 1. Set sando.yaml: environment_assumption: "dynamic"
# 2. colcon build --packages-select sando
# 3. python3 src/sando/scripts/run_benchmark.py --setup-bash install/setup.bash --mode rviz-only --cases easy medium hard --config-name dynamic --num-trials 10
# 4. python3 src/sando/scripts/analyze_dynamic_benchmark.py --data-dir src/sando/benchmark_data/dynamic --all-cases --table-type dynamic --latex-name dynamic_benchmark.tex

# ── Static benchmark (full pipeline) ──
# 1. Set sando.yaml: environment_assumption: "static"
# 2. colcon build --packages-select sando
# 3. python3 src/sando/scripts/run_benchmark.py --setup-bash install/setup.bash --mode gazebo --cases easy medium hard --config-name static --num-trials 10
# 4. python3 src/sando/scripts/analyze_dynamic_benchmark.py --data-dir src/sando/benchmark_data/static --all-cases --table-type static --latex-name static_benchmark.tex
```

## Hover Avoidance Testing

SANDO includes a hover avoidance system that detects nearby dynamic obstacles when the drone is hovering at a reached goal and autonomously evades them. Two test modes are provided via `run_sim.py`.

### Hover Test (Trefoil Obstacles)

Spawns the drone in an empty world with 3 trefoil-knot obstacles orbiting nearby. The drone's goal equals its start position, so it immediately enters `GOAL_REACHED`. As obstacles pass close, the drone transitions to `HOVER_AVOIDING`, flies away, then returns when safe.

```bash
cd ~/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando
source install/setup.bash

# Launch hover avoidance test
python3 src/sando/scripts/run_sim.py --mode hover-test -s install/setup.bash
```

**What to expect in RViz:**
- Red translucent spheres show the danger zone around each obstacle (radius = `hover_avoidance_d_trigger`)
- An orange dot marks the hover position (the goal the drone returns to)
- The drone evades when a red sphere covers the orange dot, then flies back when it clears

**Expected console output cycle:**
1. Drone starts at (0, 0, 2) and goal is sent to (0, 0, 2)
2. `GOAL_REACHED` — drone hovers in place
3. Obstacle approaches — `HOVER_AVOIDING` — drone moves away
4. Obstacle recedes — `TRAVELING` — drone returns to hover position
5. `GOAL_SEEN` — `GOAL_REACHED` — cycle repeats

### Adversarial Test (Chaser vs. Evader)

Spawns two SANDO agents: an evader (NX01, v_max=5.0 m/s) hovering in place and a chaser (NX02, v_max=1.0 m/s) that continuously navigates toward the evader. Both agents share trajectories, so the evader's hover avoidance triggers when the chaser approaches.

```bash
cd ~/code/dynus_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sando
source install/setup.bash

# Launch adversarial test
python3 src/sando/scripts/run_sim.py --mode adversarial-test -s install/setup.bash
```

**What to expect:**
- NX01 (evader) hovers at (0, 0, 2) and evades when NX02 gets close
- NX02 (chaser) starts at (8, 0, 2) and slowly pursues NX01
- A `chaser_goal_forwarder` node continuously sends NX01's position as NX02's goal

### Configuration

Hover avoidance parameters are in `src/sando/config/sando.yaml`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `hover_avoidance_enabled` | Enable/disable hover avoidance | `true` |
| `hover_avoidance_d_trigger` | Danger radius around obstacles (m) | `4.0` |
| `hover_avoidance_h` | Evasion distance (m) | `3.0` |

Both test modes support `--dry-run` to inspect the generated tmuxp YAML without launching:

```bash
python3 src/sando/scripts/run_sim.py --mode hover-test -s install/setup.bash --dry-run
python3 src/sando/scripts/run_sim.py --mode adversarial-test -s install/setup.bash --dry-run
```
