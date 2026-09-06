# Maze_Task7
making ROS2 package for solving a maze


## 🚀 How to Run

### 1. Clone the Repository

```bash
git clone <TEAM_REPO_URL>
```

### 2. Navigate to the ROS 2 Workspace

```bash
cd "path to"/Maze_Task7/ros2_ws
```

### 3. Build the Workspace

```bash
colcon build
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

### 5. Open 4 Terminals

Make sure to run `source install/setup.bash` in each terminal before running the nodes.

**Terminal 1 — Launch the Maze Simulation**

```bash
ros2 launch maze_control maze_simulation_tb3.launch.py
```

**Terminal 2 — Run the Movement Node**

```bash
ros2 run robot_controller movement_node
```

**Terminal 3 — Run the Wall Client**

```bash
ros2 run robot_controller wall_client
```

**Terminal 4 — Run the Maze Solve Node**

```bash
ros2 run robot_controller maze_solve
```

### ▶️ Run Order

1. `maze_simulation_tb3.launch.py`
2. `movement_node`
3. `wall_client`
4. `maze_solve`

After running the four nodes, the complete maze system should be running.


