# How to Run

## Requirements

* Windows recommended
* Python 3.10 or newer
* Git
* Pygame

The project uses Python's standard library for its navigation and simulation backend. Pygame is required for the current visualization interface.

## 1. Clone the Repository

Open PowerShell or a terminal:

```bash
git clone https://github.com/Akshaj-echo/warehouse-robot-simulator.git
cd warehouse-robot-simulator
```

## 2. Check Python

On Windows, use the Python launcher:

```bash
py --version
```

Python 3.10 or newer is recommended.

If `python` opens the Microsoft Store or is not recognized, use `py` instead.

## 3. Install Dependencies

Install the required package:

```bash
py -m pip install -r requirements.txt
```

If `requirements.txt` is not yet available, install Pygame directly:

```bash
py -m pip install pygame
```

## 4. Run the Simulator

Start the current V4 application with:

```bash
py main.py
```

The graphical interface is used for visualization and debugging. The navigation system itself is implemented in the backend.

## 5. Run the Automated Tests

Individual tests can be executed directly.

Examples:

```bash
py test_dynamic_navigation.py
py test_full_dynamic_navigation.py
py test_moving_obstacle_navigation.py
py test_guaranteed_replanning.py
py test_unreachable_goal.py
```

Additional tests cover:

* A* and Dijkstra navigation
* configurable A* heuristics
* simulation time
* obstacle appearance and disappearance
* multiple dynamic obstacles
* robot footprint collision
* swept collision
* dynamic swept collision
* robot turning
* controller behavior
* world-state handling
* obstacle representation

Run the relevant test files individually when developing or debugging a specific subsystem.

## 6. Run the Planner Benchmark

The V4 benchmark compares A* and Dijkstra on randomized static environments:

```bash
py benchmark_planners.py
```

The benchmark reports:

* success rate
* planning time
* nodes explored
* path length
* path cost
* A* speedup
* A* node reduction

The benchmark is intended for reproducible performance comparison rather than visual demonstration.

## 7. Backend-First Testing

The navigation stack can be tested without relying on the graphical interface.

This is useful when:

* developing planners
* testing collision handling
* testing dynamic obstacles
* testing replanning
* debugging robot motion
* running automated verification
* benchmarking planners

Backend tests should be preferred when validating navigation behavior.

## 8. Development Workflow

Create a feature branch before making project changes:

```bash
git checkout -b feature-name
```

Make and test the changes, then commit:

```bash
git add .
git commit -m "Describe the change"
```

Push the branch:

```bash
git push -u origin feature-name
```

Changes to `main` are merged through a pull request.

## 9. Troubleshooting

### `py` is not recognized

Verify that Python is installed:

```bash
py --version
```

If this fails, install Python and ensure the Python launcher is available.

### `python` opens the Microsoft Store

Windows may have the Microsoft Store Python alias enabled.

Use:

```bash
py
```

instead of:

```bash
python
```

### `ModuleNotFoundError: No module named 'pygame'`

Install the dependency:

```bash
py -m pip install pygame
```

Or install all project dependencies:

```bash
py -m pip install -r requirements.txt
```

### A test fails

Run the failing test directly:

```bash
py test_name.py
```

Read the failure output before changing the implementation. A failing test should be treated as evidence that the expected behavior or implementation needs investigation.

## 10. Reproducing the Planner Benchmark

To reproduce the V4 planner benchmark:

```bash
py benchmark_planners.py
```

The benchmark generates randomized environments and compares the planners using the same environment conditions.

Results may vary slightly between runs because the environments are randomized.
