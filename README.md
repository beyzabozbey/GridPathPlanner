# GridPathPlanner

Forward A* and Adaptive A* for navigation in partially known environments

## Settings

- `bool adaptive`: Determines whether the code should operate as Repeated Forward A* (if `adaptive = false`) or as Adaptive A* (`adaptive = true`)
- `bool higher_g`: Determines whether the code should break ties in favor of cells with larger g-values (if `higher_g = true`) or smaller g-values (if `higher_g = false`)

You can change these variables in `main.cpp`

## How to compile and run

Use `make` command to compile, and it will create an executable named `GridPathPlanner`. To run the program, use `./GridPathPlanner` command.
