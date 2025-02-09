# PathFinder.jl - Sampling-Based Path Planning Algorithms in Julia

This Julia package provides implementations of two popular sampling-based path planning algorithms:

- Rapidly exploring random tree (RRT)
- Probabilistic roadmap (the simplified, k-nearest variant: k-Nearest sPRM)

|![RRT](examples/results/rrt_hard.svg) |![PRM](examples/results/prm_hard.svg) |
|--------------------------------------|--------------------------------------|

## Usage

1. Define your `PlanningSpace`

    ```julia
    dimension_of_my_configuration_space = 2
    limits_matrix_of_my_configuration_space = [0 1; 0 1]
    my_collision_checker = conf -> false  # environment with no obstacles

    ps = PlanningSpace(
        dim=dimension_of_my_configuration_space,
        limits=limits_matrix_of_my_configuration_space,
        collision_check=my_collision_checker,
        collision_resolution=0.01)
    ```
    where:
    - `PlanningSpace.dim` is the dimension of the configuration space
    - `PlanningSpace.limits` is (dim, 2) matrix with [min, max] for each dimension
    - `PlanningSpace.collision_check` is a predicate function, which takes a configuration (dim-dimensional vector) as input and returns True if it collides with obstacles
    - `PlanningSpace.collision_resolution` is the distance, for which collision is checked during planning

2. Select your favorite planner

    ```julia
    planner = RRTPlanner(step_size=0.05, max_iters=10000, tolerance=0.05)
    ```
    or
    ```julia
    planner = PRMPlanner(roadmap_size=200, k_neighbors=8)
    ```


3. Run the `plan` function
    
    ```julia
    start_conf = [0.1, 0.1]
    goal_conf = [0.9, 0.9]
    path, graph = plan(start_conf, goal_conf, ps, planner)
    ```
    which returns:
    - `path` from start_conf to goal_conf ((dim, num_of_waypoints) matrix) or `nothing` if the planning algorithm could not find a collision-free path
    - `graph` which is either a `Graph` for PRM algorithm or `Tree` for RRT algorithm

See examples/example.jl for a full example.

## Author

Leoš Drahotský (draholeo@fel.cvut.cz)

## License

This project is licensed under the [MIT License](LICENSE.md) - see the LICENSE.md file for details.
