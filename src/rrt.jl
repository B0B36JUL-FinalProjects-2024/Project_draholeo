using LinearAlgebra

"""
Defines options for the RRT algorithm.

# Fields
- `step_size::Float64`: Distance between nodes in the RRT search tree.
- `max_iters::Int64 = 10000`: Maximum number of iterations of the RRT algorithm.
- `tolerance::Float64 = 0.05`: Radius of the goal region around the target configuration. The algorithm considers the 
    goal reached when a node is within this distance.

# Example
```julia
julia> planner = RRTPlanner(step_size=0.05, max_iters=10000, tolerance=0.05)
```
"""
Base.@kwdef struct RRTPlanner <: AbstractPlanner
    step_size::Float64 = 0.1
    max_iters::Int64 = 10000
    tolerance::Float64 = 0.05
end

steer(nearest, random, planner::RRTPlanner) = nearest + (random - nearest) * planner.step_size / norm(random - nearest)

function get_result_path(node)
    path = node.coords
    while !isnothing(node.parent)
        path = hcat(path, node.parent.coords)
        node = node.parent
    end
    reverse!(path, dims=2)
    return path
end

"""
    path, tree = plan(start, goal, ps::PlanningSpace, planner::RRTPlanner)

Plans a collision-free path from `start` to `goal` using the Rapidly Exploring Random Tree (RRT) algorithm in the given  
`ps` planning space.

# Returns
- `path::Matrix{Float64}`: A (ps.dim, num_waypoints) matrix representing the planned path,  
  or `nothing` if no valid path is found.
- `tree::PathFinder.Tree`: The constructed RRT search tree.

# Example
```julia-repl
julia> ps = PlanningSpace(
            dim=2,
            limits=[0 1; 0 1],
            collision_check=(conf::Vector{Float64}) -> false, 
            collision_resolution=0.01
        )
julia> planner = RRTPlanner(step_size=0.05, max_iters=10000, tolerance=0.05)
julia> path, tree = plan([0.1, 0.5], [0.9, 0.5], ps, planner)
```
"""
function plan(start, goal, ps::PlanningSpace, planner::RRTPlanner)
    tree = Tree(start)

    for _ in 1:planner.max_iters
        random_conf = sample_random_conf(ps)
        nearest_node = nn_search(tree, random_conf)
        new_conf = steer(nearest_node.coords, random_conf, planner)

        if !line_collides(nearest_node.coords, new_conf, ps)
            new_node = add_node!(tree, nearest_node, new_conf)
            if norm(new_conf - goal) <= planner.tolerance
                return get_result_path(new_node), tree
            end
        end
    end

    return nothing, tree
end
