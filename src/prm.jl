using LinearAlgebra
using DataStructures

"""
Defines options for the PRM algorithm.

# Fields
- `roadmap_size::Int64`: Number of vertices in the generated roadmap.
- `k_neighbors::Int64`: Number of nearest neighbors each vertex connects to in the roadmap. Higher values improve 
    path quality but increase computation time.

# Example
```julia
julia> planner = PRMPlanner(roadmap_size=200, k_neighbors=8)
```
"""
Base.@kwdef struct PRMPlanner <: AbstractPlanner
    roadmap_size::Int64 = 1000
    k_neighbors::Int64 = 12
end

function construct_roadmap(start, goal, ps::PlanningSpace, planner::PRMPlanner)
    graph = Graph(ps.dim)
    start_node = add_node!(graph, start)
    goal_node = add_node!(graph, goal)

    while length(get_nodes(graph)) < planner.roadmap_size
        random_conf = sample_random_conf(ps)
        if !ps.collision_check(random_conf)
            add_node!(graph, random_conf)
        end
    end

    for node in get_nodes(graph)
        neighbors = knn_search(graph, node.coords, planner.k_neighbors + 1)
        for n in neighbors
            if n.coords == node.coords
                continue
            end
            if !line_collides(node.coords, n.coords, ps)
                add_edge!(node, n)
            end
        end
    end

    return start_node, goal_node, graph
end

function reconstruct_path(node::Node, parents)
    path = node.coords
    while !isnothing(parents[node])
        path = hcat(path, parents[node].coords)
        node = parents[node]
    end
    reverse!(path, dims=2)
    return path
end

function A_star(start::Node, goal::Node, graph::Graph)
    h(node) = norm(goal.coords - node.coords) # euclidean heuristics definition

    visited = Set{Node}()

    parents = Dict{Node,Union{Node,Nothing}}()
    parents[start] = nothing

    g_scores = Dict{Node,Float64}()
    g_scores[start] = 0.0

    queue = PriorityQueue{Node,Float64}()
    enqueue!(queue, start, 0 + h(start))

    while length(queue) > 0
        current = dequeue!(queue)
        push!(visited, current)

        if current == goal
            return reconstruct_path(current, parents)
        end

        for neighbor in current.neighbors
            if neighbor in visited
                continue
            end

            g = g_scores[current] + norm(current.coords - neighbor.coords)
            if neighbor in keys(queue) && g > g_scores[neighbor]
                continue
            end

            parents[neighbor] = current
            g_scores[neighbor] = g
            queue[neighbor] = g + h(neighbor)
        end
    end

    return nothing
end

"""
    path, graph = plan(start, goal, ps::PlanningSpace, planner::PRMPlanner)

Plans a collision-free path from `start` to `goal` using the Probabilistic Roadmap (PRM) algorithm in the given  
`ps` planning space.

# Returns
- `path::Matrix{Float64}`: A (ps.dim, num_waypoints) matrix representing the planned path,  
  or `nothing` if no valid path is found.
- `graph::PathFinder.Graph`: The constructed PRM roadmap.

# Example
```julia-repl
julia> ps = PlanningSpace(
            dim=2, 
            limits=[0 1; 0 1], 
            collision_check=(conf::Vector{Float64}) -> false, 
            collision_resolution=0.01
        )
julia> planner = PRMPlanner(roadmap_size=200, k_neighbors=8)
julia> path, graph = plan([0.1, 0.5], [0.9, 0.5], ps, planner)
```
"""
function plan(start, goal, ps::PlanningSpace, planner::PRMPlanner)
    start_node, goal_node, roadmap = construct_roadmap(start, goal, ps, planner)
    path = A_star(start_node, goal_node, roadmap)
    return path, roadmap
end

