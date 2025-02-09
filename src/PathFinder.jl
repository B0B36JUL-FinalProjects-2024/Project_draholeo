module PathFinder

export PlanningSpace, RRTPlanner, PRMPlanner
export plan

abstract type AbstractPlanner end

"""
Represents an abstract configuration space for a path planning problem.

# Fields
- `dim::Int64`: Dimension of the configuration space.
- `limits::Matrix{Float64}`: (dim, 2) matrix with [min, max] for each dimension.
- `collision_check::Function`: A predicate function that takes a configuration (a dim-dimensional vector) as input and 
    returns `true` if the configuration is in collision with an obstacle, otherwise `false`.
- `collision_resolution::Float64`: The resolution at which the planner checks for collisions along a path. Smaller 
    values result in finer collision checks but may slow down planning.

# Example
```julia
julia> ps = PlanningSpace(
            dim=2, 
            limits=[0 1; 0 1], 
            collision_check=(conf::Vector{Float64}) -> false, 
            collision_resolution=0.01
        )
```
"""
Base.@kwdef struct PlanningSpace
    dim::Int64
    limits::Matrix{Float64}
    collision_check::Function
    collision_resolution::Float64 = 0.1
end

include("graph.jl")
include("tree.jl")
include("utils.jl")
include("rrt.jl")
include("prm.jl")

end
