module PathFinder

export plan

abstract type AbstractPlanner end

Base.@kwdef struct PlanningSpace
    dim::Int64
    limits::Matrix{Float64}
    collision_check::Function
    collision_resolution::Float64 = 0.1
end

include("rrt.jl")
include("tree.jl")

end
