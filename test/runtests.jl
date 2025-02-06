using PathFinder
using Test
using Aqua

@testset "PathFinder.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(PathFinder)
    end
    # Write your tests here.
end
