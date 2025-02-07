using PathFinder
using PathFinder: Tree, add_node!, nn_search
using Test
using Aqua

@testset "PathFinder.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(PathFinder)
    end

    @testset "Nearest neighbor" begin
        tree = Tree([0, 0])
        root = nn_search(tree, [1, 1])
        @test root.coords == [0, 0]
        add_node!(tree, root, [3.0, 0])
        add_node!(tree, root, [3.0, 3.0])
        node3 = nn_search(tree, [1.0, 3.0])
        @test node3.coords == [3, 3]
        add_node!(tree, root, [0, 3])
        node4 = nn_search(tree, [1.0, 3.0])
        @test node4.coords == [0, 3]
    end
end
