using PathFinder
using PathFinder: Tree, Graph, add_node!, add_edge!, nn_search, knn_search, line_collides
using Test
using Aqua

@testset "PathFinder.jl" begin
    @testset "Code quality (Aqua.jl)" begin
        Aqua.test_all(PathFinder)
    end

    @testset "Tree nearest neighbor" begin
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
        nodes = knn_search(tree, [0, 1], 2)
        @test nodes[1].coords == [0, 0]
        @test nodes[2].coords == [0, 3]
    end

    @testset "Graph nearest neighbor" begin
        graph = Graph(2)
        @test isnothing(knn_search(graph, [1, 1], 2))
        node1 = add_node!(graph, [0, 0])
        node2 = add_node!(graph, [0, 3])
        add_node!(graph, [3, 0])
        add_node!(graph, [3, 3])
        add_edge!(node1, node2)
        test_node = nn_search(graph, [1.0, 3.0])
        @test test_node.coords == [0, 3]
        nodes = knn_search(graph, [0, 1], 2)
        @test nodes[1].coords == [0, 0]
        @test nodes[2].coords == [0, 3]
        nodes = knn_search(graph, [0, 1], 5)
        @test length(nodes) == 4
    end

    @testset "Collision detection" begin
        ps = PlanningSpace(
            dim=2,
            limits=[0 1; 0 1],
            collision_check = c -> c[1] >= 0.5,
            collision_resolution = 0.1
        )
        @test !line_collides([0, 1], [0, 0], ps)
        @test line_collides([0, 1.0], [1.0, 0.0], ps)
    end
end
