function line_collides(start, stop, ps::PlanningSpace)
    dist = norm(stop - start)
    steps = ceil(dist / ps.collision_resolution)
    interpolation = (start * (1 - i / steps) + (stop * i / steps) for i in 0:steps)
    for conf in interpolation
        if ps.collision_check(conf)
            return true
        end
    end

    return false
end

sample_random_conf(ps::PlanningSpace) = rand(ps.dim) .* (ps.limits[:, 2] - ps.limits[:, 1]) + ps.limits[:, 1]