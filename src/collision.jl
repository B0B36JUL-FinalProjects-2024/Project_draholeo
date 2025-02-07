function line_collides(start, stop, pp::PlanningSpace)
    dist = norm(stop - start)
    steps = ceil(dist / pp.collision_resolution)
    interpolation = (start * (1 - i / steps) + (stop * i / steps) for i in 0:steps)
    for conf in interpolation
        if pp.collision_check(conf)
            return True
        end
    end

    return False
end
