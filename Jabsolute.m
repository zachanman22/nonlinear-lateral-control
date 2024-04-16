function j = Jabsolute(state, dt)
    j = sum(abs(state(1,:))) * dt;
end