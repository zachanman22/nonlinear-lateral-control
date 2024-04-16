function j = Jsquare(state, dt)
    j = 0.5 * sum(state(1,:).^2) * dt;
end