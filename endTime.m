function convergeTime = endTime(states,dt)
    ys = states(1,:);
    mask = abs(ys) > 0.01;
    convergeTime = sum(mask)*dt;
end

