function delta = optimalController(currState, parameters, dt)
    Cf = parameters.Cf;
    Cr = parameters.Cr;
    lf = parameters.lf;
    lr = parameters.lr;
    I = parameters.I;
    m = parameters.m;
    Vx = parameters.Vx;

    beta = 0.5;
    alpha = 0.5;

    dt = dt * 100;

    tspan = 0:dt:1;

    delta = zeros(1,length(tspan));

    num_iterations = 5;
    
    for iteration = 1:num_iterations
        state = zeros(5,length(tspan));
        state(:,1) = currState;
        
        for i = 2:length(tspan)
            state(1,i) = state(1,i-1) + dt * (Vx * sin(state(3,i-1)) + state(2,i-1) * cos(state(3,i-1)));
            state(2,i) = state(2,i-1) + dt * (-Vx * state(4,i-1) - ...
                Cf / m * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) - ...
                Cr / m * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
            state(3,i) = state(3,i-1) + dt * (state(4,i-1));
            state(4,i) = state(4,i-1) + dt * (-lf * Cf / I * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) + ...
                lr * Cr / I * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
            state(5,i) = state(5,i-1) + dt * (Vx * cos(state(3,i-1)) + state(2,i-1) * sin(state(3,i-1)));
        end
        
        costate = zeros(4,length(tspan));
        % costate(:,length(tspan)) = 40 * state(:,end).^3;
        
        for i = flip(1:(length(tspan)-1))
            dfdx = [0, cos(state(3,i+1)), Vx * cos(state(3,i+1)) - state(2,i+1) * sin(state(3,i+1)), 0;
                    0, -Cf / (m * Vx) * 1 / (1 + ((state(2,i+1) + state(4,i+1) * lf) / Vx)^2) - Cr / (m * Vx) * 1 / (1 + ((state(2,i+1) - state(4,i+1) * lr) / Vx)^2), ...
                        0, -Vx - lf * Cf / (m * Vx) * 1 / (1 + ((state(2,i+1) + state(4,i+1) * lf) / Vx)^2) + lr * Cr / (m * Vx) * 1 / (1 + ((state(2,i+1) - state(4,i+1) * lr) / Vx)^2);
                    0, 0, 0, 1;
                    0, -lf * Cf / (I * Vx) * 1 / (1 + ((state(2,i+1) + state(4,i+1) * lf) / Vx)^2) + lr * Cr / (I * Vx) * 1 / (1 + ((state(2,i+1) - state(4,i+1) * lr) / Vx)^2), ...
                        0, -lf^2 * Cf / (I * Vx) * 1 / (1 + ((state(2,i+1) + state(4,i+1) * lf) / Vx)^2) - lr^2 * Cr / (I * Vx) * 1 / (1 + ((state(2,i+1) - state(4,i+1) * lr) / Vx)^2)];
            
            % dLdx = [state(1,i+1), 0, state(3,i+1), 0];
            dLdx = [state(1,i+1), 0, 0, 0];
            costate(:,i) = costate(:,i+1) - dt * (-dfdx' * costate(:,i+1) - dLdx');
        end
        
        v = [0, Cf / m, 0, lf * Cf / I] * costate;
        
        k = 0;
        while J(delta - beta^k * v,dt,tspan, currState, parameters) - J(delta,dt,tspan, currState, parameters) > ...
                -alpha * beta^k * sum(v.^2) * dt
            k = k + 1;
        end
        
        delta = delta - beta^k * v;
    end

    delta = delta(1);

end

function j = J(delta, dt, tspan, initState, parameters)
    Cf = parameters.Cf;
    Cr = parameters.Cr;
    lf = parameters.lf;
    lr = parameters.lr;
    I = parameters.I;
    m = parameters.m;
    Vx = parameters.Vx;

    state = zeros(5,length(tspan));
    state(:,1) = initState;

    for i = 2:length(tspan)
        state(1,i) = state(1,i-1) + dt * (Vx * sin(state(3,i-1)) + state(2,i-1) * cos(state(3,i-1)));
        state(2,i) = state(2,i-1) + dt * (-Vx * state(4,i-1) - ...
            Cf / m * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) - ...
            Cr / m * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
        state(3,i) = state(3,i-1) + dt * (state(4,i-1));
        state(4,i) = state(4,i-1) + dt * (-lf * Cf / I * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) + ...
            lr * Cr / I * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
        state(5,i) = state(5,i-1) + dt * (Vx * cos(state(3,i-1)) + state(2,i-1) * sin(state(3,i-1)));
    end

    % j = 0.5 * sum(state(1,:).^2 + state(3,:).^2) * dt;
    j = 0.5 * sum(state(1,:).^2) * dt;
end