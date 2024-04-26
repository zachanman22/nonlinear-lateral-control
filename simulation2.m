function state = simulation2(initialState, controllerID, localOutput)

    %% Parameters Control
    Cf = 60000;
    Cr = 60000;
    lf = 1.22;
    lr = 1.62;
    I = 2920;
    m = 1590;
    Vx = 10;
    
    parameters = struct('Cf',Cf, 'Cr', Cr, 'lf', lf, 'lr', lr, ...
                        'I', I, 'm', m, 'Vx', Vx);
    
    dt = 0.001;
    tspan = 0:dt:10;
    
    
    %% Initial State Setup
    state = zeros(5,length(tspan));
    % integrators = zeros(2,length(tspan));
    
    state(1) = initialState(1);
    state(2) = initialState(2);
    state(3) = initialState(3);
    state(4) = initialState(4);
    state(5) = initialState(5);
    
    delta = zeros(1,length(tspan));
    
    % state(:,1) = [1.2; -1; 0.5 * pi / 180; -0.5 * pi / 180; 0];
    
    for i = 2:length(tspan)
    
        %% Controlers

        if(controllerID == 0)
            delta(i-1) = linearController(state(:,i-1), parameters);
        elseif (controllerID == 1)
            delta(i-1) = optimalController(state(:,i-1),parameters,dt);
        elseif (controllerID == 2)
            delta(i-1) = feedbackLinController(state(:,i-1),parameters);
        elseif (controllerID == 3)
            delta(i-1) = backsteppingController2(state(:,i-1),parameters);
        elseif (controllerID == 4)
            delta(i-1) = backsteppingController2(state(:,i-1),parameters);
        end
    
        % delta(i-1) = feedbackLinController(state(:,i-1), parameters);
        % delta(i-1) = sontagController(state(:,i-1), parameters);
        % delta(i-1) = boundedSontagController(state(:,i-1), parameters);
        % delta(i-1) = backsteppingController(state(:,i-1), parameters);
        % delta(i-1) = backsteppingController2(state(:,i-1), parameters);
        % delta(i-1) = backsteppingController3(state(:,i-1), parameters);
        
        % delta(i-1) = optimalController(state(:,i-1),parameters,dt);
    
        %% Non-Linear States
        state(1,i) = state(1,i-1) + dt * (Vx * sin(state(3,i-1)) + state(2,i-1) * cos(state(3,i-1)));
        state(2,i) = state(2,i-1) + dt * (-Vx * state(4,i-1) - ...
            Cf / m * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) * cos(delta(i-1)) - ...
            Cr / m * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
        state(3,i) = state(3,i-1) + dt * (state(4,i-1));
        state(4,i) = state(4,i-1) + dt * (-lf * Cf / I * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) + ...
            lr * Cr / I * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
        state(5,i) = state(5,i-1) + dt * (Vx * cos(state(3,i-1)) + state(2,i-1) * sin(state(3,i-1)));
    end
    
    if localOutput
        finalSquareCost = Jsquare(state, dt);
        finalAbsoluteCost = Jabsolute(state, dt);
        [minAbsoluteError, maxAbsoluteError] = Jextremes(state);
        
        figure(1)
        subplot(3,1,1)
        plot(tspan, state(2,:))
        title("States over time")
        ylabel("y Velocity")
        subplot(3,1,2)
        plot(tspan, state(3,:))
        ylabel("Yaw Angle")
        subplot(3,1,3)
        plot(tspan, state(4,:))
        ylabel("Yaw Rate")
        xlabel("Time")
    
        figure(2)
        plot(state(5,:), state(1,:))
        xlabel("Global x")
        ylabel("Global y")
        title("Trajectory in XY plane")
        xlim([0,100])
        ylim([-10,10])
    
        figure(3)
        plot(tspan, delta)
        title("Control over time")
        ylabel("Steering Angle")
        xlabel("Time")
    
        % figure(4)
        % plot(tspan, atan2(state(2,:),Vx))
        
        finalSquareCost = Jsquare(state, dt);
        finalAbsoluteCost = Jabsolute(state, dt);
        [minAbsoluteError, maxAbsoluteError] = Jextremes(state);
    end
    
    % state(2,i) = state(2,i-1) + dt * (-Vx * state(4,i-1) - ...
    % Cf / m * atan((state(2,i-1) + state(4,i-1) * lf) / Vx) - ...
    % Cr / m * atan((state(2,i-1) - state(4,i-1) * lr) / Vx));
    
    % state(4,i-1) + dt * (-lf * Cf / I * atan((state(2,i-1) + state(4,i-1) * lf) / Vx) + ...
    % lr * Cr / I * atan((state(2,i-1) - state(4,i-1) * lr) / Vx));
end