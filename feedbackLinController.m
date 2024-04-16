function delta = feedbackLinController(state, parameters)
% Working
    Cf = parameters.Cf;
    Cr = parameters.Cr;
    lf = parameters.lf;
    lr = parameters.lr;
    I = parameters.I;
    m = parameters.m;
    Vx = parameters.Vx;

    epsilon = 0.001;
    
    linearizedStates = [state(1);
                        Vx * sin(state(3)) + state(2) * cos(state(3))];
    
    % Klqr = lqr([0, 1; ...
    %             0, 0], ...
    %            [0; ...
    %             1], ...
    %            [10, 0; ...
    %             0, 0], ...
    %             1);

    Klqr = [3.16227766016838, 2.51486685936587];

    delta1 = -Klqr * linearizedStates;
    % delta1 = 0;

    delta = m / (Cf * cos(state(3))) * (-Vx * cos(state(3)) * state(4) + state(2) * sin(state(3)) * state(4) - ...
        cos(state(3)) * (-Vx * state(4) - Cf / m * atan2(state(2) + state(4) * lf, Vx) - ...
        Cr / m * atan2(state(2) - state(4) * lr, Vx)) + delta1);
    
end