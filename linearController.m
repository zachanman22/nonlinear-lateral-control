function delta = linearController(state, parameters)
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
                        state(2);
                        state(3);
                        state(4)];
    
    % Klqr = lqr([0, 1; ...
    %             0, 0], ...
    %            [0; ...
    %             1], ...
    %            [10, 0; ...
    %             0, 1], ...
    %             1);

    Klqr = [3.16227766016838, 2.70639156818387, 4, 5];

    delta1 = -Klqr * linearizedStates;
    % delta1 = 0;

    % delta = m / (Cf) * (-Vx * state(4) - ...
    %     (-Vx * state(4) - Cf / m * (state(2) + state(4) * lf/ Vx) - ...
    %     Cr / m * (state(2) - state(4) * lr/ Vx)) + delta1);

    delta = delta1;
end