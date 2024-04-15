function delta = backsteppingController2(state, parameters)
% Working
    Cf = parameters.Cf;
    Cr = parameters.Cr;
    lf = parameters.lf;
    lr = parameters.lr;
    I = parameters.I;
    m = parameters.m;
    Vx = parameters.Vx;

    k = 1;

    yglobalDot = Vx * sin(state(3)) + state(2) * cos(state(3));

    alpha = (-state(1) - Vx * sin(state(3))) / cos(state(3));

    % alphaDot = -(yglobalDot * cos(state(3)) + state(1) * sin(state(3)) * state(4)) / ...
    %     cos(state(3))^2;

    alphaDot = -(yglobalDot * cos(state(3)) + state(1) * sin(state(3)) * state(4)) / ...
        cos(state(3))^2 - Vx * sec(state(3))^2 * state(4);
    
    G = cos(state(3));
    dVdnuG = state(1) * G;

    z = state(2) - alpha;

    delta = m / Cf * (Vx * state(4) + Cf / m * atan2(state(2) + state(4) * lf, Vx) + ...
        Cr / m * atan2(state(2) - state(4) * lr, Vx) + alphaDot - dVdnuG - k * z);
end