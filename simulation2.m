
%% RNG MANIP
rng(42)

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
integrators = zeros(2,length(tspan));

% -1.8 to 1.8 m lateral position
state(1) = 3.6 * rand(1) - 1.8;
%state(1) = 0;
% -1.8 to 1.8 m/s lateral velocity
state(2) = 3.6 * rand(1) - 1.8;
%state(2) = 0;
% -5 to 5 degrees error but in radians
state(3) = 10 * pi / 180 * rand(1) - 5 * pi / 180;
%state(3) = 0;
% -5 to 5 degrees/s error but in radians
state(4) = 10 * pi / 180 * rand(1) - 5 * pi / 180;
% state(4) = 0;
state(5) = 0;

delta = zeros(1,length(tspan));

% state(:,1) = [1.2; -1; 0.5 * pi / 180; -0.5 * pi / 180; 0];

for i = 2:length(tspan)

    %% Controlers

    % delta(i-1) = feedbackLinController(state(:,i-1), parameters);
    % delta(i-1) = sontagController(state(:,i-1), parameters);
    % delta(i-1) = boundedSontagController(state(:,i-1), parameters);
    % delta(i-1) = backsteppingController(state(:,i-1), parameters);
    % delta(i-1) = backsteppingController2(state(:,i-1), parameters);
    % delta(i-1) = backsteppingController3(state(:,i-1), parameters);
    delta(i-1) = linearController(state(:,i-1), parameters);

    %% Non-Linear States
    % state(1,i) = state(1,i-1) + dt * (Vx * sin(state(3,i-1)) + state(2,i-1) * cos(state(3,i-1)));
    % state(2,i) = state(2,i-1) + dt * (-Vx * state(4,i-1) - ...
    %     Cf / m * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) * cos(delta(i-1)) - ...
    %     Cr / m * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
    % state(3,i) = state(3,i-1) + dt * (state(4,i-1));
    % state(4,i) = state(4,i-1) + dt * (-lf * Cf / I * (atan2(state(2,i-1) + state(4,i-1) * lf, Vx) - delta(i-1)) + ...
    %     lr * Cr / I * atan2(state(2,i-1) - state(4,i-1) * lr, Vx));
    % state(5,i) = state(5,i-1) + dt * (Vx * cos(state(3,i-1)) + state(2,i-1) * sin(state(3,i-1)));

    %% Linear States

    state(1, i) = state(1, i-1) + dt * (state(2, i-1));
    state(2, i) = state(2, i-1) + dt * (-Vx*state(4, i-1) - Cf/m * (state(2, i-1) + state(4, i-1)*lf)/Vx - Cr/m * (state(2, i-1) - state(4, i-1)*lr)/Vx + Cf/m * delta(i-1));
    state(3, i) = state(3, i-1) + dt * (state(4, i-1));
    state(4, i) = state(4, i-1) + dt * (-lf*Cf/I*(state(2, i-1) + state(4, i-1)*lf)/Vx + lr*Cr/I*(state(2, i-1) - state(4, i-1)*lr)/Vx + lf*Cf/I*delta(i-1));
    state(5, i) = state(5, i-1) + dt * Vx;
end

figure(1)
subplot(3,1,1)
plot(tspan, state(2,:))
subplot(3,1,2)
plot(tspan, state(3,:))
subplot(3,1,3)
plot(tspan, state(4,:))

figure(2)
plot(state(5,:), state(1,:))
xlim([0,100])
ylim([-10,10])

figure(3)
plot(tspan, delta)

figure(4)
plot(tspan, atan2(state(2,:),Vx))


% state(2,i) = state(2,i-1) + dt * (-Vx * state(4,i-1) - ...
% Cf / m * atan((state(2,i-1) + state(4,i-1) * lf) / Vx) - ...
% Cr / m * atan((state(2,i-1) - state(4,i-1) * lr) / Vx));

% state(4,i-1) + dt * (-lf * Cf / I * atan((state(2,i-1) + state(4,i-1) * lf) / Vx) + ...
% lr * Cr / I * atan((state(2,i-1) - state(4,i-1) * lr) / Vx));