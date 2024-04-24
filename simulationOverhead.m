yInitRange = -1:0.1:1;
thetaInitRange = -9:9;

finalSquareCost = zeros(21,19);
finalAbsoluteCost = zeros(21,19);
minAbsoluteError = zeros(21,19);
maxAbsoluteError = zeros(21,19);
finalOutput = zeros(21,19);
convergeTimes = zeros(21,19);


model = 0;
j=0;
for yInit = yInitRange
    j = j + 1;
    i = 0;
    yInit
    for thetaInit = thetaInitRange
        i = i+1;
        %% Set Initial States
        initStates = zeros(5,1);
        % -1.8 to 1.8 m lateral position
        initStates(1,1) = yInit;
        % state(1) = 0;
        % -1.8 to 1.8 m/s lateral velocity
        initStates(2,1) = 0;
        %state(2) = 0;
        % -5 to 5 degrees error but in radians
        initStates(3,1) = thetaInit/180 * pi;
        % state(3) = 0;
        % -5 to 5 degrees/s error but in radians
        initStates(4,1) = 0;
        % state(4) = 0;
        initStates(5,1) = 0;


        %% Run Simulation
        stateOut = simulation2(initStates, model, false);
        convergeTimes(j, i) = endTime(stateOut, 0.001);
        finalOutput(j, i) = stateOut(1,10001);
        % finalSquareCost(thetaInit+6, model+1) = Jsquare(stateOut, 0.001);
        % finalAbsoluteCost(thetaInit+6, model+1) = Jabsolute(stateOut, 0.001);
        % [minAbsoluteError(thetaInit+6, model+1), maxAbsoluteError(thetaInit+6, model+1)] = Jextremes(stateOut);
    end
end