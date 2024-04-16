function [jmin,jmax] = Jextremes(state)
    jmin = min(abs(state(1,:)));
    jmax = max(abs(state(1,:)));
end