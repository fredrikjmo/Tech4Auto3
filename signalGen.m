function y = signalGen(t)
    % Check conditions and assign output
    if t < 26
        y = 0;
    elseif t >= 26 && t < 34
        y = (-250*t +6500)*10^-6;
    elseif t >= 34 && t < 64
        y = -2*10^-3;
    elseif t >= 64 && t < 72
        y = (250*t -18000)*10^-6;
    elseif t >= 72 && t < 132
        y = 0;
    elseif t >= 132 && t < 140
        y = (250*t - 33000)*10^-6;
    elseif t >= 140 && t < 170
        y = 2*10^-3;
    elseif t >= 170 && t < 178
        y = (-250*t+44500)*10^-6; 
    else
        y=0;
    end
end