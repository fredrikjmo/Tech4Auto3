function y = signalGen(t)
    % Check conditions and assign output
    if t < 26
        y = 0;
    elseif t >= 26 && t < 34
        y = -250*t +6500;
    elseif t >= 34 && t < 64
        y = -2000;
    elseif t >= 64 && t < 72
        y = 250*t -18000;
    elseif t >= 72 && t < 135
        y = 0;
    elseif t >= 135 && t < 140
        y = 400*t -54000;
    elseif t >= 140 && t < 170
        y = 2000;
    elseif t >= 170 && t < 178
        y = -250*t+44500; 
    else
        y=0;
    end
end