function step = nanCheck(step)
    for i = 1:1:length(step)
        if (isnan(step(i)))
            step(i) = 0;
        end
    end
end