function output = Saturate( input, min, max )

    output = input;

    if (output < min)
        output = min;
    end
    if (output > max)
        output = max;
    end

end