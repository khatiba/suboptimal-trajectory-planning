function [value, isterminal, direction] = IntEvents(t,y,P)   

    value = 1;        % Initialize the trigger
    isterminal = 1;   % Stop the integration
    direction = -1;   % Negate the direction

    V = y(4);   % Velocity

    % Detect when the velocity drops below the minimum required
    if (V(end) < 450)
        value = 0;
    end

end