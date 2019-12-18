function phiSun = ComputephiRover (data, t)
    % The following function computes the angle throught the whole control
    % horizon, for timesteps of size ts
    % ------------------------------------------------------------
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % NEED TO DO THE THING THAT I HAVE AT HOME WITH THE LENGTH AND THE
    % ANGLES
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     xRover = f(0:ts:Tcontrol);    % x 
    
    % Implementation with a look-up table:
    % iteration     Phi_180    Phi_360
    %--------------------------------------
    %   1             0           0
    %   2             -10         350
    %   ...           ...         ...
%     if (rango == 0)
%         phiSun = data (iter, 1);
%     elseif (rango == 1)
%         phiSun = data (iter, 2);
%     else
%         error('The range is not correct');
%     end
    
    %% Implementation solving the angle for each time
            phiSun = data(t);
    
%     phiSun = 5*sin(f/3);
end