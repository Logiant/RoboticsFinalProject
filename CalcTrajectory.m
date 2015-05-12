% ----------------------------------------------------------------------
% Main File   : CalcTrajectory.m
% Source Files: None
% Description : Calculates a cubic spline trajectory given an initial and
%               final position, velocity, and time
% Inputs: t0, x0, v0 - initial position, velocity, acceleration
%         t1, x1, v1 - final position, velocity, acceleration
% Outputs: coeffs - coefficient matrix of the cubic polynomial
% Author: Logan Beaver
% Date: 5/8/2015
% Bugs: none
% ----------------------------------------------------------------------
function coeffs = CalcTrajectory(t0, x0, xdot0, y0, ydot0, ...
                                 tf, xf, xdotf, yf, ydotf)
    
    tMat = [1, t0, t0^2, t0^3;
            0, 1, 2*t0, 3*t0^2;
            1, tf, tf^2, tf^3;
            0, 1, 2*tf, 3*tf^2]
   
        
    invT = inv(tMat);
    
    xMat = [x0; xdot0; xf; xdotf]
    yMat = [y0; ydot0; yf; ydotf]
    
    ax = invT * xMat
    ay = invT * yMat
    
    coeffs = [ax, ay];
end