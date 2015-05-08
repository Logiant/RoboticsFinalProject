% ----------------------------------------------------------------------
% Main File   : GetNearestMellon.m
% Source Files: distance.m
% Description : Calculates a cubic spline trajectory given an initial and
%               final position, velocity, and time
% Inputs: mellonArray - a 2xN array of (x, y) mellon position
%         position - a 2x1 array of the robot's current position
% Outputs: index - row of the nearest mellon
% Author: Logan Beaver
% Date: 5/8/2015
% Bugs: none
% ----------------------------------------------------------------------
function index = GetNearestMellon(positionArray, position)
    %nearest position and its index
    nearest = inf; index = -1;
    %Get number of positions in the array
    num = size(positionArray); num = num(2);
    %initial position is the robot's position
    x0 = position(1); y0 = position(2);
    %find nearest distance
    for i = 1:num
        x1 = positionArray(1, i); y1 = positionArray(2, i);
        d = distance(x0, y0, x1, y1);
        if (d < nearest)
           index = i;
           nearest = d;
        end
    end
end