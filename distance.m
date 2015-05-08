% ----------------------------------------------------------------------
% Main File   : distance.m
% Source Files: None
% Description : calculates the distance between (x0, y0) and (x1, y1)
% Inputs: x0, y0 - position 0. x1, y1 - position 1
% Outputs: d - distance between points
% Author: Logan Beaver
% Date: 5/8/2015
% Bugs: none
% ----------------------------------------------------------------------
function d = distance(x0, y0, x1, y1)
    d = sqrt((x1-x0)^2 + (y1-y0)^2);
end