% ----------------------------------------------------------------------
% Main File   : FinalProject.m
% Source Files: GetNearestMellon.m, CalcTrajectory.m
% Description : Final Project Template for ME498 - Robotics
% Author: Logan Beaver, Justin Collins
% Date: 5/3/2015
% Bugs: none
% -------------------------------------------------------------------
clear; clc;
numMellons = 5; %number of mellons to smash
speed = 2; %average robot speed in m/s
index = 1;

%calculate a 2xN matrix of mellon (X, Y) positions
mellonsPos = (rand(2,numMellons) - 0.5) * 2; %between 0 and 1
position = [0; 0]; %Robot position
time = 0; %initial time is 0
figure(1);
clf();
hold on;
plot(position(1), position(2), 'or');
for i = 1:numMellons
    plot(mellonsPos(1,i), mellonsPos(2,i), '*g');
end

for q = 1:numMellons
    %generate trajectory
    nearestIndex = GetNearestMellon(mellonsPos, position); %get nearest
    targetPos = mellonsPos(:, nearestIndex); %target nearest mellon
    mellonsPos(:, nearestIndex) = []; %remove nearest mellon from storage
    numMellons = numMellons - 1; %decrement mellon array size
    distanceToMellon = distance(position(1), position(2), ...
        targetPos(1), targetPos(2));
    dt = distanceToMellon / speed; %calculate time to keep the average speed
    coeffs = CalcTrajectory(time, position(1), 0, position(2), 0, ...
        time+dt, targetPos(1), 0, targetPos(2), 0);
    
    
    
    xC = coeffs(:,1); yC = coeffs(:,2);
    for t = time:0.01:time+dt
        x(index) = xC(1) + xC(2)*t + xC(3)*t*t + xC(4)*t*t*t;
        y(index) = yC(1) + yC(2)*t + yC(3)*t*t + yC(4)*t*t*t;
        tm(index) = t;
        index = index + 1;
    end
    time = time + dt;
    position = targetPos;
end

plot(x, y);
figure(2);
plot(tm, x);
figure(3);
plot(tm, y);
