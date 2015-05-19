% ----------------------------------------------------------------------
% Main File   : FinalProject.m
% Source Files: GetNearestMellon.m, CalcSpline.m distance.m
% Description : Final Project Template for ME498 - Robotics
% Author: Logan Beaver, Justin Collins
% Date: 5/3/2015
% Bugs: none
% -------------------------------------------------------------------
clear; clc;
wheelRadius = 0.2; %m
wheelSpacing = 0.5; %m
numMellons = 5; %number of mellons to smash
speed = 2; %average robot speed in m/s
index = 1;
%simulink variables
N = 1;
Bm = 1;
Jm = 1; J1 = 1;
m1 = 1; g1 = 1; L1 = 1; B1 = 1;
Jw=1; mc=1; m2=1; m3=1; dc=1;

%calculate a 2xN matrix of mellon (X, Y) positions
mellonsPos = (rand(2,numMellons) - 0.5) * 2; %between 0 and 1
position = [0; 0]; %Robot position
time = 0; %initial time is 0
leftPhi = 0; %the car starts parked, you see
rightPhi = 0; %the car starts parked, you see
figure(1);
clf();
hold on;
plot(position(1), position(2), 'or');
for i = 1:numMellons
    plot(mellonsPos(1,i), mellonsPos(2,i), '*g');
end

for q = 1:numMellons
    %get next target mellon
    nearestIndex = GetNearestMellon(mellonsPos, position); %get nearest
    targetPos = mellonsPos(:, nearestIndex); %target nearest mellon
    mellonsPos(:, nearestIndex) = []; %remove nearest mellon from storage
    numMellons = numMellons - 1; %decrement mellon array size
    %generate the trajectory to the target mellon
    dPosition = distance(position(1), position(2), targetPos(1), targetPos(2));
    dTheta = atan2(targetPos(1) - position(1), targetPos(2) - position(2));
    dt = dPosition / speed; %calculate time to keep the average speed
    %convert dTheta to a wheel average difference
    wheelDiff = dTheta*wheelSpacing/wheelRadius; %phiL - phiR
    %convert dPosition to a wheel average sum
    wheelSum = dPosition * 2 / wheelRadius; %phiL + phiR
    %calculate change in wheel angle from wheel average sum and difference
    deltaLeftPhi = (wheelSum + wheelDiff) / 2;
    deltaRightPhi = (wheelSum - wheelDiff) / 2;
    
   % set_param('wheelControl', 'StopTime', dt)    
    
    %calculate left coefficients
    leftCoeffs = CalcSpline(time,leftPhi,0, ...
                        time + dt,leftPhi + deltaLeftPhi,0);
    
    %run simulation for left wheel
    wheelCoeff = leftCoeffs;
    wCoeff = wheelCoeff;
    sim('wheel1');
    
    leftTime = thetaActual.Time;
    leftAngle = thetaActual.Data;
    
    %get leftTheta vector from simulink

    %calculate right wheel coefficients
    rightCoeffs = CalcSpline(time,rightPhi,0, ...
                        time + dt,rightPhi + deltaRightPhi,0);
    
    %run simulation for right wheel
    wCoeff = rightCoeffs;
    wheel1();
    %get rightTheta vector from simulink
    
    %calculate position and orientation from left and right theta values
    
    %update variables
    leftPhi = leftAngle(length(leftAngle)); %actual value from simulink
    rightPhi = rightPhi; %actual value from simulink
    position = targetPos; %calculated from actualPhis
    figure(2);
    hold on;
    plot(leftTime, leftAngle, '-b');
    plot(time, polyval(leftCoeffs, time), 'or', ...
        time+dt, polyval(leftCoeffs, time+dt), 'or');
    
    time = time + dt;

end