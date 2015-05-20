% ----------------------------------------------------------------------
% Main File   : FinalProject.m
% Source Files: GetNearestMellon.m, CalcSpline.m distance.m
% Description : Final Project Template for ME498 - Robotics
% Author: Logan Beaver, Justin Collins
% Date: 5/3/2015
% Bugs: none
% -------------------------------------------------------------------
clear; clc; close all;
wheelRadius = 0.2; %in
wheelSpacing = 0.5; %in
numMellons = 5; %number of mellons to smash
speed = 2; %average robot speed in in/s
index = 1;
%simulink variables
kp = 125;ki = 25;kd = 10;
N = 1;
Bm = 0.005;
Jm = 5e-4; %J1 = 1;
m1 = 0.9/32; %g1 = 1; L1 = 1;
B1 = 0.01;
Jw=1e-3; mc=4.4/32; m2=2.6/32; m3=0.9/32; dc=3.5;
%stabbing offset
offset = 0.02; %in

%calculate a 2xN matrix of mellon (X, Y) positions
mellonsPos = (rand(2,numMellons) - 0.5) * 2; %between -1 and 1
mellonStorage = mellonsPos;
position = [0; 0]; %Robot position
orientation = 0; %Robot rotation, radians
time = 0; %initial time is 0
leftPhi = 0; %the car starts parked, you see
rightPhi = 0; %the car starts parked, you see
figure(1);
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
    %generate the trajectory to the target mellon
    dPosition = distance(position(1), position(2), targetPos(1), targetPos(2));
    
    dTheta = atan2(targetPos(2) - position(2), targetPos(1) - position(1)) - orientation;
    
    dt = abs(dPosition / speed); %calculate time to keep the average speed
    %convert dTheta to a wheel average difference (0 when forward)
    wheelDiff = dTheta * wheelSpacing/wheelRadius * 2; %phiL - phiR
    %convert dPosition to a wheel average sum (0 when turning)
    wheelSum = dPosition * 2 / wheelRadius; %phiL + phiR
    
    %calculate change in wheel angle from wheel average sum and difference
    deltaLeftPhiSpin = (wheelSum*0 + wheelDiff) / 2;
    deltaRightPhiSpin = (wheelSum*0 - wheelDiff) / 2;
    
    
    %calculate left coefficients
    leftCoeffs((q-1)*2 + 1,:) = CalcSpline(time,leftPhi,0, ...
        time + dt/2,leftPhi + deltaLeftPhiSpin,0);
    %calculate right wheel coefficients
    rightCoeffs((q-1)*2 + 1,:) = CalcSpline(time,rightPhi,0, ...
        time + dt/2,rightPhi + deltaRightPhiSpin,0);
    %update variables
    leftPhi = polyval(leftCoeffs((q-1)*2 + 1,:), time + dt/2); %actual value from simulink
    rightPhi = polyval(rightCoeffs((q-1)*2 + 1,:), time + dt/2); %actual value from simulink
    position = position; %calculated from actualPhis
    
    time = time + dt/2;
    
    tArray((q-1)*2 + 1) = time;
    
    deltaLeftPhiDrive = (wheelSum + wheelDiff*0) / 2;
    deltaRightPhiDrive = (wheelSum - wheelDiff*0) / 2;
    
    %calculate left coefficients
    leftCoeffs((q-1)*2 + 1 +1,:) = CalcSpline(time,leftPhi,0, ...
        time + dt/2,leftPhi + deltaLeftPhiDrive,0);
    %calculate right wheel coefficients
    rightCoeffs((q-1)*2 + 1+1,:) = CalcSpline(time,rightPhi,0, ...
        time + dt/2,rightPhi + deltaRightPhiDrive,0);
    %update variables
    leftPhi = polyval(leftCoeffs((q-1)*2 + 1+1,:), time + dt/2); %actual value from simulink
    rightPhi = polyval(rightCoeffs((q-1)*2 + 1+1,:), time + dt/2); %actual value from simulink
    position = targetPos; %calculated from actualPhis
    orientation = orientation + dTheta;
    
    time = time + dt/2;
    tArray((q-1)*2 + 1+1) = time;
    
    
end

%run simulation for left wheel
wCoeff = leftCoeffs
sim('wheel1', [0 time]);
%get leftTheta vector from simulink
leftTime = thetaActual.Time;
leftAngle = thetaActual.Data;

%run simulation for right wheel
wCoeff = rightCoeffs;
sim('wheel1', [0 time]);
%get rightTheta vector from simulink
rightTime = thetaActual.Time;
rightAngle = thetaActual.Data;

figure(2);
plot(leftTime, leftAngle, '-k', rightTime, rightAngle, '-b');
hold on;

tArray = [0 tArray];
for k = 1:length(tArray) - 1
    plot(tArray(k:k+1), polyval(leftCoeffs(k,:), tArray(k:k+1)),'or');
    plot(tArray(k:k+1), polyval(rightCoeffs(k,:), tArray(k:k+1)), 'ob');
end

%%%Fixed DH Params, units of inches and radians

% Origin to the coordinate frame of the cart
d1=5; alph1=0; %theta and a both variable

% Cart frame to base link
d2=6.43; a2=0; alph2=pi()/2; %theta variable

% Base link to link 2
d3=0; alph3=0; a3=2.75; %theta variable

%Link 2 to spear tip
alph4=0; a4=0; th4=0; %d variable

%Animate motion
figure(1); clf;
axis([-1 1 -1 1]);
axis manual;
hold on;
for i = 1:(length(tArray) - 1)/2
    plot(mellonStorage(1,i), mellonStorage(2,i), '*g');
end
position = [0;0];
orientation = 0;
lTot = 0;
rTot = 0;
for n=1:min(length(leftAngle), length(rightAngle))
    %calculate new robot position
    
    if n > 1
        dLeft = leftAngle(n) - leftAngle(n-1);
        dRight = rightAngle(n) - rightAngle(n-1);
        if dLeft ~= dRight
            lTot = lTot + dLeft;
            rTot = rTot + dRight;
        end
        dR = wheelRadius/2*(dLeft+dRight);
        dPhi = (wheelRadius/wheelSpacing)*(dLeft - dRight)/2;
        dx = dR * cos(orientation + dPhi/2);
        dy = dR * sin(orientation + dPhi/2);
        orientation = orientation + dPhi;
        orientation * 180 / pi
        position = [position(1) + dx; position(2) + dy];
        plot(position(1), position(2), '.r');
        
        pause(.1);
    end
end
orientation*180/pi