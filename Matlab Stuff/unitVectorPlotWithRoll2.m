%Clears figures, variables, and terminal before plotting
clc;
clear;
close;

%Loading the data file into the program
%format of "   x   y   z   x2   x2   z2"
%Secont vector is the roll orientation vector
filename = input("Filename: ", 's');
load('vec.dat');
h = length(vec);

%Defines the unit vector which defines the real upwards (Z) direction
newUpVector = [0,0,1];
newUpVector = newUpVector/norm(newUpVector);

%obtains a perpindicular unit vector to the upwards direction
newYVector = [0, 1, (-newUpVector(2)/newUpVector(3))];
newYVector = newYVector/norm(newYVector);

%finds the last unit vector for the axis based on the first two
newXVector = cross(newYVector,newUpVector);

%creates a direction cosine matrix based on the new axis
calDCM = [newXVector(1), newYVector(1), newUpVector(1);...
    newXVector(2), newYVector(2), newUpVector(2);...
    newXVector(3), newYVector(3), newUpVector(3);];

%separates the direction and roll vectors out of the raw data
dirVecCal = zeros(h,3);
rolVecCal = zeros(h,3);

%applies DCM to the direction vector
dirVec = vec(:,1:3);
dirVecCal(:,1) = dirVec(:,1)*calDCM(1,1) + dirVec(:,2) * calDCM(1,2)...
    + dirVec(:,3) * calDCM(1,3);
dirVec(:,2) = dirVec(:,1)*calDCM(2,1) + dirVec(:,2) * calDCM(2,2)...
    + dirVecCal(:,3) * calDCM(2,3);
dirVecCal(:,3) = dirVec(:,1)*calDCM(3,1) + dirVec(:,2) * calDCM(3,2)...
    + dirVec(:,3) * calDCM(3,3);

%applies DCM to the roll vector
rolVec = vec(:,4:6);
rolVecCal(:,1) = rolVec(:,1)*calDCM(1,1) + rolVec(:,2) * calDCM(1,2)...
    + rolVec(:,3) * calDCM(1,3);
rolVecCal(:,2) = rolVec(:,1)*calDCM(2,1) + rolVec(:,2) * calDCM(2,2)...
    + rolVec(:,3) * calDCM(2,3);
rolVecCal(:,3) = rolVec(:,1)*calDCM(3,1) + rolVec(:,2) * calDCM(3,2)...
    + rolVec(:,3) * calDCM(3,3);

sideVec = -cross(dirVec,rolVec);
sideVecCal = -cross(dirVecCal,rolVecCal);

verticalLine = zeros(h,3);
verticalLine(:,3) = 1;

rolLineThrust = zeros(h,1);
sidLineThrust = zeros(h,1);

for ii = 1:h
    rolLineThrust(ii) = -dot(rolVecCal(ii,1:3),verticalLine(ii,1:3));
    sidLineThrust(ii) = -dot(sideVecCal(ii,1:3),verticalLine(ii,1:3));

    
end
%projVec(:,1:3) = rolVec(:,1:3) - dot(rolVec(:,1:3),norm(dirVec(:,1:3)))*rolVec(:,1:3);

%Creates first plot
%this plot contains the unmodified direction and roll data
subplot(1,3,1);
hold on;
xlim([-1.2 1.2])
ylim([-1.2 1.2])
zlim([-1.2 1.2])
title("System Animation")

% oldUpVecLine = plot3([0,0],[0,0],[0,1], 'c', 'Linewidth', 2);
% oldYVecLine = plot3([0,0],[0,1],[0,0], 'g', 'Linewidth', 2);
% oldXVecLine = plot3([0,1],[0,0],[0,0], 'g', 'Linewidth', 2);

%shows the new axis on the first plot
newUpVecLine = plot3([0,newUpVector(1)],[0,newUpVector(2)],[0,newUpVector(3)], 'm', 'Linewidth', 2);
newYVecLine = plot3([0,newYVector(1)],[0,newYVector(2)],[0,newYVector(3)], 'y', 'Linewidth', 2);
newXVecLine = plot3([0,newXVector(1)],[0,newXVector(2)],[0,newXVector(3)], 'y', 'Linewidth', 2);

%initialized plot objects for the first plot
vectorLine = plot3([0,1],[0,1],[0,1], 'r', 'Linewidth', 2);
rollLine = plot3([0,0],[0,0],[0,0], 'b', 'Linewidth', 1);
sideLine = plot3([0,0],[0,0],[0,0], 'g', 'Linewidth', 1);

view(45,45);
grid on;

%Creates second plot
%Contains data adjusted for the new axis system
subplot(1,3,2);
hold on;
xlim([-1.2 1.2])
ylim([-1.2 1.2])
zlim([-1.2 1.2])
title("System Animation Calibrated")
vectorLineCal = plot3([0,1],[0,1],[0,1], 'r', 'Linewidth', 2);
rollLineCal = plot3([0,0],[0,0],[0,0], 'b', 'Linewidth', 1);
sideLineCal = plot3([0,0],[0,0],[0,0], 'g', 'Linewidth', 1);
view(45,45);
grid on;

subplot(1,3,3);
hold on;
xlim([-1.2 1.2])
ylim([-1.2 1.2])
zlim([-1.2 1.2])
title("System Animation Calibrated");
thrustTrace = plot([0,0],[0,0], 'b');
thrustPoint = scatter([0],[0],25,'r');

%loop contains animation code for the plots
for ii = 1:h
    %updates the objects in the first plot
    set(vectorLine,'XData',[0,vec(ii,1)],'YData',[0,vec(ii,2)],'ZData',[0,vec(ii,3)]);
    set(rollLine,'XData',[0,vec(ii,4)]/2,'YData',[0,vec(ii,5)]/2,'ZData',[0,vec(ii,6)]/2);
    set(sideLine,'XData',[0,sideVec(ii,1)/2],'YData',[0,sideVec(ii,2)/2],'ZData',[0,sideVec(ii,3)/2]);
    
    %updates the objects in the second plot
    set(vectorLineCal,'XData',[0,dirVecCal(ii,1)],'YData',[0,dirVecCal(ii,2)],'ZData',[0,dirVecCal(ii,3)]);
    set(rollLineCal,'XData',[0,rolVecCal(ii,1)]/2,'YData',[0,rolVecCal(ii,2)]/2,'ZData',[0,rolVecCal(ii,3)]/2);
    set(sideLineCal,'XData',[0,sideVecCal(ii,1)]/2,'YData',[0,sideVecCal(ii,2)]/2,'ZData',[0,sideVecCal(ii,3)]/2);
    
    set(thrustTrace, 'XData',rolLineThrust(1:ii),'YData',sidLineThrust(1:ii));
    set(thrustPoint, 'XData',rolLineThrust(ii),'YData',sidLineThrust(ii));
    
    
    %1/20 second delay and draw command
    %hopefully maintians 20 fps
    pause(0.1)
    drawnow
end
