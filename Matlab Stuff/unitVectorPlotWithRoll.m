close;

%filename = input('Filename: ');

load vec.dat
h = length(vec);

hold on;
xlim([-1.2 1.2])
ylim([-1.2 1.2])
zlim([-1.2 1.2])
title("System Animation")

vectorLine = plot3([0,1],[0,1],[0,1], 'r', 'Linewidth', 2);
rollLine = plot3([0,0],[0,0],[0,0], 'b', 'Linewidth', 1); 
view(45,45);
grid on;

for ii = 1:h
    set(vectorLine,'XData',[0,vec(ii,1)],'YData',[0,vec(ii,2)],'ZData',[0,vec(ii,3)]);
    set(rollLine,'XData',[0,vec(ii,4)]/2,'YData',[0,vec(ii,5)]/2,'ZData',[0,vec(ii,6)]/2);
    pause(0.05)
    drawnow
end
