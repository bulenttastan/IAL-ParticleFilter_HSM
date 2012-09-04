clear; close all; clc;
load '../data/scenario1';
addpath('..');
wh=1;
X=X(wh,:)';Y=Y(wh,:)';T=T(wh,:)';
OX=OX(wh,:)';OY=OY(wh,:)';
figure('Position',[400,100,800,800]); hold on;
axis([-15,35,-15,35],'square'); %[-9,9,0,18]

[b,kg,c1,c2,ko,c3,c4,V]=params();
[t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
x=y(:,3);
y=y(:,4);

plot(OX,OY,'ro');
[vx,vy]=voronoi(OX,OY);
for i=1:18
    plot([vx(1,i) vx(2,i)],[vy(1,i) vy(2,i)]);
    waitforbuttonpress;
end
plot(X,Y,'r')
plot(x,y,'g')

% legend('Voronoi','SecondLife','Fajen-Warren','Location', 'NW');