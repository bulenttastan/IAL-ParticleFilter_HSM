clc; close all; clear;

figure(); hold on; axis([-2 4 -1 13], 'equal');
XY=[2 12];
OX=[];
OY=[];
lw=2;

plot(0,0,'.r');
plot(XY(1),XY(2),'+r','LineWidth',lw);


[b,kg,c1,c2,ko,c3,c4,V]=params();
[t,y]=observer(XY(1),XY(2),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,1:.2:5);
x=y(:,3); y=y(:,4);
size(x)

plot(x,y,'r','LineWidth',lw);

plot(x(38),y(38),'ok','LineWidth',6);