clear; close all; clc; hold on;
load 'data/goalrecognition2';
tr=3;
X=X(tr,:);Y=Y(tr,:);OX=OX(tr,:);OY=OY(tr,:);T=T(tr,:);
plot(FX,FY,'xg');
plot(X,Y,'r');
plot(OX,OY,'or');
x=zeros(length(FX),length(X));

goal=2;
[b,kg,c1,c2,ko,c3,c4,V] = params();
b=4.05; kg=10; c1=0.3; c2=1; ko=198; c3=6.5; c4=.8;
for i=1:length(FX)
    [t,y]=observer(FX(i),FY(i),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
    p(i).x=y(:,3)';
    p(i).y=y(:,4)';
    plot(p(i).x,p(i).y);
end
[t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
p(goal).x=y(:,3)';
p(goal).y=y(:,4)';
plot(p(goal).x,p(goal).y);

for i=2:length(X)
    plot(X(i),Y(i),'.g');
    perc=zeros(1,length(p));
    for j=1:length(p)
        ss=sumsquare(p(j).x,p(j).y,X(1:i),Y(1:i),T(1:i));
        perc(j)=1/exp(ss);
    end
    perc=perc./sum(perc);
    disp(perc);
    waitforbuttonpress;
end