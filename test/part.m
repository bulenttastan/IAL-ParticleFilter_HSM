% function part()
clear; close all; clc;
%problem 1
x=randn(100,1);
m=zeros(100,1);
m(1,1)=x(1,1);

for i=2:100
    m(i,1)=1/i*x(i,1)+(i-1)/i*m(i-1,1);
end

plot(m);
set(line([1,100],[mean(x),mean(x)]),'color','m');

%problem 2
F=sin(2*pi/30 * 1:100);
t=1:1000;
a=5*sin(2*pi*t/500);

x=zeros(2,length(t));
x0=[0;-8];
T=.1; m=5;

A=[1 T;0 1];
B=[T*T/(2*m);T/m];

for i=1:length(t)
    if i==1; x(:,i) = A*x0 + B*a(1,i) + .1*randn(2,1);
    else x(:,i) = A*x(:,i-1) + B*a(1,i) + .1*randn(2,1);
    end
end

plot(x(1,:),x(2,:));

%problem 3
