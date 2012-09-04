function [t,y] = observer(xg,zg,xo,zo,varargin)
global Xg; global Zg; global Xo; global Zo; global numobstacles; global V;
global b; global kg; global ko; global c1; global c2; global c3; global c4;
b=3.25; kg=7.5; ko=198; c1=.4; c2=.4; c3=6.5; c4=.8;

numobstacles = length(xo);
Xg = xg;
Zg = zg;
Xo = xo;
Zo = zo;
V = 1;
% hold on;
% plot(Xg,Zg,'x');
% plot(Xo,Zo,'o');

if nargin>4
    V = varargin{1};
end
if nargin>5
    y0 = varargin{2};
else
    y0 = [0 0 0 0];
end
if nargin>6
    b=varargin{3}; kg=varargin{4}; c1=varargin{5}; c2=varargin{6};
end
if nargin>10
    ko=varargin{7}; c3=varargin{8}; c4=varargin{9};
end

to = 0;
tf = sqrt((Xg-y0(3))^2+(Zg-y0(4))^2)/V;
tspan = linspace(to,tf,2);
if nargin>13
    tspan=varargin{10};
end

[t,y] = ode45('fajen',tspan,y0);
% x=y(:,3);
% z=y(:,4);
% 
% plot(x,z);
%observer(p(40,1),p(40,2),o(:,1),o(:,2),3.1119437730616886,y0);
%observer(3,4,[],[],1,[0 0 0 0],3.25,7.5,.4,.4);