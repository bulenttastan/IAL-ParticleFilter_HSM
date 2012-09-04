global Xg; global Zg; global Xo; global Zo; global numobstacles;
hold on;

distg = 9;
disto = 3;
numobstacles = 1;

Xg = sin(0/180*pi)*distg;
Zg = cos(0/180*pi)*distg;
plot(Xg,Zg,'x');

for i=1:3
Xo = sin(-4/180*pi)*(disto+i-1);
Zo = cos(-4/180*pi)*(disto+i-1);
plot(Xo,Zo,'o');


% ode45('fajen',[0 dg(y0(3),y0(4))],y0); 
% ode45('fajen',[0 20],y0);
y0 = [0 0 0 0];
[t,y] = ode45('fajen',[0 dg(y0(3),y0(4))],y0); 
x=y(:,3);
z=y(:,4);

plot(x,z);
end