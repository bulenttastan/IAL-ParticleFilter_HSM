global Xg; global Zg; global numobstacles;
hold on;

dist = 2;
numobstacles = 0;


for i=1:3
Xg = sin(20/180*pi)*dist;
Zg = cos(20/180*pi)*dist;
plot(Xg,Zg,'x');

% ode function
y0 = [0 0 0 0];
to = 0;
tf = dg(y0(3),y0(4));
tspan = linspace(to,tf,1);
[t,y] = ode45('fajen',tspan,y0);
x=y(:,3);
z=y(:,4);

plot(x,z);

dist = dist*2;
end