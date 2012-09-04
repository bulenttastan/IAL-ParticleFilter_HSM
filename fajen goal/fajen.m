function ydot = fajen( t,y )

b = 3.25;   kg = 7.5;   c1 = 0.4;   c2 = 0.4;   V = 1;

ydot = zeros(size(y));
ydot(1) = y(2);
ydot(2) = -b*y(2) -kg*(y(1)-psig(y(3),y(4)))*(exp(-c1*dg(y(3),y(4)))+c2);
ydot(3) = V*sin(y(1));
ydot(4) = V*cos(y(1));