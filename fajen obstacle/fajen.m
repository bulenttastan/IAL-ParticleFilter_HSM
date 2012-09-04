function ydot = fajen( t,y )

b=3.25; kg=7.5; ko=198; c1=.4; c2=.4; c3=6.5; c4=.8; V=1;
global numobstacles;

obstacles = 0;
ydot = zeros(size(y));
ydot(1) = y(2);
for i=1:numobstacles
    pp = y(1)-psio(i,y(3),y(4));
    obstacles = obstacles + ko*(pp)*exp(-c3*abs(pp))*exp(-c4*do(i,y(3),y(4)));
end
ydot(2) = -b*y(2) -kg*(y(1)-psig(y(3),y(4)))*(exp(-c1*dg(y(3),y(4)))+c2) + obstacles;
ydot(3) = V*sin(y(1));
ydot(4) = V*cos(y(1));