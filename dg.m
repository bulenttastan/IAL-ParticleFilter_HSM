function dist = dg( x,z )
% The distance between observer and goal

global Xg;
global Zg;

dist = sqrt((Xg-x)^2 + (Zg-z)^2);