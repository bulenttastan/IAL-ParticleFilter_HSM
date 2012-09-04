function dist = do( i,x,z )
% The distance between observer and goal

global Xo;
global Zo;

dist = sqrt((Xo(i)-x)^2 + (Zo(i)-z)^2);