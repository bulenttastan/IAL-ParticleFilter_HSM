function angle = psig( x,z )
% The angle between goal and the observer according to reference axis

global Zg;
global Xg;

angle = acos((Zg-z)/dg(x,z));   %/pi*180; to see the angle
if Xg < x
    angle = angle*-1;
end