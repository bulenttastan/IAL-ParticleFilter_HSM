function angle = psio( i,x,z )
% The angle between obstacle i and observer according to reference axis

global Zo;
global Xo;

angle = acos((Zo(i)-z)/do(i,x,z));
if Xo(i) < x
    angle = -angle;
end