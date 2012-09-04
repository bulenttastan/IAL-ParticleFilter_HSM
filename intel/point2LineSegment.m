function [distance,Px,Py,r] = point2LineSegment(Ax,Ay,Bx,By,Cx,Cy,varargin)
L = sqrt((Bx-Ax)^2 + (By-Ay)^2);
%            AC dot AB
%         r = --------- 
%             ||AB||^2
% 
%         r has the following meaning:
% 
%             r=0      P = A
%             r=1      P = B
%             r<0      P is on the backward extension of AB
%             r>1      P is on the forward extension of AB
%             0<r<1    P is interior to AB
r = ((Cx-Ax)*(Bx-Ax) + (Cy-Ay)*(By-Ay)) / L^2;

Px = Ax + r*(Bx-Ax);
Py = Ay + r*(By-Ay);

%plot the points
if nargin>6 && varargin{1}
    hold on;
    plot([Ax Bx],[Ay By]);
    plot(Cx,Cy,'r.');
    plot(Px,Py,'g.');
    text(Ax,Ay,'A');
    text(Bx,By,'B');
    waitforbuttonpress;
end

if r<0
    distance = sqrt((Cx-Ax)^2 + (Cy-Ay)^2);
    if nargin>7 && varargin{2}
        disp(['r: ' num2str(r) '  -> left of A  and dist: '...
                    num2str(distance)]);
    end
elseif r>1
    distance = sqrt((Cx-Bx)^2 + (Cy-By)^2);
    if nargin>7 && varargin{2}
        disp(['r: ' num2str(r) '  -> right of B  and dist: '...
                    num2str(distance)]);
    end
else
    distance = sqrt((Cx-Px)^2 + (Cy-Py)^2);
    if nargin>7 && varargin{2}
        disp(['r: ' num2str(r) '  -> inside of AB  and dist: '...
                    num2str(distance)]);
    end
end