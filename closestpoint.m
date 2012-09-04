function [ Q1,Q2,d ] = closestpoint( curve,P )
% CLOSESTPOINT 
% returns the closest point to curve and distance

if size(curve,2)==2
    curve=curve';
end

%find the closest two points to the curve
[mins,pts]=twomin((curve(1,:)-P(1)).^2+(curve(2,:)-P(2)).^2);
Q1=[curve(1,pts(1)) curve(2,pts(1))];
Q2=[curve(1,pts(2)) curve(2,pts(2))];

posQ1 = find(curve(1,:)==Q1(1),1,'first'); %intersect(find(curve(1,:)==Q1(1)),find(curve(2,:)==Q1(2)));
posQ2 = find(curve(1,:)==Q2(1),1,'first'); %intersect(find(curve(1,:)==Q2(1)),find(curve(2,:)==Q2(2)));
if posQ2 < posQ1
    temp = Q1;
    Q1 = Q2;
    Q2 = temp;
end
    
%calculate shortest distance
d = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1);