% SecondLife
addpath('mysql');
addpath('data');
if mysql('status');
    mysql('open', 'localhost', 'root', 'asalak');
    mysql('use tracker');
end
s = zeros(1,2);
f=zeros(1,2);
o=zeros(6,2);
p = zeros(40,2);
[s(1),s(2)] = mysql('select x,y from start where trial="1"');
[f(1),f(2)] = mysql('select x,y from finish where trial="1"');
[o(:,1),o(:,2)] = mysql('select x,y from obstacle where trial<"7"');
[p(:,1),p(:,2)] = mysql('select x,y from pathlog where trial="2"');
s = p(1,:);
f = p(40,:);

figure(1);
hold on;
plot(s(1),s(2),'r.');
% plot(f(1),f(2),'x');
% plot(o(:,1),o(:,2),'o');
plot(p(:,1),p(:,2),'r:');

% Fajen et al.
opposite = p(40,1)-p(1,1);
adjacent = p(40,2)-p(1,2);
angle = atan(opposite/adjacent);
if(adjacent<0) 
    angle = angle - pi;
end
y0 = [angle 0 p(1,1) p(1,2)];
b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6;%ko=50; c3=3; c4=.25;%b=4.05; kg=10; c1=0.3; c2=1; ko=158; c3=5.5; c4=0.4;
[t,y]=observer(p(40,1),p(40,2),o(:,1),o(:,2),3.1119437730616886,y0,b,kg,c1,c2,ko,c3,c4);

% opensteer
load path;
plot(path(:,1), path(:,2),'k--');