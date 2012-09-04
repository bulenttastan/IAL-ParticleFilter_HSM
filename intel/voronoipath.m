close all; clear; clc;
Q1 = [3;3]; Q2 = [4;6]; P = [1.00001;1.99998];

%distance between line Q1Q2 and point P
d = abs(det([Q2-Q1,P-Q1]))/norm(Q2-Q1); % for col. vectors
% d = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1); % for row vectors.

%the nearest point R, on line Q1Q2
R = (dot(P-Q2,Q1-Q2)*Q1+dot(P-Q1,Q2-Q1)*Q2)/dot(Q2-Q1,Q2-Q1);
d2 = norm(R-P);

axis equal;
hold on;

plot([Q1(1) Q2(1)],[Q1(2) Q2(2)]);
plot(R(1),R(2),'g.');
plot(P(1),P(2),'r.');
plot([P(1) R(1)],[P(2) R(2)]);

disp(['dist  : ' num2str(d)]);
disp(['dist 2: ' num2str(d2)]);