% starts with Ken's polyline algorithm, and compares against 
% Douglas-Peucker Polyline Simplification algorithm
% just hit F5 and see that red lines which is the latter alg works better
close all; clear; clc;
outline = dlmread('../data/humanpath/outline_intel.dat');
oy    = outline(:,2);
ox    = outline(:,3);
clear outline;
% plot(ox,oy,'.r')
title('outline of experimental area')
xlabel('x [m]')
ylabel('y [m]')
axis equal;
p = [ox oy];%[ox(1:2000) oy(1:2000)];

v = zeros(size(p,1),2);
index = 6001;
skip = false;


for i=1:size(p,1)
    if ~skip
        v(i,:) = p(index,:);
    end
    p(index,:) = [];
    mn = Inf;
    skip = false;
    for j=1:size(p,1)
        dists = (v(i,1)-p(j,1))^2 + (v(i,2)-p(j,2))^2;
        if dists < mn
            mn = dists;
            index = j;
        end
    end

    if mn > 3
        skip = true;
    end
    if mod(i,1000)==0
        disp(i);
    end
end

removelist = [];
for i=1:size(v,1)
    if v(i,:) == 0
        removelist(end+1) = i;
    end
end

v(removelist,:) = [];
v(1:20,:)
hold on;
plot(v(:,1),v(:,2))
return;
% x      = 1:0.1:8*pi;
% y      = sin(x) + randn(size(x));
% p      = [x' y'];
% convert points to polyline
tol    = .1;
ps     = dpsimplify(v,tol);

plot(p(:,1),p(:,2),'k')
hold on
plot(ps(:,1),ps(:,2),'r','LineWidth',2);
legend('original polyline','simplified')

% function DouglasPeucker(PointList[], epsilon)
%     %Find the point with the maximum distance
%     dmax = 0
%     index = 0
%     for i = 2 to (length(PointList) - 1)
%         d = OrthogonalDistance(PointList[i], Line(PointList[1], PointList[end])) 
%         if d > dmax
%             index = i
%             dmax = d
%         end
%     end
% 
%     %If max distance is greater than epsilon, recursively simplify
%     if dmax >= epsilon
%         %Recursive call
%         recResults1[] = DouglasPeucker(PointList[1...index], epsilon)
%         recResults2[] = DouglasPeucker(PointList[index...end], epsilon)
% 
%         % Build the result list
%         ResultList[] = {recResults1[1...end-1] recResults2[1...end]}
%     else
%         ResultList[] = {PointList[1], PointList[end]}
%     end
% 
%     %Return the result
%     return ResultList[]
% end
