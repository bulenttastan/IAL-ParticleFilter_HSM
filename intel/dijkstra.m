function [path, cost] = dijkstra(transition, from, to)
% transition: transition cost matrix, for unconnected put inf
%             0 means same node
% from: start node index [1, V]
% to: final node index [1, V]
% path: shortest path in column vector, if there's no path empty list returned
% cost: cost of shortest path, if there's no path inf returned
% ie:
% clear; clc;
% cost=[0   3  inf inf  2  inf inf;
%       3   0   1  inf inf inf inf;
%      inf  1   0   2  inf  4  inf;
%      inf inf  2   0  inf  2  inf;
%       2  inf inf inf  0   3  inf;
%      inf inf  4   2   3   0   2;
%      inf inf inf inf inf  2   0];
%  
% [path,cost]=dijkstra(cost,1,4)

% Main loop.
count = min(size(transition));
parent = zeros(count, 1);
loss = inf(count, 1);
    
% Initialize.
m = from;
if issparse(transition)
    index = transpose(full(transition(m, :) > 0.0));
    index(m) = true;
else
    index = transpose(not(isinf(transition(m, :))));
end
loss(:) = inf;
loss(index) = transpose(transition(m, index));
parent(:) = 0;
parent(index) = m;
queue = find(index);
    
% Explore graph.
while not(isempty(queue))
    k = queue(1);
    distance = transpose(loss(k) + transition(k, :));
    index = distance < loss;
    if issparse(transition)
        index = and(index, full(transpose(transition(k, :) > 0)));
    end
    if any(index)
        loss(index) = distance(index);
        parent(index) = k;
        queue = cat(1, queue(2:end, :), find(index));
    else
        queue = queue(2:end, :);
    end
end
    
% Back track.
path = zeros(size(parent));
n = to;
for k = numel(parent):(-1):1
    if or(eq(m, n), eq(n, 0))
        break
    end
    path(k) = n;
    n = parent(n);
end
if eq(m, n)
    path(k) = n;
    path = path(k:end);
else
    path = [];
end
cost = loss(to);

return
