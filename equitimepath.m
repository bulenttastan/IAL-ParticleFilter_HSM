function [x,y,t] = equitimepath( steps,x,y,t )
% EQUITIMEPATH divides given path (x,y) into equitime distances
% steps: how many partitions
% x,y: path data
% t: time on each point of the path

interval=t(numel(t))/steps;
P=zeros(steps,2);
tt=zeros(steps,1);
ind=1;

for i=1:steps
    %calculate equitime
    T=i*interval;
    if i==steps; T=t(end); end;
    
    %interpolate on time frame to position
    ind=findfirst(t,T,ind);
    P(i,1) = interp1([t(ind-1) t(ind)],[x(ind-1) x(ind)],T);
    P(i,2) = interp1([t(ind-1) t(ind)],[y(ind-1) y(ind)],T);
    tt(i)=T;
end

x=P(:,1);
y=P(:,2);
t=tt;