function calcstats( x1,y1,x2,y2,t )
%calculates paired t-test for paths using equitime points

steps=numel(x1);
interval=t(numel(t))/steps;
ind=1;
xx2=zeros(1,steps);
yy2=zeros(1,steps);

for i=1:steps
    %calculate equitime
    T=i*interval;
    if i==steps; T=t(end); end;
    
    %interpolate on time frame to position
    ind=findfirst(t,T,ind);
    xx2(i) = interp1([t(ind-1) t(ind)],[x2(ind-1) x2(ind)],T);
    yy2(i) = interp1([t(ind-1) t(ind)],[y2(ind-1) y2(ind)],T);
    plot(xx2(i),yy2(i),'.g');
end

disp(['test: ' num2str(mean(x1)) ' ' num2str(mean(x2))]);
ttest(x1,xx2,1);
% ttest(y1,yy2,1);
% calcstats(X,Y,y(:,3),y(:,4),t);


% disp(['test: ' num2str(mean(ss(:,2))) ' ' num2str(mean(ss(:,1)))]);
% ttest(zeros(1,9),ss(:,1),1);