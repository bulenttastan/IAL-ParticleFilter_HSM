function [error,catchtime,newx,newy] = occlusion( start,increment,stop,X,Y,T,OX,OY,varargin )

catchtime=Inf;
isplot=0;    if nargin>8 && varargin{1}; isplot=varargin{1}; end;
isdisplay=0; if nargin>9 && varargin{2}; isdisplay=varargin{2}; end


[b,kg,c1,c2,ko,c3,c4,V] = params();
[t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
x=y(:,3);   y=y(:,4);

if isplot
    plot(X,Y,'r');
    plot(OX,OY,'or');
    plot(x,y);
end


%find starting lost position
indstart=findfirst(T,start,1);
move = [X(indstart)-X(indstart-1),Y(indstart)-Y(indstart-1)];

%new is lost direction
newx = zeros(1,numel(X));
newx(1:indstart) = X(1:indstart);
newx(indstart+1:end)=X(indstart) + (0:(numel(X)-indstart-1)).*move(1);
newy = zeros(1,numel(Y));
newy(1:indstart) = Y(1:indstart);
newy(indstart+1:end)=Y(indstart) + (0:(numel(Y)-indstart-1)).*move(2);

if isplot
    plot(newx,newy,'g');
    plot(X(indstart),Y(indstart),'.r');
end

% ssss=sumsquare(X,Y,x(1:indstart),y(1:indstart),T(1:indstart),0,100*start/stop);
% disp(ssss);
%loop through each time step
cought=0;
recordindex=0;
error=zeros(floor((stop-start)/increment+1),3);
for i=start:increment:stop
    recordindex=recordindex+1;
    %find current time position, calculate error
    ind1=findfirst(T,i);
    ss1=sumsquare(X,Y,newx(1:ind1),newy(1:ind1),T(1:ind1),0,100*i/stop);
    
    %find position for fajen, calculate error
    ind2=findfirst(t,i);
    ss2=sumsquare(X,Y,x(1:ind2),y(1:ind2),t(1:ind2),0,100*i/stop);
    
    error(recordindex,1)=i-start;
    error(recordindex,2)=ss1;
    error(recordindex,3)=ss2;
    
%     if isplot
%         plot(newx(ind1),newy(ind1),'.g');
%         plot(x(1:ind2),y(1:ind2),'.b');
%     end
    if isdisplay
        disp(['@ time: ' num2str(i)]);
        disp(['lost: ' num2str(ss1) ' - fajen: ' num2str(ss2)]);
    end
    
    %when lost error passes fajen, then fajen succeeds
    if ss2<ss1 && ~cought
        catchtime=i-start;
        if isplot
            plot(newx(ind1),newy(ind1),'.k');
            plot(x(ind2),y(ind2),'.b');
            waitforbuttonpress;
            sumsquare(X,Y,newx(1:ind1),newy(1:ind1),T(1:ind1),1,100*i/stop);
        end
        if isdisplay
            disp(['fajen catches after ' num2str(i-start) ' seconds']);
        end
        cought=1;
    end
    if isplot
        waitforbuttonpress;
    end
end