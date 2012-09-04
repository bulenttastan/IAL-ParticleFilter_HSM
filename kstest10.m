% applies kolmogorov-smirnov test to see
% if the path is equal to the Fajen-Warren path
% by comparing both x and y seperately.
% returns: H,P and KSSTAT vectors for x and y
% ex:
close all; clear; clc; hold on;
load 'data/layout9';
% [H,P,KSSTAT]=kstestm(X,Y,T,OX,OY,1,1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TT=T;
XX=X;
YY=Y;
OOX=OX;
OOY=OY;
%initialize output var.s and options
H=zeros(1,10);
P=zeros(1,10);
KSSTAT=zeros(1,10);
isplot=1; %if nargin>4; isplot=varargin{1}; end
isdisp=1; %if nargin>5; isdisp=varargin{2}; end

errors=0;
for i=1:10
    T=str2num(datestr(TT(i,:),'SS.FFF')); %#ok<ST2NM>
    X=XX(i,:);
    Y=YY(i,:);
    OX=OOX(i,:);
    OY=OOY(i,:);
    %calculates Fajen path
    [b,kg,c1,c2,ko,c3,c4,V]=params();
    [t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
    x=y(:,3);
    y=y(:,4);
%     kstestm(X,Y,T,x,y,t,.5,1,1);


    %if the paths don't end at the same time
    %cut the longer path
    ind=find(T>t(end),1,'first');
    if ~isempty(ind)
        X(ind)=interp1(T,X, t(end));    X=X(1:ind);
        Y(ind)=interp1(T,Y, t(end));    Y=Y(1:ind);
        T(ind)=t(end);
        T=T(1:ind);
        if isdisp; disp(['cut secondlife path at index: ' num2str(ind)]); end
    end

    %check if Fajen path is longer
    ind=find(t>T(end),1,'first');
    if ~isempty(ind)
        x(ind)=interp1(t,x, T(end));    x=x(1:ind);
        y(ind)=interp1(t,y, T(end));    y=y(1:ind);
        t(ind)=T(end);
        t=t(1:ind);
        if isdisp; disp(['cut fajen path at index: ' num2str(ind)]); end
    end

%     if isplot   %plot c.d.fs
%         hold on;
%         plot(T,cumsum(X/sum(X)),'r');
%         plot(t,cumsum(x/sum(x)));
%         plot(T,cumsum(Y/sum(Y)),'g');
%         plot(t,cumsum(y/sum(y)),'y');
%         title('Cumulative Diagram');
%         xlabel('time (in sec)');
%         legend('SL_X','Fajen_x','SL_Y','Fajen_y','Location','NW')
%     end


    %applies ks two sample test on x axis
    [H(i,1),P(i,1),KSSTAT(i,1)] = kstest2(X,x);


    if isplot   %plot x comparison
        figure();
        hold on;
        F1 = cdfplot(X);
        F2 = cdfplot(x);
        set(F1,'LineWidth',2,'Color','r');
        set(F2,'LineWidth',2);
        legend([F1 F2],'F_1(SL_X)','F_2(Fajen_x)','Location','NW');
    end
    if isdisp   %display x comparison
        if H(i,1)==0; disp('X == x');
        else disp('X != x'); errors=errors+1; end
    end


    %applies ks two sample test on y axis
    [H(i,2),P(i,2),KSSTAT(i,2)] = kstest2(Y,y);


    if isplot   %plot y comparison
        figure();
        hold on;
        F1 = cdfplot(Y);
        F2 = cdfplot(y);
        set(F1,'LineWidth',2,'Color','r');
        set(F2,'LineWidth',2);
        legend([F1 F2],'F_1(SL_Y)','F_2(Fajen_y)','Location','NW');
    end
    if isdisp   %display y comparison
        if H(i,2)==0; disp('Y == y');
        else disp('Y != y'); errors=errors+1; end
    end

end

disp(['Accuracy %: ' num2str((1-errors/20)*100)]);