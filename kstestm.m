function [H,P,KSSTAT] = kstestm(X,Y,T,x,y,t,varargin)
% function [H,P,KSSTAT] = kstestm(X,Y,T,OX,OY,varargin)
% applies kolmogorov-smirnov test to see
% if the path is equal to the Fajen-Warren path
% by comparing both x and y seperately.
% returns: H,P and KSSTAT vectors for x and y
% ex:
% close all; clear; clc; hold on;
% load 'data/scenario1';
% X=X(1,:)';Y=Y(1,:)';T=T(1,:)';
% OX=OX(1,:)';OY=OY(1,:)';
% % [H,P,KSSTAT]=kstestm(X,Y,T,OX,OY,1,1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initialize output var.s and options
H=zeros(1,2);
P=zeros(1,2);
KSSTAT=zeros(1,2);
alpha=.5; if nargin>6; alpha=varargin{1}; end
isplot=0; if nargin>7; isplot=varargin{2}; end
isdisp=0; if nargin>8; isdisp=varargin{3}; end

%calculates Fajen path
% [b,kg,c1,c2,ko,c3,c4,V]=params();
% [t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
% x=y(:,3);
% y=y(:,4);


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

% if isplot   %plot c.d.fs
%     hold on;
%     plot(T,cumsum(X/sum(X)),'r');
%     plot(t,cumsum(x/sum(x)));
%     plot(T,cumsum(Y/sum(Y)),'g');
%     plot(t,cumsum(y/sum(y)),'y');
%     title('Cumulative Diagram');
%     xlabel('time (in sec)');
%     legend('SL_X','Fajen_x','SL_Y','Fajen_y','Location','NW')
% end


%applies ks two sample test on x axis
[H(1),P(1),KSSTAT(1)] = kstest2(X,x,alpha);


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
    if H(1)==0; disp('X == x');
    else disp('X != x'); end
end


%applies ks two sample test on y axis
[H(2),P(2),KSSTAT(2)] = kstest2(Y,y,alpha);


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
    if H(2)==0; disp('Y == y');
    else disp('Y != y'); end
end