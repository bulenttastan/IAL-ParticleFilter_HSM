% applies kolmogorov-smirnov test to see
% if the path is equal to the Fajen-Warren path
% by comparing both x and y seperately.
% returns: H,P and KSSTAT vectors for x and y
% ex:
close all; clear; clc;
addpath('..');
load '../data/humanpath/points2';
% dat = dlmread('../data/humanpath/tracks_new_01-21-08.15.59.54.out');
dat = dlmread('../data/humanpath/tracks_new_01-22-08.10.12.48.out');
% dat = dlmread('../data/humanpath/tracks_new_01-23-08.14.50.06.out');
id   = dat(:,2);
y    = dat(:,3);
x    = dat(:,4);
uid = unique(id);
% acceptlist = [5,9,12,42,46,52,63,64]; %tracks_new_01-21-08.15.59.54
acceptlist = [11,18,19,30,31,38,41,43,44,45,50,51,72,73,74,140,144,147,150,154,155]; %tracks_new_01-22-08.10.12.48
% acceptlist = [1,2,3,24,26,33,35,37,40,46,51,58,64,65,69,71,73,80,83,85,92,93];   %tracks_new_01-23-08.14.50.06



%initialize output var.s and options
H=zeros(1,10);
P=zeros(1,10);
KSSTAT=zeros(1,10);
isplot=1; %if nargin>4; isplot=varargin{1}; end
isdisp=1; %if nargin>5; isdisp=varargin{2}; end

errors=0; corrects=0;
for accept=1:length(acceptlist)
    htrace = acceptlist(accept);
    ux = x(id == uid(htrace));
    uy = y(id == uid(htrace));
    
    X = ux-ux(1);
    Y = uy-uy(1);
    Px = px-ux(1);
    Py = py-uy(1);
    
    %%%%%%%%%%%%%%%%%%%%%% CALCULATING TIME %%%%%%%%%%%%%%%%%%%%%%%
        %tracks_new_01-21-08.15.59.54
%     if htrace == 5
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 9
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.24;
%     elseif htrace == 12
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 42
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 46
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 52
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 63
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    %tracks_new_01-22-08.10.12.48
    if htrace == 18
        t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    elseif htrace == 72
        t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    elseif htrace == 74
        t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
    %tracks_new_01-23-08.14.50.06
%     if htrace == 1
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3.1;
%     elseif htrace == 2
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3.2;
%     elseif htrace == 33
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 35
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.9;
%     elseif htrace == 37
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3;
%     elseif htrace == 46
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 51
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.6;
%     elseif htrace == 73
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 80
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 89
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
    elseif sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2) > 12
        t=sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3;
    elseif sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2) > 8
        t=sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    else
        t=sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.6;
    end
    %%%%%%%%%%%%%%%%%%%% CALCULATING TIME ENDS %%%%%%%%%%%%%%%%%%%%

    
    br = find(X~=X(1),12,'first'); br=br(end);   % correction pos
    % creating rot mat for once
    SIN = (X(br)-X(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
    COS = (Y(br)-Y(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
    rotmat = [COS -SIN;
              SIN COS];
    a = rotmat * [X';Y'];
    b = rotmat * [Px';Py'];
    X = a(1,:);
    Y = a(2,:);
    Px = b(1,:);
    Py = b(2,:);
    
    
    

    % HSM path
    [b,kg,c1,c2,ko,c3,c4,V] = params();
    disp(['(' num2str(htrace) ') Length: ' num2str(sum(sqrt(diff(ux).^2 + diff(uy).^2)))...
        '  -  HSM Length: ' num2str(sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2))]);

    [t,yy]=observer(X(end),Y(end),Px,Py,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,t);
    hx = yy(:,3);
    hy = yy(:,4);
    
    
%     rotmat = [COS SIN;
%               -SIN COS];
%     a = rotmat * [X;Y];
%     b = rotmat * [hx';hy'];
%     c = rotmat * [Px;Py];
%     X = a(1,:);
%     Y = a(2,:);
%     hx = b(1,:);
%     hy = b(2,:);
%     Px = c(1,:);
%     Py = c(2,:);
    
    hold on; axis equal;
    plot(X,Y,'LineWidth',2);
    plot(hx,hy,'g','LineWidth',2);
    plot(Px,Py,'.r');


    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% APPLY KS TEST %%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    

%     %if the paths don't end at the same time
%     %cut the longer path
%     ind=find(T>t(end),1,'first');
%     if ~isempty(ind)
%         X(ind)=interp1(T,X, t(end));    X=X(1:ind);
%         Y(ind)=interp1(T,Y, t(end));    Y=Y(1:ind);
%         T(ind)=t(end);
%         T=T(1:ind);
%         if isdisp; disp(['cut secondlife path at index: ' num2str(ind)]); end
%     end
% 
%     %check if Fajen path is longer
%     ind=find(t>T(end),1,'first');
%     if ~isempty(ind)
%         x(ind)=interp1(t,x, T(end));    x=x(1:ind);
%         y(ind)=interp1(t,y, T(end));    y=y(1:ind);
%         t(ind)=T(end);
%         t=t(1:ind);
%         if isdisp; disp(['cut fajen path at index: ' num2str(ind)]); end
%     end

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
    [H(accept,1),P(accept,1),KSSTAT(accept,1)] = kstest2(X,hx);


    if isplot   %plot x comparison
        figure();
        hold on;
        F1 = cdfplot(X);
        F2 = cdfplot(hx);
        set(F1,'LineWidth',2,'Color','r');
        set(F2,'LineWidth',2);
        legend([F1 F2],'F_1(Human_x)','F_2(HSM_x)','Location','NW');
    end
    if isdisp   %display x comparison
        if H(accept,1)==0; disp('X == hx'); corrects=corrects+1;
        else disp('X != hx'); errors=errors+1; end
    end


    %applies ks two sample test on y axis
    [H(accept,2),P(accept,2),KSSTAT(accept,2)] = kstest2(Y,hy);


    if isplot   %plot y comparison
        figure();
        hold on;
        F1 = cdfplot(Y);
        F2 = cdfplot(hy);
        set(F1,'LineWidth',2,'Color','r');
        set(F2,'LineWidth',2);
        legend([F1 F2],'F_1(Human_y)','F_2(HSM_y)','Location','NW');
        xlabel('y');
        ylabel('F(y)');
    end
    if isdisp   %display y comparison
        if H(accept,2)==0; disp('Y == hy'); corrects=corrects+1;
        else disp('Y != hy'); errors=errors+1; end
    end

end

disp(['Accuracy %: ' num2str(corrects/(corrects+errors)*100)]);