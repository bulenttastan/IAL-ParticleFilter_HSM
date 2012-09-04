clear; clc; close all;
isgoal=0;
global OX;
global OY;
if isgoal
    load 'data/traingoalangle';
    XX=X; YY=Y; TT=T;
    load 'data/traingoaldist';
    X=[X;XX]; Y=[Y;YY]; T=[T;TT];
else
    load 'data/trainobst';
end
global ym;
global tm;
global v;

global b; global kg; global c1; global c2; global ko; global c3; global c4;
if isgoal;
    b=3.45; kg=10; c1=1; c2=1; ko=0; c3=0; c4=0;
else
    b=3.8058; kg=9.9112; c1=0.24473; c2=0.40452; ko=300; c3=4.5; c4=0.6;
end
v=zeros(size(X,1),1);

ym=struct([]);
tm=struct([]);


for trace=1:size(X,1)
    % extract unique points
    [uniq,m,n]=uunique([X(trace,:)' Y(trace,:)']);
%     i=find(uniq(trace,:)~=X(trace,1),1,'first');
    x=uniq(:,1); z=uniq(:,2); t=T(trace,m);
    % discard after the point where there is no big change on both x, z
    j=find(x<x(end)-.0001,1,'last')+1;
    jz=find(z<z(end)-.0001,1,'last')+1;
    if isempty(j) || jz>j; j=jz; end
    x=x(1:j); z=z(1:j); t=t(1:j);
    %create structures
    ym(trace).y=zeros(size(x,1),4);
    tm(trace).t=t;
    
    
    % calculate ode variables
    prevang=0;
    for i=2:size(x)
        ang = acos((z(i)-z(i-1))/sqrt((x(i)-x(i-1))^2+(z(i)-z(i-1))^2));
        angdot = ang-prevang;
        angdot=angdot/(tm(trace).t(i)-tm(trace).t(i-1));

        ym(trace).y(i,1)=ang;
        ym(trace).y(i,2)=angdot;
        ym(trace).y(i,3)=x(i);
        ym(trace).y(i,4)=z(i);
        v(trace)=v(trace)+sqrt((x(i)-x(i-1))^2+(z(i)-z(i-1))^2);

        prevang = ang;
        disp(['----- ' num2str(i) ' -------']);
        disp(['Ang:  ' num2str(ym(trace).y(i,1)/pi*180)]);
        disp(['Ang`: ' num2str(ym(trace).y(i,2)/pi*180)]);
        disp(['x:    ' num2str(ym(trace).y(i,3))]);
        disp(['z:    ' num2str(ym(trace).y(i,4))]);
        disp(['time:  ' num2str(tm(trace).t(i))]);
    end
    % calculate velocity for each trace
    v(trace)=v(trace)/tm(trace).t(end);
    disp(['Velocity: ' num2str(v(trace))]);
    
    figure(); hold on;
    plot(x,z,'r','LineWidth',2);
    plot(x(end),z(end),'rx','LineWidth',2);
    if ~isgoal; plot(OX(trace),OY(trace),'or','LineWidth',2); end
    grid;
    % plot initial fajen
    if isgoal
        [t,y] = observer(x(end),z(end),[],[],v(trace),ym(trace).y(1,:),b,kg,c1,c2,ko,c3,c4,tm(trace).t);
    else
        [t,y] = observer(x(end),z(end),OX(trace,:),OY(trace,:),v(trace),ym(trace).y(1,:),b,kg,c1,c2,ko,c3,c4,tm(trace).t);
    end
    plot(y(:,3),y(:,4),'LineWidth',2);
end





% parameter estimation
options=optimset('MaxFunEvals',10000,'MaxIter',10000);
tic;
if isgoal
    k=[b kg c1 c2];
    %[k SSEmin]=fminsearch('SSEfajen_goal',k);
    [k SSEmin]=anneal(@SSEfajen_goal,k);
    disp(['b=' num2str(k(1)) '; kg=' num2str(k(2)) '; c1=' num2str(k(3)) '; c2=' num2str(k(4)) ';']);
else
    k=[ko c3 c4];
    %[k SSEmin]=fminsearch('SSEfajen_obst',k);
    [k SSEmin]=anneal(@SSEfajen_obst,k);
    disp(['ko=' num2str(k(1)) '; c3=' num2str(k(2)) '; c4=' num2str(k(3)) ';']);
end
disp(['Error: ' num2str(SSEmin)]);
toc;


% plot improved fajen
for i=1:length(ym)
    figure();
    if isgoal
        [t,y] = observer(ym(i).y(end,3),ym(i).y(end,4),[],[],v(i),ym(i).y(1,:),k(1),k(2),k(3),k(4),ko,c3,c4,tm(i).t);
    else
        [t,y] = observer(ym(i).y(end,3),ym(i).y(end,4),OX(i,:),OY(i,:),v(i),ym(i).y(1,:),b,kg,c1,c2,k(1),k(2),k(3),tm(i).t);
        plot(OX(trace),OY(trace),'or','LineWidth',2);
    end
    plot(ym(i).y(:,3),ym(i).y(:,4),'r',y(:,3),y(:,4),'k--','LineWidth',2);
    grid;
    xlabel('t');
    ylabel('y');
end



% find your way
B=0;KG=0;KO=0;C3=0;
if isgoal
    kov=linspace(b*.2,b*1.2,11);
    c3v=linspace(kg*.2,kg*1.2,11);
    [B KG]=meshgrid(kov,c3v);
else
    kov=linspace(ko*.2,ko*1.2,11);
    c3v=linspace(c3*.2,c3*1.2,11);
    [KO C3]=meshgrid(kov,c3v);
end
% bv=linspace(b*.2,b*1.2,11);
% kgv=linspace(kg*.2,kg*1.2,11);
% bv=linspace(0,500,51);
% kgv=linspace(0,500,51);
min=1000000;
minval=[0 0];
tic;
for i=1:max(length(B),length(KO))
    disp(i);
    for j=1:max(length(KG),length(C3))
        if isgoal
            SSEMat(i,j)=SSEfajen_goal([B(i,j) KG(i,j) c1 c2]);
            if SSEMat(i,j) < min
                min = SSEMat(i,j);
                minval = [B(i,j) KG(i,j)];
            end
        else
            SSEMat(i,j)=SSEfajen_obst([KO(i,j) C3(i,j) c4]);
            if SSEMat(i,j) < min
                min = SSEMat(i,j);
                minval = [KO(i,j) C3(i,j)];
            end
        end
    end
end
toc;
figure();
if isgoal
    surfc(B,KG,SSEMat);
else
    surfc(KO,C3,SSEMat);
end
colorbar;

% 
% % estimated confidence intervals
% bd=0.01*b;
% kgd=0.01*kg;
% c1d=0.01*c1;
% c2d=0.01*c2;
% 
% [t yap]=observer(Xg,Zg,[],[],V,ym(1,:),b+bd,kg,c1,c2,ko,c3,c4,tm);
% [t yam]=observer(Xg,Zg,[],[],V,ym(1,:),b-bd,kg,c1,c2,ko,c3,c4,tm);
% [t ykgp]=observer(Xg,Zg,[],[],V,ym(1,:),b,kg+kgd,c1,c2,ko,c3,c4,tm);
% [t ykgm]=observer(Xg,Zg,[],[],V,ym(1,:),b,kg-kgd,c1,c2,ko,c3,c4,tm);
% [t yc1p]=observer(Xg,Zg,[],[],V,ym(1,:),b,kg,c1+c1d,c2,ko,c3,c4,tm);
% [t yc1m]=observer(Xg,Zg,[],[],V,ym(1,:),b,kg,c1-c1d,c2,ko,c3,c4,tm);
% [t yc2p]=observer(Xg,Zg,[],[],V,ym(1,:),b,kg,c1,c2+c2d,ko,c3,c4,tm);
% [t yc2m]=observer(Xg,Zg,[],[],V,ym(1,:),b,kg,c1,c2-c2d,ko,c3,c4,tm);
% 
% X=[(yap-yam)/2/bd (ykgp-ykgm)/2/kgd (yc1p-yc1m)/2/c1d (yc2p-yc2m)/2/c2d];
% XtXinv=inv(X'*X);
% 
% n=length(tm);
% p=3;
% SE=sqrt(SSEmin/(n-p));
% Sb=SE*sqrt(XtXinv(1,1));
% Skg=SE*sqrt(XtXinv(2,2));
% Sc1=SE*sqrt(XtXinv(3,3));
% Sc2=SE*sqrt(XtXinv(4,4));
% 
% alpha=0.05;
% tval=tinv(1-alpha/2,n-p);
% bm=b-tval*Sb;
% bp=b+tval*Sb;
% kgm=kg-tval*Skg;
% kgp=kg+tval*Skg;
% c1m=c1-tval*Sc1;
% c1p=c1+tval*Sc1;
% c2m=c2-tval*Sc2;
% c2p=c2+tval*Sc2;
% disp([num2str(bm) ' <= ' num2str(b) ' <= ' num2str(bp)]);
% disp([num2str(kgm) ' <= ' num2str(kg) ' <= ' num2str(kgp)]);
% disp([num2str(c1m) ' <= ' num2str(c1) ' <= ' num2str(c1p)]);
% disp([num2str(c2m) ' <= ' num2str(c2) ' <= ' num2str(c2p)]);


% for a given y1, y2 and y3:
% 
% dy1/dt = a*y1 + b*y2 - c*y3
% dy2/dt = b*y1 - (a-1)*y2 - c*y1*y3
% dy3/dt = a*y1 - b
% 
% How to find a, b and c?
%-----------------
% Assume for a moment that you know the analytical
% solution for your system of ordinary differential
% equations, say y_i(t,a,b,c) (i=1,2,3).
% 
% Then the problem to determine optimal parameters
% can be formulated as
% 
% min_{a,b,c}:
% sum_{j=1}^{n} sum_{i=1}^{3}
% (y_i(t_j,a,b,c)-y_measured_i(t_j))^2
% 
% where the t_j are times where measurements for the
% y_i are available.
% 
% This would define a normal nonlinear
% parameter fitting problem for which you could use
% lsqnonlin, lsqcurvefit.
% 
% Now in your case an analytical solution for the
% above system of differential equations
% seems hard to determine.
% 
% For this reason, you have to couple ODE45 and one of the
% parameter estimation routines:
% 
% In each step k, you get suggestions for parameters
% a_k, b_k, c_k from lsqnonlin or lsqcurvefit.
% With these parameters, you call ODE45 to
% determine the y_i(t_j,a_k,b_k,c_k) and transfer
% these values to lsqnonlin or lsqcurvefit.
% In the next step, lsqnonlin or lsqcurvefit will
% transfer new a_(k+1), b_(k+1), c_(k+1).
% You call ODE45 and so on.
% 
% I don't know how this can be implemented in MATLAB
% in practise, but that's the general idea how you
% will have to proceed.


%----------------------------
%--  OLD CODE REPOSITORY  ---
%----------------------------
% angdot=zeros(1,21);
% angddot=zeros(1,21);
% xdot=zeros(1,21);
% zdot=zeros(1,21);

%     xdot(i)=x(i)-x(i-1);
%     xdot(i)=xdot(i)/(tm(i)-tm(i-1));
%     zdot(i)=z(i)-z(i-1);
%     zdot(i)=zdot(i)/(tm(i)-tm(i-1));
%     angddot(i)=angdot(i)-angdot(i-1);
%     angddot(i)=angddot(i)/(tm(i)-tm(i-1));