function showresults
clear; close all; clc;
addpath('ttest');
load data/opensteerdata;
showdata=1;
showall=1;
ss=zeros(9,3);
area=zeros(9,3);

% show goal results
if showdata==1 || showall
    load data/goalonly;
    for i=1:size(X,1)
        if i==1; opensteerpath=pathgoal1; end
        if i==2; opensteerpath=pathgoal2; end
        if i==3; opensteerpath=pathgoal3; end
        [ss(i,:) area(i,:)]=...
                plotpath(X(i,:),Y(i,:),[],[],opensteerpath); %#ok<COLND>
        
    end
end

% show obstacle results
if showdata==2 || showall
    load data/goal1obst;
    for i=1:size(X,1)
        if i==1; opensteerpath=path1obst1; end
        if i==2; opensteerpath=path1obst2; end
        if i==3; opensteerpath=path1obst3; end
        [ss(i+3,:) area(i+3,:)]=...
                plotpath(X(i,:),Y(i,:),OX(i),OY(i),opensteerpath); %#ok<COLND>
    end
end

% show multiple(1) results
if showdata==3 || showall
    load data/goalmultobst;
    for i=1:size(X,1)
        if i==1; opensteerpath=pathmultobst1; end
        if i==2; opensteerpath=pathmultobst2; end
        if i==3; opensteerpath=pathmultobst3; end
        [ss(i+6,:) area(i+6,:)]=...
                plotpath(X(i,:),Y(i,:),OX(i,:),OY(i,:),opensteerpath); %#ok<COLND>
    end
end


avr=zeros(2,3);
for i=1:3
    avr(:,i)=[mean(ss(:,i));mean(area(:,i))];
end
tableresult(avr(1,:),avr(2,:),'average');



% plot the paths
function [ss,area]=plotpath(X,Y,OX,OY,opensteerpath)
b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6; V=3.1119437730616886;
ss = [0 0 0];
area = [0 0 0];
islinesuntuned=0;
islinestuned=0;
islinesopensteer=0;
isdispresults=0;
isshowplot=1;
isshowtable=0;
isdispcoor4opensteer=0;

%original path
if isshowplot
    figure();hold on;
    plot(X(:),Y(:),'r');
    plot(X(1),Y(1),'.r');
    plot(X(40),Y(40),'xr');
    plot(OX,OY,'or');
end
if isdispcoor4opensteer
    disp('0 0 .6 3.1119437730616886');
    disp([num2str(X(40)) ' ' num2str(Y(40)) ' .1']);    %goal
    disp(num2str(numel(OX)));
    for i=1:numel(OX);  %obstacles
        disp([num2str(OX(i)) ' ' num2str(OY(i)) ' .4']);
    end
    disp('--------------------');
end

%fajen default param (untuned)
[t,y]=observer(X(40),Y(40),OX,OY,V);
if isshowplot; plot(y(:,3),y(:,4),'g'); end
%area calculation
area(1)=polyarea([X fliplr(y(:,3)')],[Y fliplr(y(:,4)')]);
%sum square calculation
ss(1)=sumsquare(X,Y,y(:,3),y(:,4),t,(islinesuntuned && isshowplot));


%path using DTW (tuned)
[t,y]=observer(X(40),Y(40),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
if isshowplot; plot(y(:,3),y(:,4)); end
%area calc
area(2)=polyarea([X fliplr(y(:,3)')],[Y fliplr(y(:,4)')]);
%sum square calc
ss(2)=sumsquare(X,Y,y(:,3),y(:,4),t,(islinestuned && isshowplot));


%opensteer
x=opensteerpath(:,1)';
y=opensteerpath(:,2)';
t=opensteerpath(:,3);
if isshowplot; plot(x,y,'k'); end
%area calculation
area(3)=polyarea([X fliplr(x)],[Y fliplr(y)]);
%sum square calculation
ss(3)=sumsquare(X,Y,x,y,t,(islinesopensteer && isshowplot));

if isdispresults
    disp(['SSU - SST - SSOS: ' num2str(ss(1)) ' - ' num2str(ss(2)) ' - ' num2str(ss(3))]);
    disp([' AU -  AT -  AOS: ' num2str(area(1)) ' - ' num2str(area(2)) ' - ' num2str(area(3))]);
end
% %fajen area param
% [t,y]=observer(X(40),Y(40),OX,OY,V,[0 0 0 0],4.05,10,0.3,1,158,5.5,0.4);
% plot(y(:,3),y(:,4),'k');

if isshowtable; tableresult(ss,area); end
if isshowplot; waitforbuttonpress; end

