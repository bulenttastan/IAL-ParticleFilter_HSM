function ar = aream3( X,Y,x,y,varargin )
% AREAM Calculates the area between two polygons
% X,Y first polygon
% x,y second polygon
% 2 arguments: 1 to display, 1 to plot
% Example:
% load '../data/goalmultobst';
% hold on;
% line=3;
% X=X(line,:);
% Y=Y(line,:);
% % OX=[];OY=[];
% % OX=OX(line);OY=OY(line);
% OX=OX(line,:);
% OY=OY(line,:);
% b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6; V=3.1119437730616886;
% [t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4); 
% x=y(:,3)'; y=y(:,4)';
% plot(X,Y,'r');
% plot(OX,OY,'or');
% plot(x,y);
% aream(X,Y,x,y,1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; %close all; clc;
load aream;x=path(:,1)'; y=path(:,2)'; isdisp=1; isplot=1;
figure('Position',[400,100,800,800]); hold on;
axis([-9 9 0 18],'square');
plot(X,Y,'r');
plot(OX,OY,'ro');
plot(x,y);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
isdisp=0; if nargin>4; isdisp=varargin{1}; end;
isplot=1; if nargin>5; isplot=varargin{2}; end;
if size(X,1)>size(X,2); X=X'; Y=Y'; end
if size(x,1)>size(x,2); x=x'; y=y'; end

%connect end points
x=[x X(end)]; y=[y Y(end)];


%calculate intersection points
[a,b]=curveintersect(X,Y,x,y);


%%%%%%%%%%%%%%%%% DATA ACQUSITION %%%%%%%%%%%%%%%%%%%%%%%%%
ar = 0; f=0; s=0; ff=[]; ss=[]; %#ok<NASGU>
for cross=1:length(a)
    f=0; s=0;
    
    %the intersection at initial position, i think this shouldn't matter
    if X(1)==a(cross) && Y(1)==b(cross) && x(1)==a(cross) && y(1)==b(cross)
        f=1; s=1;
    end
    
    for i=2:length(X)
        if ((X(i-1)>=a(cross)&&X(i)<a(cross)) || (X(i)>=a(cross)&&X(i-1)<a(cross))) && ...
           ((Y(i-1)>=b(cross)&&Y(i)<b(cross)) || (Y(i)>=b(cross)&&Y(i-1)<b(cross)))
            f=i-1;
        end
    end
    
    for i=2:length(x)
        if ((x(i-1)>=a(cross)&&x(i)<a(cross)) || (x(i)>=a(cross)&&x(i-1)<a(cross))) && ...
           ((y(i-1)>=b(cross)&&y(i)<b(cross)) || (y(i)>=b(cross)&&y(i-1)<b(cross)))
            s=i-1;
        end
    end
    
    
    %add points before intersection to list
    if f>0 && s>0
        ff=[ff f]; %#ok<AGROW>
        ss=[ss s]; %#ok<AGROW>
    end
end


%%%%%%%%%%%%%%%%%% CALCULATION %%%%%%%%%%%%%%%%%%%%%%%%%

%ff, ss, a, b are all sorted so that intersection points come sequentially
[ff,si]=sort(ff);
ss=ss(si);
a=a(si);
b=b(si);

%previous and initial values and indices
ap=X(1); bp=Y(1);   %previous crossing points
i1=1; i2=1; %initial index after cross on curve
e1=1; e2=1; %#ok<NASGU> %ending index before cross on curve

%calculate the area one by one
for i=1:length(ff)
    e1=ff(i);
    e2=ss(i);
    x1=[ap X(i1:e1) a(i) fliplr(x(i2:e2)) ap];
    y1=[bp Y(i1:e1) b(i) fliplr(y(i2:e2)) bp];
    ar = ar + polyarea(x1,y1);
    
    %set previous values
    ap=a(i);
    bp=b(i);
    i1=ff(i)+1;
    i2=ss(i)+1;
    
    %used to draw with different colors
    if i==1; color='c'; end;
    if i==2; color='b'; end;
    if i==3; color='y'; end;
    if i==4; color='r'; end;
    if i==5; color='k'; end;
    if i==6; color='m'; end;
    if i==7; color='g'; end;
    if i==8; color='c'; end;
    if i==9; color='b'; end;
    if i==10; color='y'; end;
    if i==11; color='r'; end;
    if i==12; color='k'; end;
    if i==13; color='m'; end;
    if i==14; color='g'; end;
    
    if isdisp; disp(['cross at: ' num2str(ff(i)) ' x ' num2str(ss(i))]); end;
    if isplot; area(x1,y1,'FaceColor',color); end;
%     disp([num2str(a(i)) ' x ' num2str(b(i))]);
%     plot(a(i),b(i),'g+');
%     plot(X(ff(i)),Y(ff(i)),'r.');
%     plot(x(ss(i)),y(ss(i)),'r.');
end

if isdisp; disp(['area: ' num2str(ar)]); end;