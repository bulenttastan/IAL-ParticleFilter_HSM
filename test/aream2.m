function ar = aream2( X,Y,x,y,varargin )
% AREAM Calculates the area between two polygons
% X,Y first polygon
% x,y second polygon
% 2 arguments: 1 to display, 1 to plot
% Example:
% load 'data/goalmultobst';
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
isdisp=0; if nargin>4; isdisp=varargin{1}; end;
isplot=0; if nargin>5; isplot=varargin{2}; end;
if size(X,1)>size(X,2); X=X'; Y=Y'; end
if size(x,1)>size(x,2); x=x'; y=y'; end

[a,b]=curveintersect(X,Y,x,y);

ar = 0; f=0; s=0;
for cross=1:length(a)
    f=0; s=0;
    for i=2:length(X)
        if ((X(i-1)>a(cross)&&X(i)<a(cross)) || (X(i)>a(cross)&&X(i-1)<a(cross))) && ...
           ((Y(i-1)>b(cross)&&Y(i)<b(cross)) || (Y(i)>b(cross)&&Y(i-1)<b(cross)))
            f=i-1;
%             disp(['f: ' num2str(f)]);
%             plot(X(i-1),Y(i-1),'g+');
%             plot(X(i),Y(i),'g+');
        end
    end
    
    for i=2:length(x)
        if ((x(i-1)>a(cross)&&x(i)<a(cross)) || (x(i)>a(cross)&&x(i-1)<a(cross))) && ...
           ((y(i-1)>b(cross)&&y(i)<b(cross)) || (y(i)>b(cross)&&y(i-1)<b(cross)))
            s=i-1;
%             disp(['s: ' num2str(s)]);
%             plot(x(i-1),y(i-1),'g+');
%             plot(x(i),y(i),'g+');
        end
    end
    
    if f>0 && s>0
        x1=[X(1:f) a(cross) fliplr(x(1:s))];
        y1=[Y(1:f) b(cross) fliplr(y(1:s))];
        ar = ar + polyarea(x1,y1);
%         plot(p1(1,:),p1(2,:),'g');
%         plot(p2(1,:),p2(2,:),'g');
        if isplot; area(x1,y1,'FaceColor','c'); end;
        if isdisp; disp(['cross at: ' num2str(f) ' x ' num2str(s)]); end;
    end
end

if isempty(a) || (f>0 && s>0) || max(a)==0
    x2=[X(end) fliplr(x(s+1:end)) a(cross) X(f+1:end)];
    y2=[Y(end) fliplr(y(s+1:end)) b(cross) Y(f+1:end)];
    ar = ar + polyarea(x2,y2);
    if isplot; area(x2,y2,'FaceColor','g'); end;
    if isdisp; disp('polygon closed'); end;
end
if isdisp; disp(['area: ' num2str(ar)]); end;