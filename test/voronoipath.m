function path = voronoipath( X,Y,OX,OY )
%VORONOIPATH Summary of this function goes here
%   Detailed explanation goes here

path=[];

voronoi(OX,OY)
[vx vy]=voronoi(OX,OY);
% for j=1:size(vx,2)
%     disp([num2str(i) ' -> ' num2str(j)]);
%     plot(vx(:,j),vy(:,j),'c');
%     plot(vx(1,j),vy(1,j),'g.');
%     plot(vx(2,j),vy(2,j),'r.');
%     waitforbuttonpress;
% end

dist1=(vx(1,:)-X(1)).^2+(vy(1,:)-Y(1)).^2;
dist2=(vx(2,:)-X(1)).^2+(vy(2,:)-Y(1)).^2;
dist3=(vx(1,:)-vx(2,:)).^2+(vy(1,:)-vy(2,:)).^2;
[m,mi]=min(sqrt(dist1)+sqrt(dist2)-sqrt(dist3));
plot(vx(:,mi),vy(:,mi),'c');
path(1)=mi;

dist1=(vx(1,:)-X(end)).^2+(vy(1,:)-Y(end)).^2;
dist2=(vx(2,:)-X(end)).^2+(vy(2,:)-Y(end)).^2;
dist3=(vx(1,:)-vx(2,:)).^2+(vy(1,:)-vy(2,:)).^2;
[m,mi]=min(sqrt(dist1)+sqrt(dist2)-sqrt(dist3));
plot(vx(:,mi),vy(:,mi),'c');
if mi~=path(1) path(2)=mi; end