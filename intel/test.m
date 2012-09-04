close all; clear; clc;

pts = [0 5; 1 1; 1 2; 3 1; 3 3; 5 3; 5 6];
% pts = [1 1; 1 2; 3 1; 3 3];

%find the voronoi path
axis equal;
hold on;
[vorx vory] = voronoi(pts(:,1),pts(:,2));
plot(vorx,vory);

vx=zeros(1,size(vorx,2)-1);
vy=zeros(1,size(vory,2)-1);
index=1;

%find all different points on voronoi path
for i=1:size(vorx,2)
    if isempty( intersect(find(vx==vorx(1,i)), find(vy==vory(1,i))) )
        vx(index) = vorx(1,i);
        vy(index) = vory(1,i);
        index = index + 1;
    end
    if isempty( intersect(find(vx==vorx(2,i)), find(vy==vory(2,i))) )
        vx(index) = vorx(2,i);
        vy(index) = vory(2,i);
        index = index + 1;
    end
end

costmat = inf(length(vx));
for i=1:length(vx)
    costmat(i,i) = 0;
end
%setup the cost map between points
for i=1:size(vorx,2)
    f = intersect(find(vx==vorx(1,i)), find(vy==vory(1,i)));
    s = intersect(find(vx==vorx(2,i)), find(vy==vory(2,i)));
    dist = sqrt((vx(f)-vx(s))^2 + (vy(f)-vy(s))^2);
    costmat(f,s) = dist;
    costmat(s,f) = dist;
end





pnt = [0 0];%is the point
close all;
dist = inf;
node = [];

for i=1:size(vorx,2)
    Ax=vorx(1,i);
    Ay=vory(1,i);
    Bx=vorx(2,i);
    By=vory(2,i);
    Cx=pnt(1);
    Cy=pnt(2);
    
    
    [dst,Px,Py,r] = point2LineSegment(Ax,Ay,Bx,By,Cx,Cy,0,0);
    
    if dst<dist
        dist = dst;
        minr = r;
        if r<0
            node = [Ax Ay];
        elseif r>1
            node = [Bx By];
        else
            if ((Px-Ax)^2+(Py-Ay)^2) < ((Px-Bx)^2+(Py-By)^2) %A is closer
                node = [Ax Ay];
            else
                node = [Bx By];
            end
        end
    end
end

start = intersect(find(vx==node(1)), find(vy==node(2)));


pnt = [5 5];%is the point
dist = inf;
node = [];

for i=1:size(vorx,2)
    Ax=vorx(1,i);
    Ay=vory(1,i);
    Bx=vorx(2,i);
    By=vory(2,i);
    Cx=pnt(1);
    Cy=pnt(2);
    
    
    [dst,Px,Py,r] = point2LineSegment(Ax,Ay,Bx,By,Cx,Cy,0,0);
    
    if dst<dist
        dist = dst;
        minr = r;
        if r<0
            node = [Ax Ay];
        elseif r>1
            node = [Bx By];
        else
            if ((Px-Ax)^2+(Py-Ay)^2) < ((Px-Bx)^2+(Py-By)^2) %A is closer
                node = [Ax Ay];
            else
                node = [Bx By];
            end
        end
    end
end

finish = intersect(find(vx==node(1)), find(vy==node(2)));
hold on;
plot(vx(start),vy(start),'ko','LineWidth',4);
plot(vx(finish),vy(finish),'ko','LineWidth',4);
disp(['Start: ' num2str(start)]);
disp(['Finish: ' num2str(finish)]);





%find the dijkstra shortest path
[path,cost] = dijkstra(costmat,start,finish);
for i=1:length(path)-1
    plot([vx(path(i)) vx(path(i+1))],[vy(path(i)) vy(path(i+1))],'k','LineWidth',3);
end


% for i=1:size(vorx,2)
%     hold on;
%     axis equal;
%     Ax=vorx(1,i);
%     Ay=vory(1,i);
%     Bx=vorx(2,i);
%     By=vory(2,i);
%     Cx=pnt(1);
%     Cy=pnt(2);
%     
% %     Let the point be C (Cx,Cy) and the line be AB (Ax,Ay) to (Bx,By)
% %     Let P be the point of perpendicular projection of C on AB.  The parameter
% %     r, which indicates P's position along AB, is computed by the dot product
% %     of AC and AB divided by the square of the length of AB:
%     %length of line
%     L = sqrt((Bx-Ax)^2 + (By-Ay)^2);
% %            AC dot AB
% %         r = --------- 
% %             ||AB||^2
% % 
% %         r has the following meaning:
% % 
% %             r=0      P = A
% %             r=1      P = B
% %             r<0      P is on the backward extension of AB
% %             r>1      P is on the forward extension of AB
% %             0<r<1    P is interior to AB
%     r = ((Cx-Ax)*(Bx-Ax) + (Cy-Ay)*(By-Ay)) / L^2;
%     
%     Px = Ax + r*(Bx-Ax);
%     Py = Ay + r*(By-Ay);
%     
%     %plot the points
%     plot([Ax Bx],[Ay By]);
%     plot(Cx,Cy,'r.');
%     plot(Px,Py,'g.');
%     text(Ax,Ay,'A');
%     text(Bx,By,'B');
%     
%     if r<0
%         dst = sqrt((Cx-Ax)^2 + (Cy-Ay)^2);
%         disp(['r: ' num2str(r) '  -> left of A  and dist: ' num2str(dst)]);
%     elseif r>1
%         dst = sqrt((Cx-Bx)^2 + (Cy-By)^2);
%         disp(['r: ' num2str(r) '  -> right of B  and dist: ' num2str(dst)]);
%     else
%         dst = sqrt((Cx-Px)^2 + (Cy-Py)^2);
%         disp(['r: ' num2str(r) '  -> inside of AB  and dist: ' num2str(dst)]);
%     end
%     
%     waitforbuttonpress;
% %     clf('reset');
% end
