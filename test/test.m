% clear;clc;
addpath('../');
% load ../data/withobstacle; C=X; D=Y; 
load ../data/samedistance; C=X; D=Y; id=1;
%for b=1:1
    kg=10; c1=1; c2=1;
    b=3.45; kg=10; c1=1; c2=1;%b=4.05; kg=10; c1=0.3; c2=1; %ko=50; c3=3; c4=.25;%ko=158; c3=5.5; c4=0.4;
% [t,y]=observer(C(id,40),D(id,40),OX(id),OY(id),3.1119437730616886,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
    [t,y]=observer(C(id,40),D(id,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
    set1=[C(id,:)' D(id,:)'];
    set2=y(:,3:4);
    x1=set1(:,1); y1=set1(:,2); x2=set2(:,1); y2=set2(:,2);

    hold on;
    plot(set1(:,1),set1(:,2));
    plot(set2(:,1),set2(:,2),'r');

    sum=0;
    for i=1:numel(x2)
        distVect = (x1-x2(i)).^2+(y1-y2(i)).^2;
        [minDist,ptIndex]=min(distVect);
        minDist=min(distVect);
        sum=sum+minDist;
        plot([x2(i) x1(ptIndex)],[y2(i) y1(ptIndex)],'g');
    end
    disp(['Sum Error: ' num2str(sum)]);
%end
% set1=[0 0;
%       1 0;
%       2 0;
%       3 0;
%       7 6];
% %       5 2;
% %       7 4;
% %       7 6];
% set2=[0 0;
%       0 2;
%       %5 2;
%       8 2;
%       8 6];
% resolution=10;




% area = areaintersection(set1, set2, resolution);
% 
% parea = mypolyarea([set1(:,1)' fliplr(set2(:,1)')],[set1(:,2)' fliplr(set2(:,2)')]);
% p1=fill([set1(:,1)' fliplr(set2(:,1)')],[set1(:,2)' fliplr(set2(:,2)')],'g');
% myp = get(p1);
% nf = myp.Faces;
% nv = myp.Vertices;
% nx = myp.XData;
% ny = myp.YData;
% 
% disp(nf);
% disp(['area: ' num2str(area) ' --- polyarea: ' num2str(parea)]);


%finds the intersection points, and works
% x1=C(3,:); y1=D(3,:); x2=y(:,3); y2=y(:,4);
% x1=set1(:,1); y1=set1(:,2); x2=set2(:,1); y2=set2(:,2);
% 
% [xr,yr,indr]=curveintersect(x1,y1,x2,y2);
% plot(x1,y1,'k',x2,y2,'b');
% 
% for i=1:size(xr,1)
%     disp(['x-y-ind:  ' num2str(xr(i)) ' - ' num2str(yr(i)) ' - ' ]);%num2str(indr(i))
% end
% 
% for i=1:size(indr,1)
%     disp(['ind:  ' num2str(indr(i))]);
%     plot(x2(indr(i)),y2(indr(i)),'ro');
%     plot(x2(indr(i)-6),y2(indr(i)-6),'ro');
% end
% 
% area = 0;
% numel(indr)
% numel(xr)
% if numel(indr)
%     for i=1:size(indr,1)
% 
%     end
% else
%     area = polyarea([x1' fliplr(x2')],[y1' fliplr(y2')]);
% end
% 
% disp(['area: ' num2str(area)]);
% % p1=fill([set1(:,1)' fliplr(set2(:,1)')],[set1(:,2)' fliplr(set2(:,2)')],'g');









