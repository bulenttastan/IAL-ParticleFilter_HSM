clear; %clc; close all;
load ../data/withobstacle; C=X; D=Y;
load ../data/samedistance; A=X;B=Y;
load ../data/berkansameangle;

b=4.05;
kg=10;
c1=0.3;
c2=1;

ko=158;
c3=5.5;
c4=0.4;
hold on;

for i=1:size(C,1)
    [t,y]=observer(C(i,40),D(i,40),OX(i),OY(i));
%     first=[C(i,15:end) fliplr(y(70:end,3)')];
%     second=[D(i,15:end) fliplr(y(70:end,4)')];
    first=[C(i,:) fliplr(y(:,3)')];
    second=[D(i,:) fliplr(y(:,4)')];
%     TRI=delaunay(first,second);
%     size(TRI)
    fill(first,second,'c');
    area1 = mypolyarea(first,second);
    [t,y]=observer(C(i,40),D(i,40),OX(i),OY(i),3.1119437730616886,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
%     rotmat=[0 1;-1 0];
%     a = rotmat*[C(i,:);D(i,:)];
%     C(i,:) = a(1,:);
%     D(i,:) = a(2,:);
%     a = rotmat*[y(:,3) y(:,4)]';
%     y(:,3) = a(1,:)';
%     y(:,4) = a(2,:)';
    C(i,:)=C(i,:)+3;
    y(:,3)=y(:,3)+3;
    plot(C(i,:),D(i,:),'r');
%     fill([C(i,:) fliplr(y(:,3)')],[D(i,:) fliplr(y(:,4)')],'c');
%     area2 = polyarea([C(i,:) fliplr(y(:,3)')],[D(i,:) fliplr(y(:,4)')]);
%     fill(fliplr(y(46:64,3)'),fliplr(y(46:64,4)'),'c');
%     area2 = polyarea(fliplr(y(46:64,3)'),fliplr(y(46:64,4)'));

    first=[C(i,:) fliplr(y(:,3)')];
    second=[D(i,:) fliplr(y(:,4)')];
    
    fill(first,second,'c');
    area2 = mypolyarea(first,second);

    disp(['org: ' num2str(area1) ' --- ' num2str(area2)]);
end

% for i=1:size(X,1);
%     [t,y]=observer(X(i,40),Y(i,40),[],[]);
%     area1 = polyarea([X(i,:) fliplr(y(:,3)')],[Y(i,:) fliplr(y(:,4)')]);
%     [t,y]=observer(X(i,40),Y(i,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
%     plot(X(i,:),Y(i,:),'r');
%     x=fill([X(i,:) fliplr(y(:,3)')],[Y(i,:) fliplr(y(:,4)')],'c');
%     area2 = polyarea([X(i,:) fliplr(y(:,3)')],[Y(i,:) fliplr(y(:,4)')]);
%     disp(['org: ' num2str(area1) ' --- ' num2str(area2)]);
% end
% disp('-------');figure();
% for i=1:size(A,1);
%     [t,y]=observer(A(i,40),B(i,40),[],[]);
%     area1 = polyarea([A(i,:) fliplr(y(:,3)')],[B(i,:) fliplr(y(:,4)')]);
%     [t,y]=observer(A(i,40),B(i,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
%     plot(A(i,:),B(i,:),'r');
%     x=fill([A(i,:) fliplr(y(:,3)')],[B(i,:) fliplr(y(:,4)')],'c');
%     area2 = polyarea([A(i,:) fliplr(y(:,3)')],[B(i,:) fliplr(y(:,4)')]);
%     disp(['org: ' num2str(area1) ' --- ' num2str(area2)]);
% end

% scale = 14.5315/9;
% for i=3:5
%     observer(0,14.5426,-sin(4/180*pi)*i*scale,cos(4/180*pi)*i*scale,3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
% end
