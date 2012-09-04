function scenario1()
clear; clc; close all;
load data/scenario1;
A=X; B=Y; TT=T; OBX=OX; OBY=OY;
start=1;
stop=5;
increment=.1;
% occy=[7 9];

e=zeros(floor((stop-start)/increment+1),3);
f=zeros(floor((stop-start)/increment+1),3);
c=0;
d=0;
for index=1:size(X,1)
    X=A(index,:);
    Y=B(index,:);
    OX=OBX(index,:);
    OY=OBY(index,:);
    T=TT(index,:);
    figure();hold on;
    
    [e(:,:),c]=occlusion(start,increment,stop,X,Y,T,OX,OY,1,1);
    f=f+e;
    d=d+c;
end


f=f/2;
for i=1:size(f,1)
    disp([num2str(f(i,1)) ' & ' num2str(f(i,2)) ' & ' num2str(f(i,3)) ' \\']);
end
disp(d/2);

name='Occlusion Scenario';
fig = figure('Position',[400 600 220 200],'Name',name);
dat = [f(:,1) f(:,2) f(:,3)]; 
cnames = {'Time','Simple','Fajen et al.'}; 
uitable('Data',dat,'ColumnNames',cnames,... 
            'Parent',fig,'Position',[20 20 250 750]);

%     plot([-5 5],[occy(1) occy(1)],'c');
%     plot([-5 5],[occy(2) occy(2)],'c');

%     ind1=min(find(y(:,4)>occy(1)));
%     ind2=min(find(y(:,4)>occy(2)));
    
%     slope = (y(ind1,4)-y(ind1-1,4))/(y(ind1,3)-y(ind1-1,3));
%     slope = 1/slope;
%     NX=zeros(1,size(y,1));
%     NY=zeros(1,size(y,1));
%     NX(1:ind1)=y(1:ind1,3)';
%     NY(1:ind1)=y(1:ind1,4)';   
%     NX(ind2:end)=y(ind2:end,3)';
%     NY(ind2:end)=y(ind2:end,4)';

%     ind1=min(find(Y>occy(1)));
%     ind2=min(find(Y>occy(2)));
%     
%     slope = (Y(ind1)-Y(ind1-1))/(X(ind1)-X(ind1-1));
%     slope = 1/slope;
%     
%     NX=zeros(1,size(X,2));
%     NY=zeros(1,size(Y,2));
%     NX(1:ind1)=X(1:ind1);
%     NY(1:ind1)=Y(1:ind1);
%     NX(ind2:end)=X(ind2:end);
%     NY(ind2:end)=Y(ind2:end);
    
%     for i=ind1:ind2
%         NX(i) = NX(ind1)+((occy(2)-occy(1))/(ind2-ind1))*(i-ind1)*slope;
%         NY(i) = NY(ind1)+((occy(2)-occy(1))/(ind2-ind1))*(i-ind1);
%     end
    
    
%     plot(NX,NY,'g')
    
    
%     X=NX;
%     Y=NY;
%     
%     %sum square calc
%     ss=sumsquare(X,Y,y(:,3),y(:,4),t);
%     disp(ss)
