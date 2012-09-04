close all;
clear;  clc;
addpath('mysql');
if mysql('status');
    mysql('open', 'localhost', 'root', 'asalak');
    mysql('use tracker');
end

%obst start-end range for each trial [st1 en1; st2 en2;...], empty means no obst
init=196;%140;
final=196;%180;
obstrange=[188 215];%[156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;
           %156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164;156 164
           %165 173;165 173;165 173;165 173;165 173;165 173;165 173;165 173;165 173;165 173;
           %174 182;174 182;174 182;174 182;174 182];
skiplist=[];
showobserver=1;
finishrange=[];
filesave='';

saveindex=0;
skipped=0;
X=zeros(final-init+1,40);
Y=zeros(final-init+1,40);
T=zeros(final-init+1,40);
if ~isempty(obstrange);OX=zeros(final-init+1,obstrange(1,2)-obstrange(1,1)+1); %# paths, #obst on 1 path
OY=zeros(final-init+1,obstrange(1,2)-obstrange(1,1)+1); end


for trial=init:final
    % skip list
    if find(trial==skiplist); skipped=skipped+1; continue; end
    figure();  title(int2str(trial));  hold on;
    saveindex=saveindex+1;
    
    query = strcat('select x,y,time from pathlog where trial=',int2str(trial));
    [x,y,t] = mysql(query);
    % converting and saving time
    T(saveindex,:)=str2num(datestr(datenum(t,'yyyy-mm-ddTHH:MM:SS.FFFZ'),'SS.FFF')); %#ok<ST2NM>
    T(saveindex,:)=mod(T(saveindex,:)-T(saveindex,1),60); %assumes everything happen in 1 min

    % querying data from MySQL
    if ~isempty(obstrange)
        [ox,oy]=mysql(['select x,y from obstacle where trial>=' int2str(obstrange(saveindex+skipped,1)) ...
                                                 ' and trial<=' int2str(obstrange(saveindex+skipped,2))]);
        % adding obstacles on the path
        for i=1:size(OX,2)
            OX(saveindex,i) = ox(i)-x(1);
            OY(saveindex,i) = oy(i)-y(1);
        end
    end
    
    if ~isempty(finishrange)
        [FX,FY]=mysql(['select x,y from finish where Trial>=' int2str(finishrange(1))...
                                               ' and Trial<=' int2str(finishrange(2))]);
       for i=1:size(FX,1)
            FX(i) = FX(i)-x(1);
            FY(i) = FY(i)-y(1);
        end
    end
    
    
    
    % zeroing initial position and storing
    X(saveindex,:) = x-x(1);
    Y(saveindex,:) = y-y(1);
    br = find(x~=x(1),1,'first');    % correction pos
    

    % creating rot mat for once
    SIN = (x(br)-x(1)) / sqrt((x(br)-x(1))^2 + ((y(br)-y(1))^2));
    COS = (y(br)-y(1)) / sqrt((x(br)-x(1))^2 + ((y(br)-y(1))^2));
    rotmat = [COS -SIN;
              SIN COS];

    
    % rotating paths, obstacle and finish
    a = rotmat * [X(saveindex,:);Y(saveindex,:)];
    X(saveindex,:) = a(1,:);
    Y(saveindex,:) = a(2,:);
    if ~isempty(obstrange)
        for i=1:size(OX,2)
            a = rotmat * [OX(saveindex,i);OY(saveindex,i)];
            OX(saveindex,i) = a(1);
            OY(saveindex,i) = a(2);
        end
        plot(OX(saveindex,:),OY(saveindex,:),'or');
    end
    if ~isempty(finishrange)
        for i=1:size(FX,1)
            a = rotmat*[FX(i);FY(i)];
            FX(i)=a(1);
            FY(i)=a(2);
            if showobserver
                [b,kg,c1,c2,ko,c3,c4,V]=params();
                [t,finishpath]=observer(FX(i),FY(i),OX(multindex,:),OY(multindex,:),V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
                plot(finishpath(:,3),finishpath(:,4));
            end
        end
    end
    
    if showobserver
        [b,kg,c1,c2,ko,c3,c4,V]=params();
        if ~isempty(obstrange)
            [t,path]=observer(X(saveindex,end),Y(saveindex,end),OX(saveindex,:),OY(saveindex,:),V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
        else
            [t,path]=observer(X(saveindex,end),Y(saveindex,end),[],[],V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
        end
        plot(path(:,3),path(:,4));
    end

    
    % plot corrected paths
    plot(X(saveindex,:),Y(saveindex,:),'r');
    plot(X(saveindex,1),Y(saveindex,1),'.g');
    plot(X(saveindex,end),Y(saveindex,end),'xg');
    % display goal and distance
    disp(['x:    ' num2str(X(saveindex,end))]);
    disp(['y:    ' num2str(Y(saveindex,end))]);
    disp(['dist: ' num2str(sqrt(X(saveindex,end)^2+Y(saveindex,end)^2))]);
    disp('-------------');
    
    waitforbuttonpress;
end


if ~isempty(filesave)
    X=X(1:saveindex,:);Y=Y(1:saveindex,:);T=T(1:saveindex,:);
    if ~isempty(obstrange)
        OX=OX(1:saveindex,:);OY=OY(1:saveindex,:);
        save(['data/' filesave],'X','Y','T','OX','OY');
    elseif ~isempty(finishrange)
        OX=OX(1:saveindex,:);OY=OY(1:saveindex,:);FX=FX(1:saveindex,:);FY=FY(1:saveindex,:);
        save(['data/' filesave],'X','Y','T','OX','OY','FX','FY');
    else
        save(['data/' filesave],'X','Y','T');
    end
end