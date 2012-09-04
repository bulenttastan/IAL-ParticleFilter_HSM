close all;
clear;  clc;
addpath('../mysql');
if mysql('status');
    mysql('open', 'localhost', 'root', 'asalak');
    mysql('use tracker');
end
isrotated=-1; %-1 for never rotate
isobstacle=1;
showobserver=1;
isdiffobst=1;
obst=[138];%[15 16 17];
ismultifinish=0;
finishrange=[29 32];
ismultiple=0;
multindex=1;
multobstrange=[119 120;121 122;124 125;126 127;128 129;130 131;132 133;134 135;134 135;136 137];
issave=0;
filename='layout1';
saveindex=0;
initialindex=120;
finalindex=120;
X=zeros(1,40);
Y=zeros(1,40);
T=zeros(1,40);
initpos=[0,0];
OX=zeros(1,1);
OY=zeros(1,1);
% test stored data, just evaluate
% load data/withobstacle
% hold on;
% plot(-2,-2,'.');
% plot(2,15,'.');
% for i=1:3
%     waitforbuttonpress;
%     plot(X(i,:),Y(i,:));
%     plot(X(i,1),Y(i,1),'.g');
%     plot(X(i,40),Y(i,40),'x');
%     plot(OX(i),OY(i),'o');
% end
for trial=initialindex:finalindex
    saveindex=saveindex+1;
    if find(trial==[46 47 48 49 50 51 52 53 56 57 61])
        %[18 19 20 23 24] %for same distance
        %[26 27 28 29 30 32 33 35 36 38 40] %for berkan's data
        continue;
    end
    
    query = strcat('select x,y,time from pathlog where trial=',int2str(trial));
    [x,y,t] = mysql(query);
    %converting and saving time
    T(saveindex,:)=datenum(t,'yyyy-mm-ddTHH:MM:SS.FFFZ');
    T(saveindex,:)=T(saveindex,:)-T(saveindex,1);
    ox=0;   oy=0;   obstindex=1;
    if isobstacle
        if isdiffobst>0
            obstindex=isdiffobst;
            isdiffobst=isdiffobst+1;
        end
        [ox,oy] = mysql(['select x,y from obstacle where trial=',int2str(obst(obstindex))]);
    end
    
    if ismultiple
        [ox,oy]=mysql(['select x,y from obstacle where trial>=' int2str(multobstrange(multindex,1)) ...
                                                 ' and trial<=' int2str(multobstrange(multindex,2))]);
        for i=1:size(OX,2)
            OX(multindex,i) = ox(i)-x(1);
            OY(multindex,i) = oy(i)-y(1);
        end
    end
    
    if ismultifinish
        [FX,FY]=mysql(['select x,y from finish where Trial>=' int2str(finishrange(1))...
                                               ' and Trial<=' int2str(finishrange(2))]);
       for i=1:size(FX,1)
            FX(i) = FX(i)-x(1);
            FY(i) = FY(i)-y(1);
        end
    end
    
    figure();
    title(int2str(trial));
    hold on;

    % zeroing orig path
    if isobstacle
        ox = ox-x(1);
        oy = oy-y(1);
        initpos=[x(1),y(1)];
    end
    x = x-x(1);
    y = y-y(1);
    br = 6;     % correction pos
  
    % creating rot mat for once
    if isrotated<1
        SIN = (x(br)-x(1)) / sqrt((x(br)-x(1))^2 + ((y(br)-y(1))^2));
        COS = (y(br)-y(1)) / sqrt((x(br)-x(1))^2 + ((y(br)-y(1))^2));
        rotmat = [COS -SIN;
                  SIN COS];
        disp('rotation matrix created');
        if isrotated==0
            isrotated=1;
        end
    end
    
    % zeroing init pos
    st=1;
    en=40;
    x = x-x(st);
    y = y-y(st);
    
    % rotating paths and obstacle
    a = rotmat*[x y]';
    x = a(1,:);
    y = a(2,:);
    if isobstacle
        a = rotmat*[ox;oy];
        ox = a(1);
        oy = a(2);
        % calc obst pos for game
%         rotmat2 = [COS SIN;
%                   -SIN COS];
%         scale = 14.5315/9;
%         distance=5;
%         b = rotmat2*[-sin(4/180*pi)*distance*scale;
%                     cos(4/180*pi)*distance*scale];
%         plot(b(1),b(2),'og');
%         b=b(:)+initpos(:);
%         disp('put obstacle here: ');
%         disp(['x: ' num2str(b(1))]);
%         disp(['y: ' num2str(b(2))]);disp('~~~~~~~~~~~');
    end
    if ismultiple
        for i=1:size(OX,2)
            a = rotmat*[OX(multindex,i);OY(multindex,i)];
            OX(multindex,i) = a(1);
            OY(multindex,i) = a(2);
        end
    end
    if ismultifinish
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
        [t,finishpath]=observer(x(end),y(end),OX(multindex,:),OY(multindex,:),V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
        plot(finishpath(:,3),finishpath(:,4));
        plot(FX,FY,'+g');
    end
    
    % storing paths    
    X(saveindex,:)=x;
    Y(saveindex,:)=y;
    if isobstacle
        OX(obstindex)=ox;
        OY(obstindex)=oy;
    end
    
    % plot corrected paths
    plot(x(st:en),y(st:en),'r');
    plot(x(st),y(st),'.g');
    plot(x(en),y(en),'x');
    if isobstacle
        plot(ox,oy,'or');
    end
    if ismultiple
        plot(OX(multindex,:),OY(multindex,:),'or');
        multindex=multindex+1;
    end
    disp(['x:    ' num2str(x(en))]);
    disp(['y:    ' num2str(y(en))]);
    disp(['dist: ' num2str(sqrt(x(en)^2+y(en)^2))]);
    disp('-------------');
    % calculate heading, goal angle, angle diff, goal distance
%     curang=0;
%     for j=st:en
%         waitforbuttonpress;
%         plot(x(j),y(j),'+g');
% %         curang = acos((y(j)-y(j-1))/sqrt((x(j)-x(j-1))^2+(y(j)-y(j-1))^2));
%         ang = acos((y(40)-y(j))/sqrt((x(40)-x(j))^2+(y(40)-y(j))^2));
%         angdif = ang-curang;
%         dist = sqrt((x(40)-x(j))^2+(y(40)-y(j))^2);
%         disp(['Cur Angle:  ',num2str(curang/pi*180)]);
%         disp(['Goal Angle: ' num2str(ang/pi*180)]);
%         disp(['Diff Angle:  ' num2str(angdif/pi*180)]);
%         disp(['Distance:   ' num2str(dist)]);
%         disp('-----');
%     end
    waitforbuttonpress;
end

if issave
    if isobstacle || ismultiple
        save(['data/' filename],'X','Y','T','OX','OY');
    else
        save(['data/' filename],'X','Y','T');
    end
end