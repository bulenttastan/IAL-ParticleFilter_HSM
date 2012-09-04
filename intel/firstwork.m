close all; clear; clc;

addpath('..');
outline = dlmread('../data/humanpath/outline_intel.dat');
oy    = outline(:,2);
ox    = outline(:,3);
% the lines below are for generating the HSM environment
% figure(1)
% plot(oy,ox,'.r')
% title('outline of experimental area')
% xlabel('y [m]')
% ylabel('x [m]')
% axis equal
% load data/humanpath/points;
% hold on;
% plot(px,py,'.b');
% [px py]=getpts();
% save data/humanpath/points2 px py;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
im = zeros(250,350);
for i=1:length(ox)
    im(200-round(ox(i)*10)  ,  round(oy(i)*10)+250) = 1;
end
% imshow(im);

con=[1 1 1;1 1 1;1 1 1];









runhough=0;
if runhough
    

    BW = edge(im,'canny');
    [H,T,R] = hough(BW);
    imshow(H,[],'XData',T,'YData',R,...
                'InitialMagnification','fit');
    xlabel('\theta'), ylabel('\rho');
    axis on, axis normal, hold on;
    P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
    x = T(P(:,2)); y = R(P(:,1));
    plot(x,y,'s','color','white');
    % Find lines and plot them
    lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
    figure, imshow(im), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
       plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

       % Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

       % Determine the endpoints of the longest line segment
       len = norm(lines(k).point1 - lines(k).point2);
       if ( len > max_len)
          max_len = len;
          xy_long = xy;
       end
    end

    % highlight the longest line segment
    plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','blue');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% MAP GENERATION %%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






% plot one of the data files:
% dat = dlmread('../data/humanpath/tracks_new_01-21-08.15.59.54.out');
dat = dlmread('../data/humanpath/tracks_new_01-22-08.10.12.48.out');
% dat = dlmread('../data/humanpath/tracks_new_01-23-08.14.50.06.out');
% time [s]  id  x [m]  y [m]  vel_x [m/s]  vel_y [m/s]
% time is since the beginning of the run. 
time = dat(:,1);
id   = dat(:,2);
y    = dat(:,3);
x    = dat(:,4);
vx   = dat(:,5);
vy   = dat(:,6);


uid = unique(id);


load '../data/humanpath/points2';

%%%%%%%%%%%%%%%%%%%%% CHECK %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
runcheck=0;
if runcheck
    plot(px,py,'.r');
    axis equal;
    hold on;
    %72 30 74 149
    track=74;
    tracklist=[72 120 147 155];
    for i=1:length(tracklist)
        track=tracklist(i);
        ux = x(id == uid(track));
        uy = y(id == uid(track));

        gp(3*i-2,1)=uy(1);
        gp(3*i-2,2)=ux(1);
        gp(3*i-1,1)=uy(3);
        gp(3*i-1,2)=ux(3);
        gp(3*i,1)=uy(end);
        gp(3*i,2)=ux(end);

        h=plot(ux,uy,'Color',rand(1,3));
        title(track);
        waitforbuttonpress;
        delete(h);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% 4 PAIRS OF GOALS %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rungoals=0;
if rungoals
%     gp = [-11.2974 3.4093;
%            -2.3993 5.7140;
%            -1.7766 4.3127;
%             3.2751 7.1243;
%            -2.0212 0.6157];
    tracklist=[72 120 147 155];
    track = 72;
    gx = x(id == uid(track));
    gy = y(id == uid(track));
    axis equal;
    hold on;
    
    plot(px,py,'.r');
    plot(gx,gy,'LineWidth',2);
    plot(gx(1),gy(1),'xb','LineWidth',2);
    plot(gx(end),gy(end),'xb','LineWidth',2);
 
    figure;
    axis equal;
    hold on;


    X = gx-gx(1);
    Y = gy-gy(1);
    px = px-gx(1);
    py = py-gy(1);
    br = find(X~=X(1),4,'first'); br=br(end);   % correction pos
    % creating rot mat for once
    SIN = (X(br)-X(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
    COS = (Y(br)-Y(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
    rotmat = [COS -SIN;
              SIN COS];
    a = rotmat * [X';Y'];
    b = rotmat * [px';py'];
    X = a(1,:);
    Y = a(2,:);
    px = b(1,:);
    py = b(2,:);

    plot(X,Y,'LineWidth',2);


    % HSM path
    [b,kg,c1,c2,ko,c3,c4,V] = params();
    time=linspace(time(1),time(end),10);
    [t,y]=observer(X(end),Y(end),px,py,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
    hx = y(:,3);
    hy = y(:,4);
    plot(hx,hy,'g','LineWidth',2);
    legend('Human Path','HSM');
    plot(px,py,'.r');

    area=aream(X,Y,hx,hy,0,0)
    ssd=sumsquare(X,Y,hx,hy,t,0)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%% CLUSTERING %%%%%%%%%%%%%%%%%%%%
runcluster=0;
if runcluster
    hold on;
    plot(ox,oy,'.r')
    X = [x y];
    K = 12;
    [cidx, ctrs] = kmeans(X,K);
    plot(ctrs(:,2),ctrs(:,1),'ko','LineWidth',3);
    
    % for i=1:length(ctrs(:,1))
    %     title(i);
    %     [ctrs(i,2),ctrs(i,1)]
    %     h=plot(ctrs(i,2),ctrs(i,1),'g.');
    %     waitforbuttonpress;
    %     delete(h);
    % end

    title('tracks - paths')
    xlabel('y [m]')
    ylabel('x [m]')
    axis equal
    axis([-15 6 -2 10]);
    hold off;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
