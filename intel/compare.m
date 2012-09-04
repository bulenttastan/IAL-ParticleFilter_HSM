close all; clear; clc;

issave = false;
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

isPlot = [1 1 1 0];


% plot one of the data files:
% dat = dlmread('../data/humanpath/tracks_new_01-21-08.15.59.54.out');
% dat = dlmread('../data/humanpath/tracks_new_01-22-08.10.12.48.out');
dat = dlmread('../data/humanpath/tracks_new_01-23-08.14.50.06.out');
% time [s]  id  x [m]  y [m]  vel_x [m/s]  vel_y [m/s]
% time is since the beginning of the run. 
time = dat(:,1);
id   = dat(:,2);
y    = dat(:,3);
x    = dat(:,4);
vy   = dat(:,5);
vx   = dat(:,6);


uid = unique(id);


load '../data/humanpath/points2';




% acceptlist = [5,9,12,42,46,52,63,64]; %tracks_new_01-21-08.15.59.54
% acceptlist = [11,18,19,30,31,38,41,43,44,45,50,51,72,73,74,140,144,147,150,154,155]; %tracks_new_01-22-08.10.12.48
acceptlist = [1,2,3,24,26,33,35,37,40,46,51,58,64,65,69,71,73,80,83,85,92,93];   %tracks_new_01-23-08.14.50.06

pathlength = zeros(1,length(acceptlist));
ss = zeros(3,length(acceptlist));

for accept=1:length(acceptlist)
%for htrace=1:length(uid)
%     if ~isempty(intersect(htrace,acceptlist)); continue; end
htrace = acceptlist(accept);
ux = x(id == uid(htrace));
uy = y(id == uid(htrace));
utime = time(id == uid(htrace));
uvx = vx(id == uid(htrace)); %no need for now
uvy = vy(id == uid(htrace)); %no need for now
utime = utime - utime(1);    %time user spends


tic;
pathlength(accept) = sum(sqrt(diff(ux).^2 + diff(uy).^2));







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% HSM %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

runhsm=1;
if runhsm
    X = ux-ux(1);
    Y = uy-uy(1);
    Px = px-ux(1);
    Py = py-uy(1);
    time = time-time(1);
    br = find(X~=X(1),12,'first'); br=br(end);   % correction pos
    % creating rot mat for once
    SIN = (X(br)-X(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
    COS = (Y(br)-Y(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
    rotmat = [COS -SIN;
              SIN COS];
    a = rotmat * [X';Y'];
    b = rotmat * [Px';Py'];
    X = a(1,:);
    Y = a(2,:);
    Px = b(1,:);
    Py = b(2,:);

    
    % this should not happen
    for t=2:length(time)
        if time(t) < time(t-1)
            disp(['found at ' int2str(t)]);
        end
    end

    

    % Time calculation for each data track, some paths need extra attention
    %tracks_new_01-21-08.15.59.54
%     if htrace == 5
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 9
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.24;
%     elseif htrace == 12
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 42
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 46
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 52
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 63
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    %tracks_new_01-22-08.10.12.48
    if htrace == 18
        t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    elseif htrace == 72
        t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    elseif htrace == 74
        t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
    %tracks_new_01-23-08.14.50.06
%     if htrace == 1
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3.1;
%     elseif htrace == 2
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3.2;
%     elseif htrace == 33
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 35
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.9;
%     elseif htrace == 37
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3;
%     elseif htrace == 46
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
%     elseif htrace == 51
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.6;
%     elseif htrace == 73
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 80
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;
%     elseif htrace == 89
%         t = sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.4;

% generic is below
    elseif sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2) > 12
        t=sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/3;
    elseif sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2) > 8
        t=sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.8;
    else
        t=sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2)/2.6;
    end
    
    
    % HSM path
    [b,kg,c1,c2,ko,c3,c4,V] = params();
%     disp(['(' num2str(htrace) ') Length: ' num2str(sum(sqrt(diff(ux).^2 + diff(uy).^2)))...
%         '  -  HSM Length: ' num2str(sqrt((X(end)-X(1))^2 + (Y(end)-Y(1))^2))]);
    % convert measured t's to HSM, to get same # of hx as X
    % check length(hx) == length(X)
    t = t * utime ./ utime(end); %split time as proportional as utime
    [t,yy]=observer(X(end),Y(end),Px,Py,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,t);
    hx = yy(:,3)';
    hy = yy(:,4)';
    
    

    
    
    
    isPF=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% PARTICLE FILTER BLOCK %%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(isPF)
        issaveframes=[0 0];
        videofilename = '';
        lineWidth = 3;
        isShowMean=1;
        
        clear pvector;
        clear path;
        NumParticles=1500;
        pvector=zeros(NumParticles,2);

        if ~isempty(videofilename); mov = avifile(['..\results\videos\' videofilename],'compression','Cinepak'); end

        
        Z=[X' Y'];
        Z(40:length(X)-40,1)=hx(40:length(X)-40);
        Z(40:length(X)-40,2)=hy(40:length(X)-40);

        % Plotting Initial Figure
        if isPlot(2)
            [x1,x2,y1,y2] = outlineaxis(Px,Py);
            pf=figure('Position',[400,100,800,800]); hold on; axis([x1,x2,y1,y2],'equal','manual');
            plot(X,Y,'r','LineWidth',lineWidth);
            FajenHandle=plot(Z(40:length(X)-40,1),Z(40:length(X)-40,2),'g--','LineWidth',lineWidth);
            plot(Px,Py,'ro','LineWidth',lineWidth);
            plot(X(end),Y(end),'+r','LineWidth',lineWidth);
            legend('Ground Truth','HSM','Obstacles','Goal','Location','NW');
            ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
        end
        
        pvector=zeros(NumParticles,2);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for dur=1:length(Z)
            if dur>=40&&dur<=length(X)-40
                [pvector,path(dur,:)] = ParticleFilter(pvector, [hx',hy'], [], NumParticles,1);
                % [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [x,y], [OX',OY'], [], NumParticles,1);
            else
                [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
                % [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [], [], Z(dur,:), NumParticles,1);
            end
            if isPlot(2)
                set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));
            end
            
            if isShowMean && isPlot(2); plot(path(dur,1),path(dur,2),'.k'); end
            drawnow;

            %writing to movie file
            if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
            if issaveframes(1); saveas(gca,['../results/intel/int_' num2str(dur) '.png']);end

%             pause(.1);
        end

        arPF=aream(X,Y,path(:,1),path(:,2),0,0);
        ssPF=sumsquare(X,Y,path(:,1),path(:,2),t,0);
        ss(3,accept) = ssPF;
        if isPlot(2); delete(ParticleHandle); end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% PARTICLE FILTER ENDS %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    % rotate things back (reverse rotate), and transform for plotting
    rotmat = [COS SIN;
              -SIN COS];
    a = rotmat * [X;Y];
    b = rotmat * [hx;hy];
    c = rotmat * [Px;Py];
    X = a(1,:) + ux(1);
    Y = a(2,:) + uy(1);
    hx = b(1,:) + ux(1);
    hy = b(2,:) + uy(1);
    Px = c(1,:) + ux(1);
    Py = c(2,:) + uy(1);
    
    
    if isPlot(1)
        figure('Name',['htrace (' num2str(htrace) ')']);%,'MenuBar','none');
        title(num2str(htrace));
        [x1,x2,y1,y2] = outlineaxis(Px,Py);
        axis([x1,x2,y1,y2],'equal','manual');    hold on;
        plot(X,Y,'LineWidth',2);
        plot(hx,hy,'k--','LineWidth',2);
        legend('Human Path','Voronoi');
        plot(Px,Py,'.r');
    end
    
    
    area=aream(X,Y,hx,hy,0,0);
    ssd=sumsquare(X,Y,hx,hy,t,0);
    ss(1,accept) = ssd;
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%









%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% VORONOI %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

runvoronoi=1;
if runvoronoi
    
    [vorx,vory]=voronoi(px,py);
%     for i=1:length(vorx)
%         title(i);
%         h=plot([vorx(1,i) vorx(2,i)],[vory(1,i) vory(2,i)]);
%         waitforbuttonpress;
%         delete(h);
%     end
    
    
    
    %%%%%%%%%%%%%% VORONOI PATH CALCULATION %%%%%%%%%%%%
    % 1) Convert voronoi graph into individual points
    % 2) Setup cost matrix of voronoi graph (indexed by individual points)
    % 3) Find initial and final nodes on voronoi graph
    % 4) Get shortest path using dijsktra on cost matrix
    % 5) Measure errors
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    vox=zeros(1,size(vorx,2)-1);
    voy=zeros(1,size(vory,2)-1);
    index=1;

    % 1) find all different points on voronoi path
    %vox,voy are vertices of voronoi graph
    for i=1:size(vorx,2)
        if isempty( intersect(find(vox==vorx(1,i)), find(voy==vory(1,i))) )
            vox(index) = vorx(1,i);
            voy(index) = vory(1,i);
            index = index + 1;
        end
        if isempty( intersect(find(vox==vorx(2,i)), find(voy==vory(2,i))) )
            vox(index) = vorx(2,i);
            voy(index) = vory(2,i);
            index = index + 1;
        end
    end

    % 2) Setup cost map
    costmat = inf(length(vox));
    for i=1:length(vox)
        costmat(i,i) = 0;
    end
    %setup the cost map between points
    for i=1:size(vorx,2)
        f = intersect(find(vox==vorx(1,i)), find(voy==vory(1,i)));
        s = intersect(find(vox==vorx(2,i)), find(voy==vory(2,i)));
        dist = sqrt((vox(f)-vox(s))^2 + (voy(f)-voy(s))^2);
        costmat(f,s) = dist;
        costmat(s,f) = dist;
    end
    
    
    %%%%% 3) Find starting vertex on voronoi graph %%%
    pnt = [ux(1) uy(1)];%is first point
    dist = inf;
    node = [];
    
    for i=1:size(vorx,2)
        Ax=vorx(1,i);
        Ay=vory(1,i);
        Bx=vorx(2,i);
        By=vory(2,i);
        Cx=pnt(1);
        Cy=pnt(2);


        [dst,Pntx,Pnty,r] = point2LineSegment(Ax,Ay,Bx,By,Cx,Cy,0,0);

        if dst<dist
            dist = dst;
            minr = r;
            if r<0
                node = [Ax Ay];
            elseif r>1
                node = [Bx By];
            else
                if ((Pntx-Ax)^2+(Pnty-Ay)^2) < ((Pntx-Bx)^2+(Pnty-By)^2) %A is closer
                    node = [Ax Ay];
                else
                    node = [Bx By];
                end
            end
        end
    end

    start = intersect(find(vox==node(1)), find(voy==node(2)));
    %%%%% End of Finding initial vertex on voronoi graph %%%

    
    %%%%% 3) Find final vertex on voronoi graph %%%
    pnt = [ux(end) uy(end)];%is last point
    dist = inf;
    node = [];
    
    for i=1:size(vorx,2)
        Ax=vorx(1,i);
        Ay=vory(1,i);
        Bx=vorx(2,i);
        By=vory(2,i);
        Cx=pnt(1);
        Cy=pnt(2);


        [dst,Pntx,Pnty,r] = point2LineSegment(Ax,Ay,Bx,By,Cx,Cy,0,0);

        if dst<dist
            dist = dst;
            minr = r;
            if r<0
                node = [Ax Ay];
            elseif r>1
                node = [Bx By];
            else
                if ((Pntx-Ax)^2+(Pnty-Ay)^2) < ((Pntx-Bx)^2+(Pnty-By)^2) %A is closer
                    node = [Ax Ay];
                else
                    node = [Bx By];
                end
            end
        end
    end

    finish = intersect(find(vox==node(1)), find(voy==node(2)));
    %%%%% End of Finding final vertex on voronoi graph %%%
    
    
    if isPlot(3)
        figure();
        hold on;
        [x1,x2,y1,y2] = outlineaxis(Px,Py);
        axis([x1,x2,y1,y2],'equal','manual');
        plot(vorx,vory,'y');
        plot(vox(start),voy(start),'k.','LineWidth',2);
        plot(vox(finish),voy(finish),'k.','LineWidth',2);
        plot(px,py,'r.');
        plot(ux,uy,'LineWidth',2);
    end
%     disp(['Start: ' num2str(start)]);
%     disp(['Finish: ' num2str(finish)]);
    
    
    %%% 4) Find the dijkstra shortest path %%%
    [path,cost] = dijkstra(costmat,start,finish);
    Vx = zeros(1,length(path));
    Vy = zeros(1,length(path));
    Vx(1) = vox(path(1));
    Vy(1) = voy(path(1));
    for i=1:length(path)-1
        if isPlot(3)
            plot([vox(path(i)) vox(path(i+1))],[voy(path(i)) voy(path(i+1))],'k--','LineWidth',2);
        end
        Vx(i+1) = vox(path(i+1));
        Vy(i+1) = voy(path(i+1));
    end
    %%% End of Finding the dijkstra shortest path %%%
    
    
    t = cumsum([0 sqrt(diff(Vx).^2 + diff(Vy).^2)]);
    if length(t)<=1;
        disp(['htrace ' num2str(htrace) ' is skipped']); continue; end
    area=aream(ux,uy,Vx,Vy,0,0);
    ssd=sumsquare(ux,uy,Vx,Vy,t,0);
    ss(2,accept) = ssd;
    %%%%%%%%%%%%%% VORONOI PATH CALCULATION %%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    
    isPF=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% PARTICLE FILTER VORONOI %%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(isPF)
        %setup X,Y and rotate so initial position will be (0,0)
        X = ux-ux(1);
        Y = uy-uy(1);
        Px = px-ux(1);
        Py = py-uy(1);
        Vx = Vx-ux(1);
        Vy = Vy-uy(1);
        time = time-time(1);
        br = find(X~=X(1),12,'first'); br=br(end);   % correction pos
        % creating rot mat for once
        SIN = (X(br)-X(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
        COS = (Y(br)-Y(1)) / sqrt((X(br)-X(1))^2 + ((Y(br)-Y(1))^2));
        rotmat = [COS -SIN;
                  SIN COS];
        a = rotmat * [X';Y'];
        b = rotmat * [Px';Py'];
        c = rotmat * [Vx;Vy];
        X = a(1,:);
        Y = a(2,:);
        Px = b(1,:);
        Py = b(2,:);
        Vx = c(1,:);
        Vy = c(2,:);

    
        issaveframes=[0 0];
        videofilename = '';
        lineWidth = 3;
        isShowMean=1;
        
        clear pvector;
        clear path;
        NumParticles=1500;
        pvector=zeros(NumParticles,2);

        if ~isempty(videofilename); mov = avifile(['..\results\videos\' videofilename],'compression','Cinepak'); end

        
        Z=[X' Y'];
        
        [qx,qy,t]=equitimepath(length(X),Vx,Vy,t);
        QXY = [qx qy];

        % Plotting Initial Figure
        if isPlot(4)
            [x1,x2,y1,y2] = outlineaxis(Px,Py);
            pf=figure('Position',[400,100,800,800]); hold on; axis([x1,x2,y1,y2],'equal','manual');
            plot(X,Y,'r','LineWidth',lineWidth);
            FajenHandle=plot(Vx,Vy,'m--','LineWidth',lineWidth);
            plot(Px,Py,'ro','LineWidth',lineWidth);
            plot(X(end),Y(end),'+r','LineWidth',lineWidth);
            legend('Ground Truth','Voronoi','Obstacles','Goal','Location','NW');
            ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
        end
        
        pvector=zeros(NumParticles,2);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for dur=1:length(Z)
            if dur>=40&&dur<=length(X)-40
                [q1,q2]=closestpoint([qx,qy],Z(dur,:));
                [pvector,path(dur,:)] = ParticleFilter(pvector, [qx qy], [], NumParticles,1);
                % [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [x,y], [OX',OY'], [], NumParticles,1);
            else
                [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
                % [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [], [], Z(dur,:), NumParticles,1);
            end
            if isPlot(4)
                set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));
            end
            
            if isShowMean && isPlot(4); plot(path(dur,1),path(dur,2),'.k'); end
            drawnow;

            %writing to movie file
            if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
            if issaveframes(1); saveas(gca,['../results/intel/int_' num2str(dur) '.png']);end

%             pause(.1);
        end

        arPF=aream(X,Y,path(:,1),path(:,2),0,0);
        ssPF=sumsquare(X,Y,path(:,1),path(:,2),t,0);
        ss(4,accept) = ssPF;
        if isPlot(4); delete(ParticleHandle); end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%% PARTICLE FILTER VOR ENDS %%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





% disp(['(' num2str(htrace) ') SSD HSM: ' num2str(ss(1,accept))...
%         ' - SSD VOR: ' num2str(ss(2,accept))...
%         ' - SSD HSM PF: ' num2str(ss(3,accept))...
%         ' - SSD VOR PF: ' num2str(ss(4,accept))]);
disp(' ');
waitforbuttonpress;
toc;
end     %humantrace end

if issave; save data pathlength ss; end