clear; clc; close all;
load data/layout9;
id=1;
X=X(id,:);Y=Y(id,:);T=T(id,:);OX=OX(id,:);OY=OY(id,:);

isrunning =  [0 0 1 1];
issaveframes=[0 0 0 0];
videofilename = '';
lineWidth = 3;


% plot(X(15:29),Y(15:29),'k','LineWidth',lineWidth+1);
% plot(OX,OY,'ro','LineWidth',lineWidth+1);
% plot(X(end),Y(end),'+r','LineWidth',lineWidth+1);
% legend('Ground Truth','Occlusion','Obstacles','Goal','Location','NW');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

isPlot=1;
NumParticles=1500;
pvector=zeros(NumParticles,2);  %pvector=10*rand(NumParticles,2)-5;
% ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
Z=[X' Y'];
if ~isempty(videofilename); mov = avifile(['results\videos\' videofilename],'compression','Cinepak'); end


distn = sum(sqrt(diff(Z(:,1)).^2+diff(Z(:,2)).^2));
pert=distn/(length(Z)-1); %pert=.4;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% CONSTANT VELOCITY %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isrunning(1)
    %occlude part of the map
    for i=16:30
        Z(i,1)=Z(i-1,1)+(Z(i-1,1)-Z(i-2,1));
        Z(i,2)=Z(i-1,2)+(Z(i-1,2)-Z(i-2,2));
    end
    
    % plotting Constant Velocity
    pf=figure('Position',[400,100,800,800]); hold on; axis([-9,9,0,18],'square');
    plot(X,Y,'r','LineWidth',lineWidth);
    plot(Z(15:29,1),Z(15:29,2),'c-.','LineWidth',lineWidth);
    plot(OX,OY,'ro','LineWidth',lineWidth);
    plot(X(end),Y(end),'+r','LineWidth',lineWidth);
    legend('Ground Truth','Constant Velocity','Obstacles','Goal','Location','NW');
    ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    path = zeros(length(Z),2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    for dur=1:length(Z)
        
        if dur>15&&dur<30
            [pvector,path(dur,:)] = ParticleFilter(pvector, Z, [], NumParticles,1);
        else
            [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
        end
        set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

        if isPlot; plot(path(dur,1),path(dur,2),'.k'); end
        drawnow;

        %writing to movie file
        if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
        if issaveframes(1); saveas(gca,['results/PF/constant velocity/cv_' num2str(dur) '.png']);end

        pause(.1);
    end

    ar(1)=aream(X,Y,path(:,1),path(:,2),0,0);
    ss(1)=sumsquare(X,Y,path(:,1),path(:,2),T,0);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
end



%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% SHORTCUT %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

if isrunning(2)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Z=[X' Y'];
    
    lastpos= Z(15,:) + (Z(40,:)-Z(15,:))./25.*15;
    [Z(16:30,1) Z(16:30,2)]=equitimepath(15,[Z(15,1);lastpos(1)],[Z(15,2);lastpos(2)],[0 5]);
    
    % plotting Shortcut
    pf=figure('Position',[400,100,800,800]); hold on; axis([-9,9,0,18],'square');
    plot(X,Y,'r','LineWidth',lineWidth);
    plot(Z(15:30,1),Z(15:30,2),'g:','LineWidth',lineWidth);
    plot(OX,OY,'ro','LineWidth',lineWidth);
    plot(X(end),Y(end),'+r','LineWidth',lineWidth);
    legend('Ground Truth','Shortcut','Obstacles','Goal','Location','NW');
    ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
    
    pvector=zeros(NumParticles,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for dur=1:length(Z)
        
        [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
        set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

        if isPlot; plot(path(dur,1),path(dur,2),'.k'); end
        drawnow;

        %writing to movie file
        if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
        if issaveframes(2); saveas(gca,['results/PF/shortcut/sh_' num2str(dur) '.png']);end

        pause(.1);
    end

    ar(2)=aream(X,Y,path(:,1),path(:,2),0,0);
    ss(2)=sumsquare(X,Y,path(:,1),path(:,2),T,0);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% VORONOI %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isrunning(3)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % axis([-17 17 -10 24],'square');
    Z=[X' Y'];
    [vx,vy]=voronoi(OX,OY);
    % for i=1:18
    %     if ~isempty(find(i==[2 3 4 5 7 10 13 17]))
    %         plot([vx(1,i) vx(2,i)],[vy(1,i) vy(2,i)])
    %         disp(i)
    %         plot(vx(1,i),vy(1,i),'.g')
    %         plot(vx(2,i),vy(2,i),'.r')
    %         disp(vx(1,i))
    %         disp(vx(2,i))
    %         waitforbuttonpress
    %     end
    % end
    curvex=zeros(9,1);
    curvey=zeros(9,1);
    t=zeros(9,1);
    curvex(1,1)=vx(2,17);
    curvex(2,1)=vx(1,17);
    curvex(3,1)=vx(2,10);
    curvex(4,1)=vx(2,7);
    curvex(5,1)=vx(2,5);
    curvex(6,1)=vx(2,4);
    curvex(7,1)=vx(2,3);
    curvex(8,1)=vx(2,2);
    curvex(9,1)=vx(2,13);
    curvey(1,1)=vy(2,17);
    curvey(2,1)=vy(1,17);
    curvey(3,1)=vy(2,10);
    curvey(4,1)=vy(2,7);
    curvey(5,1)=vy(2,5);
    curvey(6,1)=vy(2,4);
    curvey(7,1)=vy(2,3);
    curvey(8,1)=vy(2,2);
    curvey(9,1)=vy(2,13);

    for i=2:9
        t(i) = t(i-1) + sqrt((curvex(i)-curvex(i-1))^2+(curvey(i)-curvey(i-1))^2);
    end
    [x,y]=equitimepath(100,curvex,curvey,t);
    
    % plotting Voronoi
    pf=figure('Position',[400,100,800,800]); hold on; axis([-9,9,0,18],'square');
    plot(X,Y,'r','LineWidth',lineWidth);
    plot(curvex,curvey,'m--','LineWidth',lineWidth);
    plot(OX,OY,'ro','LineWidth',lineWidth);
    plot(X(end),Y(end),'+r','LineWidth',lineWidth);
    legend('Ground Truth','Voronoi','Obstacles','Goal','Location','NW');
    ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
    
    pvector=zeros(NumParticles,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for dur=1:length(Z)

        if dur>15&&dur<30
            %use voronoi as observation by finding the closest point on voronoi
            [q1,q2]=closestpoint([x,y],Z(dur,:));
            [pvector,path(dur,:)] = ParticleFilter(pvector, [], [q1,q2], NumParticles,1);
        else
            [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
        end
        set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

        if isPlot; plot(path(dur,1),path(dur,2),'.k'); end
        drawnow;

        %writing to movie file
        if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
        if issaveframes(3); saveas(gca,['results/PF/voronoi/vn_' num2str(dur) '.png']);end

        pause(.1);
    end
    ar(3)=aream(X,Y,path(:,1),path(:,2),0,0);
    ss(3)=sumsquare(X,Y,path(:,1),path(:,2),T,0);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% FAJEN WARREN %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isrunning(4)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Z=[X' Y'];
    %applying Fajen method
    b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6; V=3.1119437730616886;
    [t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,T);
    x2=y(:,3);
    y2=y(:,4);
    [x,y]=equitimepath(40,x2,y2,t);

    Z(16:29,1)=x(16:29);
    Z(16:29,2)=y(16:29);
    
    % plotting Fajen-Warren
    pf=figure('Position',[400,100,800,800]); hold on; axis([-9,9,0,18],'square');
    plot(X,Y,'r','LineWidth',lineWidth);
    plot(Z(15:29,1),Z(15:29,2),'g--','LineWidth',lineWidth);
    plot(OX,OY,'ro','LineWidth',lineWidth);
    plot(X(end),Y(end),'+r','LineWidth',lineWidth);
    legend('Ground Truth','HSM','Obstacles','Goal','Location','NW');
    ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
    pvector=zeros(NumParticles,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for dur=1:length(Z)

        if dur>15&&dur<30
            [pvector,path(dur,:)] = ParticleFilter(pvector, [x,y], [], NumParticles,1);
        else
            [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
        end
        set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

        if isPlot; plot(path(dur,1),path(dur,2),'.k'); end
        drawnow;

        %writing to movie file
        if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
        if issaveframes(4); saveas(gca,['results/PF/fajen/fj_' num2str(dur) '.png']);end

        pause(.1);
    end

    ar(4)=aream(X,Y,path(:,1),path(:,2),0,0);
    ss(4)=sumsquare(X,Y,path(:,1),path(:,2),T,0);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
    
    

if ~isempty(videofilename); mov=close(mov); end


% print the table
tableresult(ss,ar,'Particle Filter Results');

z=[ss' ar'];
info.cnames=strvcat('Squared Distance','Area'); %#ok<VCAT>
info.rnames=strvcat('Motion Models','Last Direction','Shortcut','Voronoi','Fajen et al.'); %#ok<VCAT>
info.caption='Error comparison with different methods and measurement techniques.';
info.label='tab:comp';
table2latex(z,info);