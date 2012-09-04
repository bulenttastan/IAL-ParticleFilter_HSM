close all; clear; clc;
addpath('..');
outline = dlmread('../data/humanpath/outline_intel.dat');
dat = dlmread('../data/humanpath/tracks_new_01-21-08.15.59.54.out');
% dat = dlmread('../data/humanpath/tracks_new_01-22-08.10.12.48.out');
% dat = dlmread('../data/humanpath/tracks_new_01-23-08.14.50.06.out');
load '../data/humanpath/points2';
id   = dat(:,2);
y    = dat(:,3);
x    = dat(:,4);
uid = unique(id);
acceptlist = [5,9,12,42,46,52,63,64]; %tracks_new_01-21-08.15.59.54
% acceptlist = [11,18,19,30,31,38,41,43,44,45,50,51,72,73,74,140,144,147,150,154,155]; %tracks_new_01-22-08.10.12.48
% acceptlist = [1,2,3,24,26,33,35,37,40,46,51,58,64,65,69,71,73,80,83,85,92,93];   %tracks_new_01-23-08.14.50.06


XX=X; YY=Y;TT=T;OXX=OX;OYY=OY;
for id=1:length(acceptlist)

X=XX(id,:);Y=YY(id,:);T=TT(id,:);OX=px;OY=py;

isrunning =  [1 0];
issaveframes=[0 0];
videofilename = '';
lineWidth = 3;


isPlot=0;
NumParticles=1500;
pvector=zeros(NumParticles,2);  %pvector=10*rand(NumParticles,2)-5;
% ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
Z=[X' Y'];
if ~isempty(videofilename); mov = avifile(['..\results\videos\' videofilename],'compression','Cinepak'); end


distn = sum(sqrt(diff(Z(:,1)).^2+diff(Z(:,2)).^2));
pert=distn/(length(Z)-1); %pert=.4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% FAJEN WARREN %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isrunning(1)
    visx=[];
    visy=[];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Z=[X' Y'];
    %applying Fajen method
    b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6; V=3.1119437730616886;
    [t,yy]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,T);
    x=yy(:,3);
    y=yy(:,4);
%     [x,y]=equitimepath(40,x2,y2,t);

    Z(10:29,1)=x(10:29);
    Z(10:29,2)=y(10:29);
    
    % plotting Fajen-Warren
    pf=figure('Position',[400,100,800,800]); hold on; axis([-9,9,0,18],'square');
    plot(X,Y,'r','LineWidth',lineWidth);
    FajenHandle=plot(Z(10:29,1),Z(10:29,2),'g--','LineWidth',lineWidth);
    plot(OX,OY,'ro','LineWidth',lineWidth);
    plot(X(end),Y(end),'+r','LineWidth',lineWidth);
%     plot(OX,OY,'wo','LineWidth',lineWidth);
    legend('Ground Truth','HSM','Obstacles','Goal','Location','NW');
    ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
    pvector=zeros(NumParticles,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for dur=1:length(Z)
        if dur>9&&dur<30
            [pvector,path(dur,:)] = ParticleFilter(pvector, [x,y], [], NumParticles,1);
            % [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [x,y], [OX',OY'], [], NumParticles,1);
        else
            [pvector,path(dur,:)] = ParticleFilter(pvector, [], Z(dur,:), NumParticles,1);
            % [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [], [], Z(dur,:), NumParticles,1);
        end
        set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

        if isPlot; plot(path(dur,1),path(dur,2),'.k'); end
        drawnow;

        %writing to movie file
        if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
        if issaveframes(1); saveas(gca,['../results/intel/int_' num2str(dur) '.png']);end

        pause(.1);
    end

    ar(id)=aream(X,Y,path(:,1),path(:,2),0,0);
    ss(id)=sumsquare(X,Y,path(:,1),path(:,2),T,0);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

end
if ~isempty(videofilename); mov=close(mov); end


% print the table
tableresult(ss,ar,'Particle Filter Results');

z=[ss' ar'];
info.cnames=strvcat('Squared Distance','Area'); %#ok<VCAT>
info.rnames=strvcat('Motion Models','Fajen et al.'); %#ok<VCAT>
info.caption='Error comparison with different methods and measurement techniques.';
info.label='tab:comp';
table2latex(z,info);