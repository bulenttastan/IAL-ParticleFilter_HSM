clear; clc; close all;
load data/layout9;
XX=X; YY=Y;TT=T;OXX=OX;OYY=OY;
for id=1:1
% id=1;
X=XX(id,:);Y=YY(id,:);T=TT(id,:);OX=OXX(id,:);OY=OYY(id,:);

isrunning =  [0 0 0 1];
issaveframes=[0 0 0 0];
videofilename = '';
lineWidth = 3;


isPlot=1;
NumParticles=1500;
pvector=zeros(NumParticles,2);  %pvector=10*rand(NumParticles,2)-5;
% ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
Z=[X' Y'];
if ~isempty(videofilename); mov = avifile(['results\videos\' videofilename],'compression','Cinepak'); end


distn = sum(sqrt(diff(Z(:,1)).^2+diff(Z(:,2)).^2));
pert=distn/(length(Z)-1); %pert=.4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% FAJEN WARREN %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isrunning(4)
    visx=[];
    visy=[];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Z=[X' Y'];
    %applying Fajen method
    b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6; V=3.1119437730616886;
    [t,y]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,T);
    x=y(:,3);
    y=y(:,4);
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
%         observe=false;
%         if dur==7
%             visx=[visx OX(9)];
%             visy=[visy OY(9)];
%             observe=true;
%         elseif dur==8
%             visx=[visx OX(5)];
%             visy=[visy OY(5)];
%             observe=true;
%         elseif dur==9
%             visx=[visx OX(8)];
%             visy=[visy OY(8)];
%             observe=true;
%         elseif dur==13
%             visx=[visx OX(4) OX(6) OX(7)];
%             visy=[visy OY(4) OY(6) OY(7)];
%             observe=true;
%         elseif dur==18
%             visx=[visx OX(3)];
%             visy=[visy OY(3)];
%             observe=true;
%         elseif dur==24
%             visx=[visx OX(2) OX(1)];
%             visy=[visy OY(2) OY(1)];
%             observe=true;
%         end
%         if observe
%             [t,y]=observer(X(end),Y(end),visx,visy,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,T);
%             x=y(:,3);
%             y=y(:,4);
%             set(FajenHandle,'XData',x(10:29),'YData',y(10:29));
%             plot(visx,visy,'ro','LineWidth',lineWidth);
%         end
        if dur>9&&dur<30
            [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [x,y], [OX',OY'], [], NumParticles,1);
        else
            [pvector,path(dur,:)] = ParticleFilter_ex(pvector, [], [], Z(dur,:), NumParticles,1);
        end
        set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

        if isPlot; plot(path(dur,1),path(dur,2),'.k'); end
        drawnow;

        %writing to movie file
        if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
        if issaveframes(4); saveas(gca,['results/PF/fajen/fj_' num2str(dur) '.png']);end

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