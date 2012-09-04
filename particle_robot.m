function [x,y]=particle_robot(visx,visy,robot)
close all;
G = [0 15.2]; %goal
T = 0:.2:5;

issaveframes=0;
videofilename = '';
lineWidth = 3;


isplot=1;
if ~isempty(videofilename); mov = avifile(['results\videos\' videofilename],'compression','Cinepak'); end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% FAJEN WARREN %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
b=3.45; kg=10; c1=1; c2=1;ko=300; c3=4.5; c4=0.6; V=3.1119437730616886;

[t,y]=observer(G(1),G(2),visx,visy,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,T);
x=y(:,3);
y=y(:,4);

% plotting Fajen-Warren
if isplot
    figure('Position',[400,100,600,600]); hold on; axis([-9,9,0,18],'square');
    plot(x,y,'g--','LineWidth',lineWidth);
    plot(robot(1),robot(2),'ko','LineWidth',10);
    plot(visx,visy,'ro','LineWidth',lineWidth);
    plot(G(1),G(2),'+r','LineWidth',lineWidth);

    if isempty(visx)
        legend('HSM','Robot','Goal','Location','NW');
    else
        legend('HSM','Robot','Obstacles','Goal','Location','NW');
    end
    angle=robot(3);
    line=[cos(angle) -sin(angle);sin(angle) cos(angle)]*[.4;0]  +  [robot(1);robot(2)];
    plot([robot(1) line(1)],[robot(2) line(2)],'w','LineWidth',2);
    drawnow;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [t,y]=observer(G(1),G(2),visx,visy,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4,T);
% x=y(:,3);
% y=y(:,4);
% if isplot
%     set(FajenHandle,'XData',x,'YData',y);
%     plot(visx,visy,'ro','LineWidth',lineWidth);
%     drawnow;
% end

%writing to movie file
if ~isempty(videofilename); F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end
if issaveframes; saveas(gca,['results/PF/fajen/fj_' num2str(dur) '.png']);end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isempty(videofilename); mov=close(mov); end


%%% print the table
% tableresult(ss,ar,'Particle Filter Results');
% 
% z=[ss' ar'];
% info.cnames=strvcat('Squared Distance','Area'); %#ok<VCAT>
% info.rnames=strvcat('Motion Models','Fajen et al.'); %#ok<VCAT>
% info.caption='Error comparison with different methods and measurement techniques.';
% info.label='tab:comp';
% table2latex(z,info);