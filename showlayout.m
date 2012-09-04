close all; clear; clc;

occstart=1; occend=4.5; interval=.5;
isplot=[0 1 1 1]; %org, fj, shc, vn
issave=[0 0 0 0];
isplotocc=0; %together occl
issaveocc=0;
lineWidth=3;

ss=zeros(3,10);
ar=zeros(3,10);
h=zeros(3,10,2);
p=zeros(3,10,2);
ksstat=zeros(3,10,2);
ssd=zeros(3,3);
are=zeros(3,3);
ksx=zeros(3,3);
ksy=zeros(3,3);
ks=zeros(6,3);
trackss=zeros(4,(occend-occstart)/interval);
trackar=zeros(4,(occend-occstart)/interval);
alpha=.05;
ind=0;

for layout=[1 3 9]
    close all;
    ind=ind+1;
    load(['data/layout' num2str(layout)]);
    XX=X;YY=Y;TT=T;OXX=OX;OYY=OY;
    for i=1:size(XX,1)
        X=XX(i,:);Y=YY(i,:);T=TT(i,:);OX=OXX(i,:);OY=OYY(i,:);
        [uniq,m,n]=uunique([X' Y']);
        X=uniq(:,1)'; Y=uniq(:,2)'; T=T(m);
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %plot original graph
        if isplot(1)
            org=figure(); hold on; axis equal;
            plot(X,Y,'r','LineWidth',lineWidth);
            plot(OX,OY,'or','LineWidth',lineWidth);
            plot(X(end),Y(end),'+r','LineWidth',lineWidth);
            location='NE';
            if X(end)>0; location='NW'; end
            legend('Ground Truth','Obstacles','Goal','Location',location);
            if issave(1); saveas(org,['results/original/original_' num2str(layout) '_' num2str(i) '.png']); end
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%
        %%% FAJEN-WARREN %%%
        %%%%%%%%%%%%%%%%%%%%
        [b,kg,c1,c2,ko,c3,c4,V]=params();
        [t,path]=observer(X(end),Y(end),OX,OY,V,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
        x=path(:,3); y=path(:,4);
        % plot fajen warren
        if isplot(2)
            fj=figure(); hold on; axis equal;
            plot(X,Y,'r','LineWidth',lineWidth);
            plot(x,y,'-.','LineWidth',lineWidth);
            plot(OX,OY,'or','LineWidth',lineWidth);
            plot(X(end),Y(end),'+r','LineWidth',lineWidth);
            location='NE';
            if X(end)>0; location='NW'; end
            legend('Ground Truth','HSM','Obstacles','Goal','Location',location);
            if issave(2); saveas(fj,['results/fajen/fajen_' num2str(layout) '_' num2str(i) '.png']); end
        end
        
        ar(1,i)=aream(X,Y,x,y);
        ss(1,i)=sumsquare(X,Y,x,y,t);
        [H,P,KSSTAT]=kstestm(X,Y,T,x,y,t,alpha);
        h(1,i,:)=H;
        p(1,i,:)=P;
        ksstat(1,i,:)=KSSTAT;
        
        % ADDING OCCLUSION to FAJEN-WARREN
        if layout==9
            
            % find the initial point
            TI=find(T>occstart,1,'first');
            ti=find(t>occstart,1,'first');
            
            % calc initial position
            inX=interp1(T,X,occstart);
            inY=interp1(T,Y,occstart);
            inx=interp1(t,x,occstart);
            iny=interp1(t,y,occstart);
            count=0;
            
            %plot occlusion
            if isplotocc
                occ=figure(); hold on; axis equal;
                plot(X,Y,'r','LineWidth',lineWidth);
                % plot([X(1:TI-1) inX],[Y(1:TI-1) inY],'r','LineWidth',lineWidth);
                % plot([inX X(TI:end)],[inY Y(TI:end)],'k','LineWidth',lineWidth);
                plot([inx; x(ti:end)],[iny; y(ti:end)],'-.','LineWidth',lineWidth);
            end
            
            
            for time=occstart+interval:interval:occend
                count=count+1;
                
                % find last point at time
                TF=find(T<time,1,'last');
                tf=find(t<time,1,'last');
                
                % calc final exact position at time
                fX=interp1(T,X,time);
                fY=interp1(T,Y,time);
                fx=interp1(t,x,time);
                fy=interp1(t,y,time);
                
                % generate new x,y and t for calculation
                nX=[inX X(TI:TF) fX];
                nY=[inY Y(TI:TF) fY];
                nx=x(ti:tf);
                ny=y(ti:tf);
                nt=t(ti:tf);
                
                % set steps size so that it'll be comparable
                steps=100*(time-occstart)/(occend-occstart);
                trackss(1,count)=trackss(1,count) + sumsquare(X,Y,nx,ny,nt,0,steps)/size(XX,1);
                
                %area calculation
                trackar(1,count)=trackar(1,count) + aream(nX,nY,nx,ny,0,0)/size(XX,1);
            end
        end
        % OCCLUSION ENDS to FAJEN-WARREN
      
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%
        %%% SHORTCUT %%%
        %%%%%%%%%%%%%%%%
        if isplot(3)
            sh=figure(); hold on; axis equal;
            plot(X,Y,'r','LineWidth',lineWidth);
            plot([X(1) X(end)],[Y(1) Y(end)],':g','LineWidth',lineWidth);
            plot(OX,OY,'or','LineWidth',lineWidth);
            plot(X(end),Y(end),'+r','LineWidth',lineWidth);
            location='NE';
            if X(end)>0; location='NW'; end
            legend('Ground Truth','Shortcut','Obstacles','Goal','Location',location);
            if issave(3);  saveas(sh,['results/shortcut/shortcut_' num2str(layout) '_' num2str(i) '.png']); end
        end

        % error calculation
        [x y]=equitimepath(length(X),[X(1) X(end)],[Y(1) Y(end)],[0 5]);
        ar(2,i)=aream(X,Y,x,y);
        ss(2,i)=sumsquare(X,Y,x,y,T);
        [H,P,KSSTAT]=kstestm(X,Y,T,x,y,T,alpha);
        h(2,i,:)=H;
        p(2,i,:)=P;
        ksstat(2,i,:)=KSSTAT;
        
        
        % ADDING OCCLUSION to SHORTCUT
        if layout==9
            
            % find the initial point
            TI=find(T>occstart,1,'first');
            
            % calc initial position
            inX=interp1(T,X,occstart);
            inY=interp1(T,Y,occstart);
            count=0;
            
            %plot occlusion
            if isplotocc
                plot([inX X(end)],[inY Y(end)],'g:','LineWidth',lineWidth);
            end
            
            for time=occstart+interval:interval:occend
                count=count+1;
                
                % find last point at time
                TF=find(T<time,1,'last');
                
                % calc final exact position at time
                fX=interp1(T,X,time);
                fY=interp1(T,Y,time);
                fx=inX + (X(end)-inX)*(time-occstart)/(T(end)-occstart);
                fy=inY + (Y(end)-inY)*(time-occstart)/(T(end)-occstart);
                
                % generate new x,y and t for calculation
                nX=[inX X(TI:TF) fX];
                nY=[inY Y(TI:TF) fY];
                nx=[inX fx];
                ny=[inY fy];
                nt=[occstart time];
                
                % set steps size so that it'll be comparable
                steps=100*(time-occstart)/(occend-occstart);
                trackss(2,count)=trackss(2,count) + sumsquare(X,Y,nx,ny,nt,0,steps)/size(XX,1);
                
                %area calculation
                trackar(2,count)=trackar(2,count) + aream(nX,nY,nx,ny,0,0)/size(XX,1);
            end
        end
        % OCCLUSION ENDS to SHORTCUT
        

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%
        %%% VORONOI %%%
        %%%%%%%%%%%%%%%
        %%%%%%% VORONOI GENERATION
        if layout==1
            my=max(Y);
            mix=min(X);
            mx=max(X);
            axis([-my/2+OX-1 my/2+OX+1 -1 my+1],'square');
            oobx=[mix OX mx OX];
            ooby=[OY my OY 0];
            [vx vy]=voronoi([OX oobx],[OY ooby]);
            
            if i==1 || i==2 || i==10
                x=[vx(2,5) vx(1,5) vx(1,1) vx(1,2) vx(2,7)];
                y=[vy(2,5) vy(1,5) vy(1,1) vy(1,2) vy(2,7)];
            elseif i==3 || i==5 || i==6 || i==7
                x=[vx(2,8) vx(1,8) vx(2,3) vx(2,2) vx(2,6)];
                y=[vy(2,8) vy(1,8) vy(2,3) vy(2,2) vy(2,6)];
            elseif i==4
                x=[vx(2,5) vx(1,5) vx(1,1) vx(2,6)];
                y=[vy(2,5) vy(1,5) vy(1,1) vy(2,6)];
            elseif i==8 || i==9
                x=[vx(2,8) vx(1,8) vx(2,3) vx(2,7)];
                y=[vy(2,8) vy(1,8) vy(2,3) vy(2,7)];
            end
        elseif layout==3
            [vx,vy]=voronoi(OX,OY);

            if i==1
                x=vx(:,2);
                y=vy(:,2);
            elseif i==2
                x=vx(2:-1:1,3);
                y=vy(2:-1:1,3);
            elseif i==3
                x=vx(:,1);
                y=vy(:,1);
            elseif i==4
                x=[vx(2,3) vx(1,3) vx(2,1)];
                y=[vy(2,3) vy(1,3) vy(2,1)];
            elseif i==5
                x=[vx(2,2) vx(1,2) vx(2,3)];
                y=[vy(2,2) vy(1,2) vy(2,3)];
            elseif i==6
                x=[vx(2,3) vx(1,3) vx(2,1)];
                y=[vy(2,3) vy(1,3) vy(2,1)];
            elseif i==7
                x=[vx(2,1) vx(1,1) vx(2,3)];
                y=[vy(2,1) vy(1,1) vy(2,3)];
            elseif i==8
                x=[vx(2,3) vx(1,3) vx(2,1)];
                y=[vy(2,3) vy(1,3) vy(2,1)];
            elseif i==9
                x=[vx(2,1) vx(1,1) vx(2,2)];
                y=[vy(2,1) vy(1,1) vy(2,2)];
            elseif i==10
                x=[vx(2,3) vx(1,3) vx(2,1)];
                y=[vy(2,3) vy(1,3) vy(2,1)];
            end
            
            
        elseif layout==9
            [vx vy]=voronoi(OX,OY);
        
            if i==1 || i==2
                x=[vx(2,17) vx(1,17) vx(2,10) vx(2,7) vx(2,5) vx(2,4) vx(2,3) vx(2,2) vx(2,13)];
                y=[vy(2,17) vy(1,17) vy(2,10) vy(2,7) vy(2,5) vy(2,4) vy(2,3) vy(2,2) vy(2,13)];
            elseif i==3
                x=[vx(2,15) vx(1,15) vx(1,8) vx(2,16)];
                y=[vy(2,15) vy(1,15) vy(1,8) vy(2,16)];
            elseif i==4
                x=[vx(2,15) vx(1,15) vx(1,6) vx(1,4) vx(1,3) vx(2,12)];
                y=[vy(2,15) vy(1,15) vy(1,6) vy(1,4) vy(1,3) vy(2,12)];
            elseif i==5
                x=[vx(2,12) vx(1,12) vx(2,3) vx(2,4) vx(2,6) vx(2,15)];
                y=[vy(2,12) vy(1,12) vy(2,3) vy(2,4) vy(2,6) vy(2,15)];
            elseif i==6
                x=[vx(2,17) vx(1,17) vx(1,10) vx(2,5) vx(2,2) vx(2,3) vx(2,1) vx(2,14)];
                y=[vy(2,17) vy(1,17) vy(1,10) vy(2,5) vy(2,2) vy(2,3) vy(2,1) vy(2,14)];
            elseif i==7
                x=[vx(2,13) vx(1,13) vx(1,4) vx(1,5) vx(2,6) vx(1,8)];
                y=[vy(2,13) vy(1,13) vy(1,4) vy(1,5) vy(2,6) vy(1,8)];
            elseif i==8
                x=[vx(2,11) vx(1,11) vx(1,2) vx(1,3) vx(1,4) vx(1,5) vx(2,15)];
                y=[vy(2,11) vy(1,11) vy(1,2) vy(1,3) vy(1,4) vy(1,5) vy(2,15)];
            elseif i==9
                x=[vx(2,18) vx(1,18) vx(1,11) vx(1,6) vx(1,7) vx(1,8) vx(1,10) vx(2,4) vx(2,5)];
                y=[vy(2,18) vy(1,18) vy(1,11) vy(1,6) vy(1,7) vy(1,8) vy(1,10) vy(2,4) vy(2,5)];
            elseif i==10
                x=[vx(2,16) vx(1,16) vx(1,4) vx(1,9) vx(2,12) vx(2,11) vx(2,17)];
                y=[vy(2,16) vy(1,16) vy(1,4) vy(1,9) vy(2,12) vy(2,11) vy(2,17)];
            end
        
        end
        %%%%%%%% VORONOI GENERATION ENDS
        
        % prune start and end lines
        if y(1)<0
            x(1)=interp1(y(1:2),x(1:2),0);
            y(1)=0;
        end
        if y(end)>Y(end)
            x(end)=interp1(y(end-1:end),x(end-1:end),Y(end));
            y(end)=Y(end);
        end

        %plot voronoi diagram
        if isplot(4)
            vn=figure(); hold on; axis equal;
            plot(X,Y,'r','LineWidth',lineWidth);
            plot(x,y,'m--','LineWidth',lineWidth);
            plot(OX,OY,'or','LineWidth',lineWidth);
            plot(X(end),Y(end),'+r','LineWidth',lineWidth);
            location='NE';
            if X(end)>0; location='NW'; end
            legend('Ground Truth','Voronoi','Obstacles','Goal','Location',location);
            if issave(4);  saveas(vn,['results/voronoi/voronoi_' num2str(layout) '_' num2str(i) '.png']); end
        end


        % calculate time (length of patch) and scale to original time
        if length(x)>2
            t=zeros(1,length(x));
            for j=2:length(x)
                t(j) = t(j-1) + sqrt((x(j)-x(j-1))^2+(y(j)-y(j-1))^2);
            end
        else
            t=[0 5];
        end
        t=t*T(end)/t(end);
        
        %error calculation
        [nx,ny,nt]=equitimepath(length(T),x,y,t);
        ar(3,i)=aream(X,Y,x,y); %before equitime, because equitime changes the points
        ss(3,i)=sumsquare(X,Y,x,y,t);
        [H,P,KSSTAT]=kstestm(X,Y,T,nx,ny,nt,alpha);
        h(3,i,:)=H;
        p(3,i,:)=P;
        ksstat(3,i,:)=KSSTAT;
        
        
        % ADDING OCCLUSION to VORONOI
        if layout==9
            % find initial position
            ti=find(t>occstart,1,'first');
            % calc initial position
            inX=interp1(T,X,occstart);
            inY=interp1(T,Y,occstart);
            inx=interp1(y,x,inY);
            iny=inY;
            count=0;
            
            %plot occlusion
            if isplotocc
                plot([inx x(ti:end)],[iny y(ti:end)],'m--','LineWidth',lineWidth);
            end
            
            for time=occstart+interval:interval:occend
                count=count+1;
                
                % find last point at time
                TF=find(T<time,1,'last');
                
                % calc final exact position at time
                fX=interp1(T,X,time);
                fY=interp1(T,Y,time);
                fx=interp1(y,x,fY);
                fy=fY;
                tf=find(y<fY,1,'last');
                
                % generate new x,y and t for calculation
                nX=[inX X(TI:TF) fX];
                nY=[inY Y(TI:TF) fY];
                nx=[inx x(ti:tf) fx];% if isempty(x(ti:tf)); nx(end+1)=fx; end
                ny=[iny y(ti:tf) fy];% if isempty(y(ti:tf)); ny(end+1)=fy; end
                nt=[occstart t(ti:tf) time];% if isempty(t(ti:tf)); nt(end+1)=time; end
                
                % set steps size so that it'll be comparable
                steps=100*(time-occstart)/(occend-occstart);
                trackss(3,count)=trackss(3,count) + sumsquare(X,Y,nx,ny,nt,0,steps)/size(XX,1);
                
                %area calculation
                trackar(3,count)=trackar(3,count) + aream(nX,nY,nx,ny,0,0)/size(XX,1);
            end
        end
        % OCCLUSION ENDS to VORONOI

    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        %%% CONSTANT VELOCITY %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % ADDING OCCLUSION to CONSTANT VELOCITY
        if layout==9

            % find the initial point
            TI=find(T>occstart,1,'first');
            ti=interp1(T,1:length(T),occstart);

            % calc initial position
            inX=interp1(T,X,occstart);
            inY=interp1(T,Y,occstart);
            dX=X(TI)-X(TI-1);
            dY=Y(TI)-Y(TI-1);
            count=0;
            
            %plot occlusion
            if isplotocc
                plot(inX+dX*(0:length(T)-TI),inY+dY*(0:length(T)-TI),'c:.','LineWidth',lineWidth);
                plot(OX,OY,'or','LineWidth',lineWidth);
                plot(X(end),Y(end),'+r','LineWidth',lineWidth);
                location='SW';
                legend('Ground Truth','HSM','Shortcut','Voronoi',...
                    'Constant Velocity','Obstacles','Goal','Location',location);
                if issaveocc; saveas(occ,['results/together/occlusion_' num2str(layout) '_' num2str(i) '.png']); end
            end
            

            for time=occstart+interval:interval:occend
                count=count+1;

                % find last point at time
                TF=find(T<time,1,'last');
                tf=interp1(T,1:length(T),time);

                % calc final exact position at time
                fX=interp1(T,X,time);
                fY=interp1(T,Y,time);
                fx=inX+dX*(tf-ti);
                fy=inY+dY*(tf-ti);

                % generate new x,y and t for calculation
                nX=[inX X(TI:TF) fX];
                nY=[inY Y(TI:TF) fY];
                nx=[inX fx];
                ny=[inY fy];
                nt=[occstart time];

                % set steps size so that it'll be comparable
                steps=100*(time-occstart)/(occend-occstart);
                trackss(4,count)=trackss(4,count) + sumsquare(X,Y,nx,ny,nt,0,steps)/size(XX,1);

                %area calculation
                trackar(4,count)=trackar(4,count) + aream(nX,nY,nx,ny,0,0)/size(XX,1);
            end
        end
        % OCCLUSION ENDS to CONSTANT VELOCITY
        
    end
    
    for i=1:3
        are(i,ind)=mean(ar(i,:));
        ssd(i,ind)=mean(ss(i,:));
        ksx(i,ind)=sum(h(i,:,1)==0);
        ksy(i,ind)=sum(h(i,:,2)==0);
        ks((i-1)*2+1,ind)=ksx(i,ind);
        ks(2*i,ind)=ksy(i,ind);
    end
    
end





%%%%%%%%%%%%%% REPORTING %%%%%%%%%
%z=[ss' ar'];
info.cnames=strvcat('1','3','9'); %#ok<VCAT>
info.rnames=strvcat('Layout','Fajen-Warren','Shortcut','Voronoi'); %#ok<VCAT>
info.caption='Error in area comparison with different methods and obstacle layout.';
info.label='tab:errorAREA';
table2latex(are,info);

info.caption='Error in sum squared distance comparison with different methods and obstacle layout.';
info.label='tab:errorSSD';
table2latex(ssd,info);

info.rnames=strvcat('Layout','\multirow{2}{*}{Fajen-Warren}& x','& y ',...
    '\midrule \multirow{2}{*}{Shortcut}& x','& y ','\midrule \multirow{2}{*}{Voronoi}& x','& y '); %#ok<VCAT>
info.caption='Number of matched paths on x axis after KS Test';
info.label='tab:errorKS';
info.fmt='%3d';
table2latex(ks,info);

info.cnames=strvcat('0.5','1.0','1.5','2.0','2.5','3.0','3.5'); %#ok<VCAT>
info.rnames=strvcat('Time','Fajen-Warren','Shortcut','Voronoi','Constant Velocity'...
            ,'\midrule Fajen-Warren','Shortcut','Voronoi','Constant Velocity'); %#ok<VCAT>
info.caption=['Average error of 10 sample paths having occlusion after $1^{st}$ '...
    'second. Top error set is sum squared distance error, bottom set is area error.'];
info.label='tab:track';
info.fmt='%25.4f';
table2latex([trackss;trackar],info);

