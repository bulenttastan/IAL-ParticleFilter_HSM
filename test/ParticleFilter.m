%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate a UAV flying over a search %
% area looking for a downed aircraft. %
% The UAV should use radio beacon     %
% finding give that the UAV has a     %
% directional antennae.  This should  %
% be written to support Brad Huber's  %
% thesis work.                        %
%                                     %
% Michael A. Goodrich                 %
% May 26, 2006.                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%
	% INITIALIZATION %
    %%%%%%%%%%%%%%%%%%
    
    %PLACE BEACON IN THE WORLD
	beacon=500*rand(1,2)-250; % Returns the x-y coordinates of a downed aircraft
                           % centered at zero and with a standard deviation
                           % of 100.
    
   %mov = avifile('FindAircraft.avi');  % Uncomment to create movie of
                                        % UAV finding the radio beacon. 
    
    % INTIALIZE PARTICLE FILTER PRIOR BELIEF
    SweepAngleWidth = 15;  % Plus or minus angle to detect if beacon and particle are within the angle window.
    NumParticles=1500;  % The number of particles that I will track.
	pvector=700*rand(NumParticles,2)-350; % Randomly placed particles in the world.  
                                       % Place Normally in 2-D according to the problem specs.
                                       % This represents the initial belief
                                       % of the location of the beacon.

                                       % Plot and assign handles for animation.
	
    % CREATE GRAPHICS HANDLES TO TRACK WHERE MY BELIEFS LIE
    %ParticleHandle=zeros(NumParticles,1);
	figure(1);clf;
	ParticleHandle=plot(pvector(:,1),pvector(:,2),'b.');

    % CREATE REPRESENTATION OF THE UAV
    HalfUAVWidth=40; HalfUAVHeight=10;UAVSweep=2;
	UAVFoot=[0,-HalfUAVHeight,-3*HalfUAVHeight,-2*HalfUAVHeight,-3*HalfUAVHeight,-HalfUAVHeight;...
            0,HalfUAVWidth,HalfUAVWidth,0,-HalfUAVWidth,-HalfUAVWidth;...
            1,1,1,1,1,1];  % Use homogeneous coordinates
    UAVHandle=patch(UAVFoot(1,:),UAVFoot(2,:),'g');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FLY THE UAV IN A SPIRAL AROUND A LARGE CIRCLE. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	
	% EQUATIONS OF MOTION
    %
    % Hopf bifurcation equations -- desired location model
    % (x,y) = target UAV position -- is this the center of the orbit?
    % (hatx,haty) = actual/estimated UAV position
    % xc = hatx - x; difference between estimated and actual position
    % yc = haty - y; ditto
    % dx/dt = yc + xc/(alpha * r^2) (r^2-x^2-y^2)
    % dy/dt = xc + yc/(alpha * r^2) (r^2-x^2-y^2) 
    % psi = atan2(dy,dx);  Desired direction of the UAV
    % r = desired orbital radius of UAV
    % alpha = agressiveness for getting back on trajectory
    %
    % Actual UAV evolution equations via Euler integration
    % hatx = hatx + T cos(psi)  + v(1); v is process noise
    % haty = haty + T sin(psi) + v(2); v is process noise
    
    % SIMULATION PARAMETERS
	Duration=100;        % Number of time ticks in the simulation. was 5000
    vstd=2;             % Standard deviation of process noise
    r = 30;             % Radius of UAV orbit
    RSpiral = 300;      % Radius of spiral orbit
    alpha = 1;          % Kluge parameter.  I don't know the range of this parameter
    %ThetaSpiral = 0;   % Position on the spiral orbit.
    dT = 1;             % Time step of simulation.
    phi = pi;           % Initial Angle of the UAV orbit
    V = 20;             % Velocity of the UAV
    theta = phi;        % The initial UAV direction equals the initial angle of the orbit.
    UAVhist = zeros(Duration,2);
    
    
    % CHOOSE INITIAL LOCATION AND ORIENTATION OF THE UAV
    UAVpos=[RSpiral+r,0];
    UAVhist(1,:) = UAVpos;

    % Target center of UAV orbit.  For now, don't update target orbit.
    ThetaSpiral = atan2(UAVpos(2),UAVpos(1));  % Angle of UAV from center of spira orbit.
    xp=RSpiral*cos(ThetaSpiral);
    yp=RSpiral*sin(ThetaSpiral);

    % Initial location of the UAV orbit is the UAV's original position
    T = [1,0,UAVhist(1,1);0,1,UAVhist(1,2);0,0,1]; % Camera translation matrix in homogeneous coordinates.
    R = [cos(phi), -sin(phi), 0 ; sin(phi), cos(phi), 0; 0,0,1];  % Camera rotation matrix in hcoordinates
    PlotUAVFoot = T*R*UAVFoot;
    set(UAVHandle,'Xdata',PlotUAVFoot(1,:),'Ydata',PlotUAVFoot(2,:));
    
    ShadowFoot = [0,-600*cos(phi+SweepAngleWidth/180*pi),-600*cos(phi-SweepAngleWidth/180*pi),...
        0,600*cos(phi-SweepAngleWidth/180*pi),600*cos(phi+SweepAngleWidth/180*pi);...
        0,-600*sin(phi+SweepAngleWidth/180*pi),-600*sin(phi-SweepAngleWidth/180*pi),...
        0,600*sin(phi+SweepAngleWidth/180*pi),600*sin(phi-SweepAngleWidth/180*pi);...
        1,1,1,1,1,1];
    ShadowHandle = patch(ShadowFoot(1,:),ShadowFoot(2,:),[.9,.9,.9]);
    set(ShadowHandle,'EraseMode','xor');
    axis([-500,500,-500,500]);

    for t=2:Duration
        
        % SIMULATION
        % Target center of UAV orbit.  
        ThetaSpiral = ThetaSpiral + (V/pi)*dT/RSpiral;  % Move one diameter of orbit radius on spiral each time I finish an orbit.
        xp=RSpiral * cos(ThetaSpiral);
        yp=RSpiral * sin(ThetaSpiral);

        % Update target location.
        phi = phi + V*dT/r; % Complete one orbit. 
        xorbit = r*cos(phi);
        yorbit = r*sin(phi);
        x = xp + xorbit;
        y = yp + yorbit;
        
        % Derive direction of travel for the UAV
        hatx=UAVpos(1);haty=UAVpos(2);
        xc = hatx-x; yc = haty-y;
        tmp = r^2 - xc^2 - yc^2;
        dx = yc + xc / (alpha * r^2) * tmp;
        dy = xc + yc / (alpha * r^2) * tmp;  % Should this be -xc or +xc??
        psi = atan2(dy,dx);
        % Saturate out turn angle.

        
        % Update UAV position.
        UAVpos(1,1) = UAVpos(1,1) + dT*V*cos(psi) + vstd*randn(1,1);
        UAVpos(1,2) = UAVpos(1,2) + dT*V*sin(psi) + vstd*randn(1,1);


        UAVhist(t,1) = UAVpos(1);
		UAVhist(t,2) = UAVpos(2);
        % Animate the center of the UAV
        hold on;
		plot(UAVhist(t-1:t,1),UAVhist(t-1:t,2),'r-');
        h=plot(beacon(1),beacon(2),'kx');
        set(h,'markersize',10,'linewidth',3);
		hold off;
        % Animate the UAV footprint
        T = [1,0,UAVhist(t,1);0,1,UAVhist(t,2);0,0,1]; % Camera translation matrix in homogeneous coordinates.
        theta = psi;  % orientation of the UAV.  Assumes that the UAV can match orientation very quickly
        %theta = atan2(UAVhist(t,2)-UAVhist(t-1,2),UAVhist(t,1)-UAVhist(t-1,1));
        R = [cos(theta), -sin(theta), 0 ; sin(theta), cos(theta), 0; 0,0,1];  % Camera rotation matrix in hcoordinates
        PlotUAVFoot = T*R*UAVFoot;
        set(UAVHandle,'Xdata',PlotUAVFoot(1,:),'Ydata',PlotUAVFoot(2,:));
        PlotShadowFoot = T*R*ShadowFoot;
        set(ShadowHandle,'Xdata',PlotShadowFoot(1,:),'Ydata',PlotShadowFoot(2,:));
      
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % IMPLEMENT PARTICLE FILTER %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Detect Hits
        rel_angle = atan2(UAVpos(1,2)-beacon(1,2),UAVpos(1,1)-beacon(1,1));
        if ((abs(rel_angle-theta)) < SweepAngleWidth/180*pi) || (abs(rel_angle-(theta+pi))<SweepAngleWidth/180*pi) || (abs((pi+rel_angle)-theta)<SweepAngleWidth/180*pi)
            xlabel('Hit');
            z = true;
        else
            xlabel('No Hit');
            z = false;
        end;
        %pause(1);
        
        % p(x_t|x_{t-1},z_{t:1},u(t));  No movement, so x_t=x_{t-1}.  But,
        % use this to introduce some variance in the particles.  This is
        % the SAMPLE step in Table 4.3 in Probabilistic Robotics 
        pvector=pvector+2*randn(NumParticles,2);  % Magic number -- perturbation on resampling
        % Compute the IMPORTANCE FACTOR
        % p(z_t|x_t) is the likelihood.
        weights=zeros(NumParticles,1);
        for i=1:NumParticles
            rel_angle = atan2(UAVpos(1,2)-pvector(i,2),UAVpos(1,1)-pvector(i,1));
            if ((abs(rel_angle-theta)) < SweepAngleWidth/180*pi) || (abs(rel_angle-(theta+pi))<SweepAngleWidth/180*pi) || (abs((pi+rel_angle)-theta)<SweepAngleWidth/180*pi)  % If within +- SweepAngleWidth degrees
                if (z==true)
                    weights(i) = .8;  % Correct detection rate
                else % z==false
                    weights(i) = .2;  % Missed detection rate
                end
            else % not within SweepAngleWidth degrees
                if (z==true)
                    weights(i) = .3; % False alarm rate
                else %z==false
                    weights(i) = .7; % Correct miss
                end;
            end; % End if hit or miss
         end; % End for each particle
         %Introduce some errors into the detections to see what happens.
         if (z==false) % Change a correctly diagnosed miss into a false alarm.
            if (rand(1,1)>.5); z=true;
            end;
         else %if z==true % Change a correctly diagnosed hit into a missed detection
            if (rand(1,1)>.8); z=false;
            end
         end;
         % Probability faulty observations seems to affect how many loops
         % have to be done before the target will be suitably localized.
            
        % Resample
        [s,si] = sort(weights);
        cumdf = cumsum(weights(si)/sum(weights));
        svector = zeros(NumParticles,2);
        for i=1:NumParticles
            gsample = rand(1,1);
            [tmp,tmpi]=min(abs(cumdf-gsample));
            svector(i,:) = pvector(si(tmpi),:);
        end;
        pvector = svector;
        set(ParticleHandle,'Xdata',pvector(:,1),'YData',pvector(:,2));
        
        
        drawnow;
        
        
    end;
    break
    
    

    %mov = close(mov);
