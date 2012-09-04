function output = ParticleFilter( Z,varargin )
% Calculates particle filter with given
% observation matrix Z in format:
% ie Z= [0 0;
%        1 2];
% varargin: NumParticles, isLowVarianceSampler, isShowOutput
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMPLEMENT PARTICLE FILTER %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NumParticles=1500;
if nargin>1; NumParticles=varargin{1}; end
lvs=0;
if nargin>2; lvs=varargin{2}; end
%dist=0; for i=1:length(Z)-1; dist = dist+sqrt((Z(i,1)-Z(i+1,1))^2+(Z(i,2)-Z(i+1,2))^2); end
dist = sum(sqrt(diff(Z(:,1)).^2+diff(Z(:,2)).^2));
pert=dist/(length(Z)-1); %pert=.4;
write2file=0;
pvector=zeros(NumParticles,2);  %pvector=10*rand(NumParticles,2)-5;
weights=zeros(NumParticles,1);
ParticleHandle=plot(pvector(:,1),pvector(:,2),'.');
if write2file; mov = avifile('chase8.avi','compression','Cinepak'); end
output = zeros(length(Z),2);

for dur=1:length(Z)
    % p(x_t|x_{t-1},z_{t:1},u(t));  No movement, so x_t=x_{t-1}.  But,
    % use this to introduce some variance in the particles.  This is
    % the SAMPLE step in Table 4.3 in Probabilistic Robotics 
    %perturbation on resampling for particles
    pvector=pvector + pert*randn(NumParticles,2);
    
    % Compute the IMPORTANCE FACTOR
    % p(z_t|x_t) is the likelihood.
    %calculating weights
    for i=1:NumParticles
        dist = (Z(dur,1)-pvector(i,1))^2 + (Z(dur,2)-pvector(i,2))^2;
        if 0%dur>15 && dur<30 %brownian motion effect
            dist = 1;
        end
        weights(i) = 1/dist;
        %disp(1/dist);
    end
    
    %resampling
    [s,si] = sort(weights);
    cumdf = cumsum(weights(si)/sum(weights));
    svector = zeros(NumParticles,2);
    for i=1:NumParticles
        gsample = rand(1,1);
        [tmp,tmpi]=min(abs(cumdf-gsample));
        svector(i,:) = pvector(si(tmpi),:);
    end;
    
    %%%%%% Low Variance Sampler
    if lvs
        weights = weights/sum(weights);
        r=rand(1,1)/NumParticles;
        c=weights(1);
        i=1;
        for m=1:NumParticles
            u=r+(m-1)/NumParticles;
            while u>c
                i=i+1;
                c=c+weights(i);
            end
            svector(m,:) = pvector(i,:);
        end;
    end
    %location of the particle with the highest probability weight
    %output(dur,:)=pvector(si(NumParticles),:);
    
    pvector = svector;
    set(ParticleHandle,'XData',pvector(:,1),'YData',pvector(:,2));

    %particles weighted by the probability
    output(dur,1)=sum(pvector(:,1).*(weights/sum(weights)));
    output(dur,2)=sum(pvector(:,2).*(weights/sum(weights)));
    if nargin>3 && varargin{3}; plot(output(dur,1),output(dur,2),'.r'); end
    drawnow;
    
    %writing to movie file
    if write2file; F = getframe(gca);mov = addframe(mov,F);mov = addframe(mov,F);mov = addframe(mov,F);end

    pause(.1);
end

if write2file; mov = close(mov); end %#ok<NASGU>