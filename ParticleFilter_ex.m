function [pvector,output] = ParticleFilter_ex(pvector, U, O, Z, varargin)
% PARTICLE FILTER
% Calculates particle filter with given
% control control unit U in format:
%   U = [muX     muY;
%        sigmaX  sigmaY];
% observation matrix Z in format:
%   Z= [obsX  obsY];
% varargin: NumParticles, isLowVarianceSampler
% returns 1) X_t and 2) weighted mean of particles

NumParticles=1500;  if nargin>3; NumParticles=varargin{1}; end
lvs=0;              if nargin>4; lvs=varargin{1}; end
weights=zeros(NumParticles,1);
sigma=.51;%.38;

% p(x_t|x_{t-1},z_{t:1},u(t));  No movement, so x_t=x_{t-1}.  But,
% use this to introduce some variance in the particles.  This is
% the SAMPLE step in Table 4.3 in Probabilistic Robotics 
%perturbation on resampling for particles
if isempty(U)
    pvector=pvector + sigma.*randn(NumParticles,2);
else
    for i=1:NumParticles
        [q1,q2]=closestpoint(U(:,1:2),pvector(i,:));
%         theta=mod(atan2((q2(2)-q1(2)),(q2(1)-q1(1))),pi); %buggy
        theta=atan2((q2(2)-q1(2)),(q2(1)-q1(1)));
        l=sqrt((q2(2)-q1(2))^2+(q2(1)-q1(1))^2);   %length
        sigmad=l;
        sigman=sigmad/4;
        rot=[cos(theta) -sin(theta);sin(theta) cos(theta)]*[l+sigmad*randn;0];
        
        d=(q1+q2)/2-pvector(i,:);
        
        vect=O-repmat(pvector(i,:),length(O),1);
        distn=hypot(vect(:,1),vect(:,2));
        effect=zeros(length(O),1);
%         effect = .1./distn;
        for o=1:length(O)
%             alpha=mod(atan2(vect(o,2),vect(o,1))+2*pi, 2*pi); %buggy
            alpha=atan2(vect(o,2),vect(o,1));
%             disp(['the ' num2str(180/pi*theta) ' - alp ' num2str(alpha*180/pi) ' - diff ' num2str(180/pi*mod((theta-alpha)+2*pi, 2*pi))]);
%             plot(pvector(i,1),pvector(i,2),'r+');
            if mod((theta-alpha)+2*pi, 2*pi) < pi
%                 disp(['apply left for obst ' int2str(o)])
                effect(o)=1/exp(distn(o));
                vect(o,:)=[cos(theta+pi/2) -sin(theta+pi/2);sin(theta+pi/2) cos(theta+pi/2)] * [effect(o);0];
            else
%                 disp(['apply right for obst ' int2str(o)])
                effect(o)=1/exp(distn(o));
                vect(o,:)=[cos(theta-pi/2) -sin(theta-pi/2);sin(theta-pi/2) cos(theta-pi/2)] * [effect(o);0];
            end
            
        end
        rebound=sum(vect);
%         waitforbuttonpress;
%         for o=1:length(O)
%             rebound=[cos(theta) -sin(theta);sin(theta) cos(theta)]*[0;distn(o)];
%         end
%         less=5;
%         distn=O-repmat(pvector(i,:),length(O),1);
%         distn=hypot(distn(:,1),distn(:,2));
%         emph=1/exp(mean(distn(distn<less)));
%         d=d*emph;
%         distn(mi)=Inf;
%         [m2,mi2]=min(distn);
%         a=(O(mi,:)+O(mi2,:))/2-pvector(i,:);
        
        noise=d.*sigman+sigman.*randn(1,2);
        
        pvector(i,1)=pvector(i,1) + rot(1) + rebound(1) + noise(1);%*cos(theta);% + sigman*randn;
        pvector(i,2)=pvector(i,2) + rot(2) + rebound(2) + noise(2);%*sin(theta);% + sigman*randn;
    end
end
% Compute the IMPORTANCE FACTOR
% p(z_t|x_t) is the likelihood.
%calculating weights
if isempty(Z)
    weights(:) = 1; %Brownian Motion due to no observation
else
    for i=1:NumParticles    %use observation
        dist = (Z(1)-pvector(i,1))^2 + (Z(2)-pvector(i,2))^2;
        weights(i) = 1/dist;
    end
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

pvector = svector;

%particles weighted by the probability
output(1)=sum(pvector(:,1).*(weights/sum(weights)));
output(2)=sum(pvector(:,2).*(weights/sum(weights)));