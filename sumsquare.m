function ss = sumsquare( x1,y1,x2,y2,t,varargin )
%calculates the sum squared error
%between lines(vectors) [x1 y1] and [x2 y2] interpolating on t of curve 2
%6th optinal boolean input indicates to plot or not
%7th optional integer defines step size [default: 100]

%delete below (written to run seperately)
% clear; clc; close all;
% % axes('position',[0,0,1,1]);
% hold on;
% load data/goalonly;
% x1=X(1,:);
% y1=Y(1,:);
% [t,y]=observer(x1(40),y1(40),[],[],3.1119437730616886);
% x2=y(:,3);
% y2=y(:,4);
%delete above
steps=100;
if nargin>6; steps=varargin{2}; end;
interval=t(numel(t))/steps;
ind=1;
P=[0;0];
ss=0;

for i=1:steps
    %calculate equitime
    T=i*interval+t(1);
    if i==steps; T=t(end); end;
    
    %interpolate on time frame to position
    ind=findfirst(t,T,ind);
    P(1) = interp1([t(ind-1) t(ind)],[x2(ind-1) x2(ind)],T);
    P(2) = interp1([t(ind-1) t(ind)],[y2(ind-1) y2(ind)],T);
    
    %find the closest positions to org curve
    [mins,pts]=twomin((x1-P(1)).^2+(y1-P(2)).^2);
    Q1=[x1(pts(1));y1(pts(1))];
    Q2=[x1(pts(2));y1(pts(2))];
    
    %calculate shortest dist
    d = abs(det([Q2-Q1,P-Q1]))/norm(Q2-Q1);
    if ~isnan(d); ss=ss+d; end
    
    %plot lines y=mx+n
    if nargin>5 && varargin{1}
        m1=(Q2(2)-Q1(2))/(Q2(1)-Q1(1));
        n1=Q1(2)-m1*Q1(1);
        m2=-1/m1;
        n2=P(2)-m2*P(1);
        P2(1)=(n2-n1)/(m1-m2);
        P2(2)=m1*P2(1)+n1;
        plot([P(1) P2(1)],[P(2) P2(2)],'y');
    end
end


% for i=1:numel(x2)
%     [mins,pts]=twomin((x1-x2(i)).^2+(y1-y2(i)).^2);
%     Q1=[x1(pts(1));y1(pts(1))];
%     Q2=[x1(pts(2));y1(pts(2))];
%     P=[x2(i);y2(i)];
%     
%     d = abs(det([Q2-Q1,P-Q1]))/norm(Q2-Q1);
%     if ~isnan(d); ss=ss+d; end
%     
%     %plot lines y=mx+n
%     if nargin==5 && varargin{1}
%         m1=(Q2(2)-Q1(2))/(Q2(1)-Q1(1));
%         n1=Q1(2)-m1*Q1(1);
%         m2=-1/m1;
%         n2=P(2)-m2*P(1);
%         P2(1)=(n2-n1)/(m1-m2);
%         P2(2)=m1*P2(1)+n1;
%         plot([P(1) P2(1)],[P(2) P2(2)],'y');
%     end
% end
%if i==steps && varargin{1}; disp(pts); plot(x1(pts(1)),y1(pts(1)),'+g');
%plot(x1(pts(2)),y1(pts(2)),'+g'); end;      %check
%     doit=0;
%     if pts(1) < pts(2)
%         a=Q2-Q1;
%         b=P-Q1;
%     else
%         a=P-Q1;
%         b=Q2-Q1;
%     end
%     a=[a(1) a(2) 0];
%     b=[b(1) b(2) 0];
%     angle = atan2(norm(cross(a,b)),dot(a,b));
%     %angle = mod(atan2(a(1)*b(2)-b(1)*a(2),a(1)*b(1)+a(2)*b(2)),2*pi);
%     disp(num2str(angle/pi*180));
%     if angle>pi/2
%         plot(x1(pts(1)),y1(pts(1)),'+k');
%         plot(x1(pts(2)),y1(pts(2)),'+k');
%         doit=1;
%     end
    