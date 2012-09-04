function val = SSEfajen_obst(k)

global ym; global tm; global v; global OX; global OY;
global b; global kg; global c1; global c2;
ko=k(1);
c3=k(2);
c4=k(3);
val=0;

for i=1:length(ym)
    [t,y] = observer(ym(i).y(end,3),ym(i).y(end,4),OX(i,:),OY(i,:),v(i),ym(i).y(1,:),b,kg,c1,c2,ko,c3,c4,tm(i).t);

    resid=y(:,3:4)-ym(i).y(:,3:4);%y-ym(i).y;%
    val=val+sum(sum(resid.^2));
end