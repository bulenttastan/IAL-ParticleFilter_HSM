function val = SSEfajen_goal(k)

global ym; global tm; global v;
b=k(1);
kg=k(2);
c1=k(3);
c2=k(4);
val=0;

for i=1:length(ym)
    [t,y] = observer(ym(i).y(end,3),ym(i).y(end,4),[],[],v(i),ym(i).y(1,:),b,kg,c1,c2,0,0,0,tm(i).t);

    resid=y(:,3:4)-ym(i).y(:,3:4);%y-ym(i).y;%
    val=val+sum(sum(resid.^2));
end