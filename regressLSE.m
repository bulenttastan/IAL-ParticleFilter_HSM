clear;
clc;
load data/samedistance; A=X;B=Y;
load data/berkansameangle;
sqrerror = 10000; area1=0; area2=0;
fb=0; fkg=0; fc1=0; fc2=0;

tic;
for b=4:.05:4
    for kg=1:.5:10
        for c1=0:.05:1
            for c2=0:.05:1
                broke=0;
                error1=0; error2=0;
                for i=1:size(A,1);
                    [t,y]=observer(A(i,40),B(i,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
                    for j=1:numel(y(:,3))
                        error1 = error1 + min((A(i,:)-y(j,3)).^2+(B(i,:)-y(j,4)).^2);
                    end
                    dist=(y(end,3)-A(i,40))^2+(y(end,4)-B(i,40))^2;
                    if error1>=sqrerror || dist>4
                        broke=1;
                        break;
                    end
                end
                if broke==1
                    continue;
                end
                for i=1:size(X,1);
                    [t,y]=observer(X(i,40),Y(i,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
                    for j=1:numel(y(:,3))
                        error2 = error2 + min((X(i,:)-y(j,3)).^2+(Y(i,:)-y(j,4)).^2);
                    end
                    dist=(y(end,3)-X(i,40))^2+(y(end,4)-Y(i,40))^2;
                    if error2>=sqrerror || dist>4
                        broke=1;
                        break;
                    end
                end
                if broke==1
                    continue;
                end

                sqrerror = error1+error2;
                fb=b; fkg=kg; fc1=c1; fc2=c2;
            end
        end
    end
end
mytime=toc;

disp(['It took ' num2str(mytime) ' seconds']);
disp('-- Final Results --');
disp(['b:    ' num2str(fb)]);
disp(['kg:   ' num2str(fkg)]);
disp(['c1:   ' num2str(fc1)]);
disp(['c2:   ' num2str(fc2)]);
disp(['squarederror: ' num2str(sqrerror)]);
% report(['b: ' num2str(fb) ' , kg: ' num2str(fkg) ' , c1: ' num2str(fc1) ' , c2: ' num2str(fc2) ' , squarederror: ' num2str(sqrerror) ' , it took: ' num2str(mytime)]);