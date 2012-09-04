clc;
load data/withobstacle
sqrerror = 10000;
fko=0; fc3=0; fc4=0;
b=4.05; kg=10; c1=0.3; c2=1;

tic;
for ko=50:50
    for c3=0:.5:10
        for c4=0:.05:1
            broke=0;
            error1=0;
            for i=1:size(X,1)
                [t,y]=observer(X(i,40),Y(i,40),OX(i),OY(i),3.1119437730616886,[0 0 0 0],b,kg,c1,c2,ko,c3,c4);
                for j=1:numel(y(:,3))
                    error1 = error1 + min((X(i,:)-y(j,3)).^2+(Y(i,:)-y(j,4)).^2);
%                     hold on;
%                     [xxx,pt]=min((X(i,:)-y(j,3)).^2+(Y(i,:)-y(j,4)).^2);
%                     plot([X(i,pt) y(j,3)],[Y(i,pt) y(j,4)],'g');
                end
                dist=(y(end,3)-X(i,40))^2+(y(end,4)-Y(i,40))^2;
                if error1>=sqrerror || dist>4
                    broke=1;
                    break;
                end
            end
            if broke==1
                continue;
            end
            
            sqrerror = error1;
            fko=ko; fc3=c3; fc4=c4;
        end
    end
end
mytime=toc;

disp(['It took ' num2str(mytime) ' seconds']);
disp('-- Final Results --');
disp(['ko:   ' num2str(fko)]);
disp(['c3:   ' num2str(fc3)]);
disp(['c4:   ' num2str(fc4)]);
disp(['squarederror: ' num2str(sqrerror)]);
% report(['ko: ' num2str(fko) ' , c3: ' num2str(fc3) ' , c4: ' num2str(fc4) ' , area: ' num2str(area) ' , it took: ' num2str(mytime)]);