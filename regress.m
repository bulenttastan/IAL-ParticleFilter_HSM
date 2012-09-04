clear;
clc;
load data/samedistance; A=X;B=Y;
load data/berkansameangle;
area = 1000; area1=0; area2=0;
fb=0; fkg=0; fc1=0; fc2=0;

tic;
for b=0:.05:0
    for kg=1:.1:1
        for c1=0:.05:1
            for c2=0:.05:1
                broke=0;
                area1=0; area2=0;
                for i=1:size(A,1);
                    [t,y]=observer(A(i,40),B(i,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
                    area1 = area1+polyarea([A(i,:) fliplr(y(:,3)')],[B(i,:) fliplr(y(:,4)')]);
                    if area1>area
                        broke=1;
                        break;
                    end
                end
                if broke==1
                    continue;
                end
                for i=1:size(X,1);
                    [t,y]=observer(X(i,40),Y(i,40),[],[],3.1119437730616886,[0 0 0 0],b,kg,c1,c2);
                    area2 = area2+polyarea([X(i,:) fliplr(y(:,3)')],[Y(i,:) fliplr(y(:,4)')]);
                    if (area1+area2)>area
                        broke=1;
                        break;
                    end
                end
                if broke==1
                    continue;
                end
                
                if (area1+area2) < area
                    area = area1+area2;
                    fb=b; fkg=kg; fc1=c1; fc2=c2;
                end
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
disp(['area: ' num2str(area)]);
report(['b: ' num2str(fb) ' , kg: ' num2str(fkg) ' , c1: ' num2str(fc1) ' , c2: ' num2str(fc2) ' , area: ' num2str(area) ' , it took: ' num2str(mytime)]);


%                     plot(y(:,3),y(:,4));
%                     plot(X(i,:),Y(i,:),'r');
%                     x=fill([X(i,:) fliplr(y(:,3)')],[Y(i,:)
%                     fliplr(y(:,4)')],'c');
%                     area(y(:,3),y(:,4));
%                     area(X(i,:),Y(i,:));