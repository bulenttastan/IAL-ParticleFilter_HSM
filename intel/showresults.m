close all; clear; clc;
%load results into variables ssd, length
% load '01-21-08.15.59.54.mat';
load result1;
ssd = ss;
length = pathlength;
% load '01-22-08.10.12.48.mat';
load result2;
ssd = [ssd ss];
length = [length pathlength];
% load '01-23-08.14.50.06.mat';
load result3;
ssd = [ssd ss];
length = [length pathlength];
clear ss pathlength;

scale = repmat(mean(length)./length, 4, 1);
ssd = ssd.*scale;

disp(['Average SSD for HSM: ' num2str(mean(ssd(1,:)))]);
disp(['Average SSD for VOR: ' num2str(mean(ssd(2,:)))]);
disp(['Average SSD for HSM PF: ' num2str(mean(ssd(3,:)))]);
disp(['Average SSD for VOR PF: ' num2str(mean(ssd(4,:)))]);
[h1,p1,c1] = ttest2(ssd(1,:), ssd(2,:),0.01,'left');
[h2,p2,c2] = ttest2(ssd(3,:), ssd(4,:),0.01,'left');
disp(['p value HSM-VOR: ' num2str(p1)]);
disp(['p value HSM-VOR PF: ' num2str(p2)]);


ss = [mean(ssd(1,:)) mean(ssd(3,:)) p1; mean(ssd(2,:)) mean(ssd(4,:)) p2];
z=[ss];
info.cnames=strvcat('SSD','SSD for PF','p value'); %#ok<VCAT>
info.rnames=strvcat('Motion Models','Fajen et al.','Voronoi'); %#ok<VCAT>
info.caption='Average error comparison with different methods.';
info.label='tab:comp';
table2latex(z,info);