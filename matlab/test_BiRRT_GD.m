% In this experiment I test the GD with BiRRT while varying the step size
% are:
% Vector c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
% Vector c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs
% Last updated: 08/26/17

clear all
clc

File = 'benchmark_BiRRT_GD_obs_rangeB.txt'; % Not checking collision for the milestones

%%
D1 = load(File); 
% D1(D1(:,1)==0.05 | D1(:,1)>=2.55,:) = [];

verf = D1(:,2)==1;
suc = D1(:,3)==1;

disp('PCS:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,4)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,5))*1e3)  ' +/- ' num2str(std(D1(:,5))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,8)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,9)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D1(:,5)))) ]);

%%
% GD
rg = sort(unique(D1(:,1)));
for i = 1:length(rg)
    M = D1(D1(:,1)==rg(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tg(i) = mean(M(:,4))*1e3;
    tg_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    nodes_g(i) =  mean(M(:,10));
    trees_g(i) =  mean(M(:,11));
    tlc_g(i) = mean(M(:,12))*1e3;
end

%%
disp(' ');
[tgmin, ig] = min(tg);
disp(['Minimum avg. runtime for GD is ' num2str(tgmin) 'msec with d = ' num2str(rg(ig)) ]);

%%
h = figure(1);
clf
errorbar(rg,tg,tg_ste,'--k','linewidth',2);
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
% xlim([0 6]);
xlim([0 max(rg)]);

%%
F = load(File); 

D1 = F(F(:,1)==rg(ig), 2:end);

verf = D1(:,1)==1;
suc = D1(:,2)==1;

disp('GD - with optimized step size:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,4))*1e3)  ' +/- ' num2str(std(D1(:,4))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,8)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,9)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D1(:,5)))) ]);

%%
D = D1;

% GD
tg = D(:,4);
maxT = max(tg);
T1 = linspace(0,maxT,100);
T1 = T1(2:end);
for i = 1:length(T1)
    sg = tg < T1(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end
%%
h = figure(2);
clf
plot(T1,Mg*100,'-k','linewidth',2);
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
xlim([0 max([T1 T1])]);
title('SBL');
% set(h, 'Position', [100, 100, 800, 400]);