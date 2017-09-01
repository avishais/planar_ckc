% In this experiment I test the GD and PCS with the SBL planner while varying the step size
% are:
% Vector c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
% Vector c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs
% Last updated: 08/27/17

clear all
clc

%%
D1 = load('benchmark_SBL_PCS_obs_rangeB.txt'); 

D2 = load('benchmark_SBL_GD_obs_rangeB_IKobsCheck.txt'); 

D3 = load('benchmark_SBL_HB_obs_rangeB.txt'); 

%%
% PCS
rd = sort(unique(D1(:,1)));
for i = 1:length(rd)
    M = D1(D1(:,1)==rd(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    td(i) = mean(M(:,4))*1e3;
    td_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    nodes_d(i) =  mean(M(:,10));
    trees_d(i) =  mean(M(:,11));
    tlc_d(i) = mean(M(:,12))*1e3;
end
%%
% GD
rg = sort(unique(D2(:,1)));
for i = 1:length(rg)
    M = D2(D2(:,1)==rg(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tg(i) = mean(M(:,4))*1e3;
    tg_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    nodes_g(i) =  mean(M(:,10));
    trees_g(i) =  mean(M(:,11));
    tlc_g(i) = mean(M(:,12))*1e3;
end

%%
% HB
rb = sort(unique(D3(:,1)));
for i = 1:length(rb)
    M = D3(D3(:,1)==rb(i), 2:end);
%     disp([i-1 rg(i)  size(M,1)])
    tb(i) = mean(M(:,4))*1e3;
    tb_ste(i) = 1e3*std(M(:,4))/sqrt(size(M,1));
    nodes_b(i) =  mean(M(:,10));
    trees_b(i) =  mean(M(:,11));
    tlc_b(i) = mean(M(:,12))*1e3;
end

%%
disp(' ');
[tdmin, id] = min(td);
disp(['Minimum avg. runtime for PCS is ' num2str(tdmin) 'msec with d = ' num2str(rd(id)) ]);
[tgmin, ig] = min(tg);
disp(['Minimum avg. runtime for GD is ' num2str(tgmin) 'msec with d = ' num2str(rg(ig)) ]);
[tbmin, ib] = min(tb);
disp(['Minimum avg. runtime for HB is ' num2str(tbmin) 'msec with d = ' num2str(rb(ib)) ]);

%%
h = figure(1);
clf
errorbar(rd,td,td_ste,'-k','linewidth',2);
hold on
errorbar(rg,tg,tg_ste,'--k','linewidth',2);
errorbar(rb,tb,tb_ste,':k','linewidth',2);
hold off
ylabel('mean runtime [msec]');
xlabel('max. local-connection distance');
legend('PCS','GD','HB');
% xlim([0 6]);
xlim([0 max(rg)]);

%%
D1 = D1(D1(:,1)==rd(id), 2:end);
verf = D1(:,1)==1;
suc = D1(:,2)==1;

disp('------------------------------------');
disp('PCS - with optimized step size:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,4))*1e3)  ' +/- ' num2str(std(D1(:,4))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,8)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,9)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D1(:,5)))) ]);

%%
D2 = D2(D2(:,1)==rg(ig), 2:end);
verf = D2(:,1)==1;
suc = D2(:,2)==1;

disp('------------------------------------');
disp('GD - with optimized step size:');
disp(['Results of ' num2str(size(D2,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D2(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D2(:,4))*1e3)  ' +/- ' num2str(std(D2(:,4))/sqrt(size(D2,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D2(:,8)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D2(:,9)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D2(:,5)))) ]);

%%
D3 = D3(D3(:,1)==rb(ib), 2:end);
verf = D3(:,1)==1;
suc = D3(:,2)==1;

disp('------------------------------------');
disp('HB - with optimized step size:');
disp(['Results of ' num2str(size(D3,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D3(1,3)) ]);
disp(['Avg. runtime: ' num2str(mean(D3(:,4))*1e3)  ' +/- ' num2str(std(D3(:,4))/sqrt(size(D3,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D3(:,8)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D3(:,9)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D3(:,5)))) ]);
%%
% PCS
td = D1(:,4);
maxT = max(td);
T1 = linspace(0,maxT,100);
T1 = T1(2:end);
for i = 1:length(T1)
    sd = td < T1(i);
    md(i) = mean(td(sd));
    Md(i) = 1-sum(sd)/length(td);
end

%%
% GD
tg = D2(:,4);
maxT = max(tg);
T2 = linspace(0,maxT,100);
T2 = T2(2:end);
for i = 1:length(T2)
    sg = tg < T2(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end

%%
% HB
tb = D3(:,4);
maxT = max(tb);
T3 = linspace(0,maxT,100);
T3 = T3(2:end);
for i = 1:length(T3)
    sb = tb < T3(i);
    mb(i) = mean(tb(sb));
    Mb(i) = 1-sum(sb)/length(tb);
end
%%
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
hold on
plot(T2,Mg*100,'--k','linewidth',2);
plot(T3,Mb*100,':k','linewidth',2);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
xlim([0 max([T1 T2])]);
legend('PCS','GD','HB');
title('SBL');
% set(h, 'Position', [100, 100, 800, 400]);

%% 
disp(' ');
disp(['Speed-up t_{gd}/t_{pcs}: ' num2str(tdmin/tgmin) ]);
disp(['Speed-up t_{gd}/t_{hb}: ' num2str(tgmin/tbmin) ]);
disp(['Speed-up t_{pcs}/t_{hb}: ' num2str(tdmin/tbmin) ]);
