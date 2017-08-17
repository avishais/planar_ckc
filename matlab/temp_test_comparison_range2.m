

clear all
clc

%%
D1 = load('benchmark_PCS_obs_range2_m20.txt'); 

verf = D1(:,2)==1;
suc = D1(:,3)==1;
D1 = D1(:,3:end);

disp('PCS:');
disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,2)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,3))*1e3)  ' +/- ' num2str(std(D1(:,3))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D1(:,7)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D1(:,8)))) ]);
disp(['Avg. number of IK solutions: ' num2str(floor(mean(D1(:,4)))) ]);

%%
D2 = load('benchmark_PCS_obs_range2_m18_old.txt'); 

suc = D2(:,3)==1;
D2 = D2(:,2:end);

disp('PCS - old:');
disp(['Results of ' num2str(size(D2,1)) ' queries.']);
disp(['Plan distance: ' num2str(D2(1,2)) ]);
disp(['Avg. runtime: ' num2str(mean(D2(:,3))*1e3)  ' +/- ' num2str(std(D2(:,3))/sqrt(size(D1,1))*1e3) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D2(:,7)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D2(:,8)))) ]);
disp(['Avg. number of IK solutions: ' num2str(floor(mean(D2(:,4)))) ]);


%%
% PCS
td = D1(:,3);
maxT = max(td);
T1 = linspace(0,maxT,1000);
T1 = T1(2:end);
for i = 1:length(T1)
    sd = td < T1(i);
    md(i) = mean(td(sd));
    Md(i) = 1-sum(sd)/length(td);
end
%%
% GD
tg = D2(:,3);
maxT = max(tg);
T2 = linspace(0,maxT,1000);
T2 = T2(2:end);
for i = 1:length(T2)
    sg = tg < T2(i);
    mg(i) = mean(tg(sg));
    Mg(i) = 1-sum(sg)/length(tg);
end
%%
h = figure(1);
clf
plot(T1,Md*100,'-k','linewidth',2);
hold on
plot(mean(td)*[1 1], ylim,':k','linewidth',1.5);
plot(T2,Mg*100,'--k','linewidth',2);
plot(mean(tg)*[1 1], ylim,'-.k','linewidth',1.5);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
legend('PCS','PCS-avg.','GD','GD-avg.');
xlim([0 max(T1)]);
% set(h, 'Position', [100, 100, 800, 400]);

h = figure(2);
clf
semilogy(T1,Md*100,'-k','linewidth',2);
hold on
semilogy(T2,Mg*100,'--k','linewidth',2);
hold off
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
legend('PCS','GD');
% xlim([0 6]);
% set(h, 'Position', [100, 100, 800, 400]);

%% 
disp(' ');
disp(['Speed-up t_{pcs}/t_{gd}: ' num2str(mean(D1(:,3))/mean(D2(:,3))) ]);
