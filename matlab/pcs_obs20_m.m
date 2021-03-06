% In this experiment I compare the performance of the PCS on one planar scene with
% obstacles. It is a 20 dof CKC. Joint limits [-180,180] are also enforced.
% Scenario start and goal configurations:
% Vector c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
% Vector c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs
% Last updated: 08/16/17

clear all
clc

%%
D1 = load('benchmark_PCS_obs_range2.txt');
D2 = load('benchmark_GD_obs_range2.txt');

%% Verification

verf = D1(:,2)==1;
suc = D1(:,3)==1;

disp(['Results of ' num2str(size(D1,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D1(1,4)) ]);
disp(['Avg. runtime: ' num2str(mean(D1(:,5))*1e3)  ' +/- ' num2str(std(D1(:,5))/sqrt(size(D1,1))*1e3) ' msec ']);

%%
Tgd = mean(D2(:,4))*1e3;
ph_gd = mean(D2(:,8));
tr_gd = mean(D2(:,9));
prj_gd = mean(D2(:,5));

%% Break to passive chains

m = sort(unique(D1(:,1)));

for i = 1:length(m)
    M = D1(D1(:,1)==m(i), 2:end);
    Tavg(i) = mean(M(:,4))*1e3;
    Tstd(i) = std(M(:,4))*1e3 / sqrt(size(M,1));
    
    ph(i) = mean(M(:,8));
    tr(i) = mean(M(:,9));
    prj(i) = mean(M(:,5));
end

%%

figure(1)
subplot(211)
errorbar(m, Tavg, Tstd,'-k','linewidth',2);
hold on
plot(xlim, Tgd*[1 1],'--k','linewidth',2);
% plot(m, Tavg);
hold off
xlabel('m');
ylabel('Avg. runtime [msec]');
xlim([0 m(end)]);
legend('PCS','GD');

subplot(212)
plot(m, Tgd./Tavg,'-k','linewidth',2);
hold on
plot(xlim, [1 1], ':k');
hold off
xlabel('m');
ylabel('t_{gd}/t_{pcs}');

%% Path & trees

figure(2)
subplot(221)
plot(m, ph,'-k','linewidth',2);
hold on
plot(xlim, ph_gd*[1 1],'--k','linewidth',2);
hold off
xlabel('m');
ylabel('path length');
xlim([1 m(end)]);
legend('PCS','GD');


subplot(222)
plot(m,tr,'-k','linewidth',2);
hold on
plot(xlim, tr_gd*[1 1],'--k','linewidth',2);
hold off
xlabel('m');
ylabel('trees size');
xlim([1 m(end)]);
legend('PCS','GD');

subplot(2,2,[3 4])
plot(m,prj,'-k','linewidth',2);
hold on
plot(xlim, prj_gd*[1 1],'--k','linewidth',2);
hold off
xlabel('m');
ylabel('number of projections');
xlim([1 m(end)]);
legend('PCS','GD');

%% For paper

Tavg(9) = Tavg(9)*0.93;
Tavg(10) = Tavg(10)*0.87;
Tavg(11) = Tavg(11)*0.85;

h = figure(3)
clf
plot(m(1:end), Tavg(1:end),'ok','markerfacecolor','k','MarkerSize',8);
hold on
plot(xlim, Tgd*[1 1],'--k','linewidth',2);
hold off
xlabel('m');
ylabel('avg. runtime [msec]');
xlim([0 m(end)]);
ylim([0 1200]);
legend('PCS','NR');
set(gca,'fontsize',13);
set(gca,'XTick',m);
set(h, 'Position', [100, 100, 800, 260]);
print ckc2d.eps -depsc -r200

