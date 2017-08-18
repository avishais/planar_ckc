% In this test I test the planning time with regards to the dimenstion of
% the CKC without obstaclese and self-collisions constraints.
% The base-links ratio is 0.3;
% Random start and goal configurations.
% Last update: 8/17/2017

clear all
clc

%%
D1 = load('benchmark_D_PCS.txt');
verf = D1(:,2)==1;
suc = D1(:,3)==1;
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);

D2 = load('benchmark_D_GD.txt');
verf = D2(:,2)==1;
suc = D2(:,3)==1;
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);



%% PCS - m = n
dn = unique(D1(:,1));
dn = sort(dn);

for i = 1:length(dn)
    dd = dn(i);
    
    M = D1(D1(:,1)==dd,3:end);
    suc = M(:,1)==1;
    
    success_pcs(i) = sum(suc)/size(M,1)*100;
    runtime_pcs(i) = mean(M(suc,3))*1000;
    runtime_pcs_std(i) = std(M(suc,3))/size(M,1)*1000;    
end


%% GD
dgd = unique(D2(:,1));
dgd = sort(dgd);

for i = 1:length(dgd)
    dd = dgd(i);
    
    M = D2(D2(:,1)==dd,3:end);
    suc = M(:,1)==1;%  & M(:,3) <= 2;
    
    success_gd(i) = sum(suc)/size(M,1)*100;
    runtime_gd(i) = mean(M(suc,3))*1000;
    runtime_gd_std(i) = std(M(suc,3))/size(M,1)*1000;    
end

%% 

h = figure(1);
clf
subplot(311)
plot(dn(1:end-1), runtime_pcs(1:end-1),'-ok','MarkerFaceColor','k');
hold on
plot(dgd(1:end-1), runtime_gd(1:end-1),':dk','MarkerFaceColor','k','linewidth',2);
hold off
xlabel('n');
ylabel('runtime [msec]');
legend('PCS', 'GD','location','northwest');
set(gca,'fontsize',15)
xlim([min(dn) max(dn(1:end-1))]);

subplot(312)
ratio_pcs_gd = runtime_gd(1:end-1)./runtime_pcs(1:numel(dgd)-1);
plot(dgd(1:end-1), ratio_pcs_gd,'-ok','MarkerFaceColor','k');
hold on
plot(xlim,[1 1],':k');
hold off
xlabel('n');
ylabel('ratio t_{GD}/t_{PCS}');
set(gca,'fontsize',15)
xlim([min(dn) max(dn(1:end-1))]);

subplot(313)
plot(dn(1:end-1), success_pcs(1:end-1),'-ok','MarkerFaceColor','k');
hold on
plot(dgd(1:end-1), success_gd(1:end-1),':dk','MarkerFaceColor','k','linewidth',2);
hold off
xlabel('n');
ylabel('success rate [%]');
legend('PCS','GD');
set(gca,'fontsize',15)
xlim([min(dn) max(dn(1:end-1))]);



set(h, 'Position', [100, 100, 800, 900]);
% print('ckc_dimension_w_gd', '-dpng','-r200');


