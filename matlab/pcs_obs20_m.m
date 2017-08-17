% In this experiment I compare the performance of the PCS on one planar scene with
% obstacles. It is a 20 dof CKC. Joint limits [-180,180] are also enforced.
% Scenario start and goal configurations:
% Vector c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
% Vector c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs
% Last updated: 08/16/17

clear all
clc

%%
D = load('benchmark_PCS_obs_range2.txt');
D(111,:) = [];
%% Verification

verf = D(:,2)==1;
suc = D(:,3)==1;

disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Percent of successful queries verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);
disp(['Plan distance: ' num2str(D(1,4)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,5))*1e3)  ' +/- ' num2str(std(D(:,5))/sqrt(size(D,1))*1e3) ' msec ']);

%% Break to passive chains

m = sort(unique(D(:,1)));

for i = 1:length(m)
    M = D(D(:,1)==m(i), 2:end);
    Tavg(i) = mean(M(:,4))*1e3;
    Tstd(i) = std(M(:,4))*1e3 / sqrt(size(M,1));
end

figure(1)
errorbar(m, Tavg, Tstd);
% plot(m, Tavg);
xlabel('m');
ylabel('Avg. runtime [msec]');