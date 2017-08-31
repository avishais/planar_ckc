% In this experiment I test the projection distance and the possible fix by
% assigning a new nearest neighbor.
% are:
% Vector c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
% Vector c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs
% This while benchmarking the maximum step distance.
% Tested with BiRRT planner
% last updated: 08/31/17

clear all
clc

%%
D1 = load('test_BiRRT_PCS_projDist.txt');

D = D1;

rd = sort(unique(D(:,3)));

for i = 1:length(rd)
    M = D(D(:,3)==rd(i), :);
    dd1(i) = mean(M(:,1)); % distance before projection
    dd2(i) = mean(M(:,2)); % distance after projection
    sd(i) = sum(M(:,4))/size(M,1);
end


%%
D2 = load('test_BiRRT_GD_projDist.txt');

D = D2;

rg = sort(unique(D(:,3)));

for i = 1:length(rg)
    M = D(D(:,3)==rg(i), :);
    dg1(i) = mean(M(:,1)); % distance before projection
    dg2(i) = mean(M(:,2)); % distance after projection
    sg(i) = sum(M(:,4))/size(M,1);
end

%%
lim = [1.67 1.88];

%%

figure(1)
subplot(221)
plot(rd, dd2,'-','linewidth',2);
hold on
plot(rg, dg2,'--','linewidth',2);
hold off
legend('PCS-before new NN','GD-before new NN.','location','southeast');
xlabel('max. step length');
ylabel('distance from NN');
xlim([min([rd; rg]) max([rd; rg])]);
title('BiRRT');

subplot(222)
plot(rd, dd2./rd','-','linewidth',2);
hold on
plot(rg, dg2./rd','--','linewidth',2);
hold off
xlabel('max. step length');
ylabel('ratio of max. step size');
legend('PCS-before new NN','GD-before new NN.');
xlim([min([rd; rg]) max([rd; rg])]);

subplot(2,2,3:4)
plot(rd, sd*100, '-k','linewidth',2);
hold on
plot(rg, sg*100, '--k','linewidth',2);
plot(lim(1)*[1 1], ylim,':k','linewidth',2);
plot(lim(2)*[1 1], ylim,'-.k','linewidth',2);
hold off
xlabel('max. step length');
ylabel('failed local-conn.');
legend('PCS','GD','PCS-opt. step','GD-opt. step');
xlim([min([rd; rg]) max([rd; rg])]);
