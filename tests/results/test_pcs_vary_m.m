% In this test, I test the visibility with regards to the number of passive
% chains used.
% This test is for a 20 DOF planar CKC with [-180,180] joint limits while
% using up to 20 passive chains.
% Random start and goal configurations are sampled and tested for
% connection. The sampling strategy is sampling a feasible point, sampling
% a random point and choping it from the first point, projecting and
% checking local connection.
% last updated: 08/17/17

% Uses test_pcs_vary_m.cpp
% pcs_rbs_verification.txt with seed: 1502899207
% Results in the above files in this form:
% f << {success} << " " << {path verified} << " " << {distance between confs.} << " " << {success with m=1} << ... << {success with m=n} << endl;

clear all
clc

D = load('pcs_m_analysis.txt');

M = D(:,4:end);
m = size(M,2);

%% Verification

suc = D(:,1)==1;
verf = D(:,2)==1;

disp(['Results of ' num2str(size(D,1)) ' local-connection queries.']);

disp(['Percent of successful local-connections that were verified: ' num2str(sum(verf & suc)/sum(suc)*100) '%']);

%%

disp(['Success rate/visibility for the RBS: ' num2str(sum(suc)/size(D,1)*100) '%']);

%% visibility

for i = 1:m
    V(i) = sum(M(:,i))/size(M,1);    
end

figure(1)
plot(1:m, V*100,'o-k','linewidth',2.5);
xlabel('m');
ylabel('visibility [%]');
xlim([1 m]);

%%
load('Vgd.mat');
% Vgd = [100 Vgd];
Vgd = [Vgd Vgd(end)];

%% Visibility vs distance

clear V d
Dd = D(:,3);
max_d = max(Dd);
d = linspace(0, max_d, 50);
for j = 1:m
    for i = 2:length(d)
        S = M(D(:,3)>=d(i-1) & D(:,3)<d(i), j);
        Vd(i-1,j) = sum(S)/length(S) * 100;
    end
end
Vd = [100*ones(1, size(Vd,2)); Vd];

h = figure(2)
clf
plot(d, Vd(:,1),'-k','linewidth',2.);
hold on
plot(d, Vd(:,2),':k','linewidth',2.5);
plot(d, Vd(:,3),'--k','linewidth',2.5);
plot(d, Vd(:,7),'-.k','linewidth',2.5);
% plot(d, Vd(:,12),'-b','linewidth',2.5);
plot(d, Vd(:,20),'-k','linewidth',3.5);
plot(dgd,Vgd,'-r','linewidth',4.5);
hold off
xlabel('distance: $\sqrt{(\phi_1-\phi_2)^T(\phi_1-\phi_2)}$','Interpreter','latex');
ylabel('success rate [%]');
legend('m=1','m=2','m=3','m=7','m=20','NR');
xlim([0 12]);
set(gca,'fontsize',13);

set(h, 'Position', [100, 100, 800, 260]);
print visibility2d.eps -depsc -r200