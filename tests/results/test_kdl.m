% In this test, I compare the projection of the kdl and my own gd.
% Projection of random points and compare distance of projectio.
% Seed = 1502774265
% last updated: 08/14/17

% Results in kdl_gd.txt in this form:
%  f << {kdl success} << " " << kdl_time << " " << {kdl projection distance} << " " << {gd success} << " " << gd_time << " " << {gd projection distance} << endl;


clear all
clc

D = load('kdl_verification.txt');

disp(['Number of projections: ' num2str(size(D,1)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3))*1e3) ' +/- ' num2str(std(D(:,3))/sqrt(size(D(:,3),1))*1e3) ' msec ']);

disp(['Success rate: ' num2str(sum(D(:,1))/size(D,1)*100) '%']);
disp(['Successful constraint verification: ' num2str(sum(D(:,2))/size(D,1)*100) '%']);


