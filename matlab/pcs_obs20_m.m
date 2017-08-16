% In this experiment I compare the performance of the PCS on one planar scene with
% obstacles. It is a 20 dof CKC. Joint limits [-180,180] are also enforced.
% Scenario start and goal configurations:
% Vector c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
% Vector c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs
% Last updated: 08/16/17

clear all
clc

%%