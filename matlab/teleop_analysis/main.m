%% Set things up
close all
clear
clc
% set(groot,'defaulttextinterpreter','latex');
% set(groot, 'defaultAxesTickLabelInterpreter','latex');
% set(groot, 'defaultLegendInterpreter','latex');


ta=teleop_analysis;
% ec.errorBBox
% ec.plot_force_and_vel()
% ec.plot_motor_status()
[experiments,a] = ta.dataset_generation;
%%

figure
for i=1:size(a,1)
    plot3(a(i,1),a(i,2),a(i,3),'.'),hold on
end