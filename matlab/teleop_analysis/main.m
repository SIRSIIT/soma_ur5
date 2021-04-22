%% Set things up
close all
clear
clc
% set(groot,'defaulttextinterpreter','latex');
% set(groot, 'defaultAxesTickLabelInterpreter','latex');
% set(groot, 'defaultLegendInterpreter','latex');


ta=teleop_analysis;
% [exp,cs] = ta.dataset_generation;
[obj_w] = ta.dataset_generation;
obj_w=obj_w(~cellfun('isempty',obj_w));
obj = ta.world2obj(obj_w);
ta.plot_obj_ee_pose(obj)