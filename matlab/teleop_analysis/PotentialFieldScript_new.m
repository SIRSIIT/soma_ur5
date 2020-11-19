% PotentialFieldScript.m
%
clc,clear all
%% Generate some points
X = linspace(0,1,200);
Y = linspace(0,1,200);
Z = linspace(0,1,200);

[x, y, z] = meshgrid (1:200, 1:200, 1:200);

start = [0.3, 0.15, 0.75];
goal = [0.75, 0.25, 0.05];

I=find(X>=goal(1));
J=find(Y>=goal(2));
K=find(Z>=goal(3));

goal_d=[I(1),J(1),K(1)];

I=find(X>=start(1));
J=find(Y>=start(2));
K=find(Z>=start(3));

start_d=[I(1),J(1),K(1)];
%% Compute attractive force

xi = 1/5;

f = xi * ( (x - goal_d(1)).^2 + (y - goal_d(2)).^2 + (z - goal_d(3)).^2 );

%% Plan route

route_d = GradientBasedPlanner_3D(f, start_d, goal_d, 1000);

for i=1:size(route_d,1)
    
    route(i,:) = [X(round(route_d(i,1))),Y(round(route_d(i,2))),Z(round(route_d(i,3)))];
    
end

%% Plot route
figure
plot3(start(1),start(2),start(3),'k*')
hold on
plot3(goal(1),goal(2),goal(3),'r*')

plot3(route(:,1),route_d(:,2),route(:,3),'.')