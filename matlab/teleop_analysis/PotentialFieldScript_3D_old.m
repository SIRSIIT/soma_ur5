%
% PotentialFieldScript.m
%
clc,clear all
%% Generate some points

nrows = 20;
% ncols = 600;

obstacle = false(nrows, nrows,nrows);

[x, y, z] = meshgrid (1:nrows, 1:nrows, 1:nrows);

%% Generate some obstacle

% obstacle (300:end, 100:250) = true;
% obstacle (5, 5, 5) = true;

t = ((x - 5).^2 + (y - 5).^2 + (z - 5).^2) < 4^2;
obstacle(t) = true;
% 
% t = ((x - 4).^2 + (y - 3).^2 + (z - 6).^2) < 100^2;
% obstacle(t) = true;

%% Compute distance transform

d = bwdist(obstacle);

% Rescale and transform distances

d2 = (d/100) + 1;

d0 = 2;
nu = 10;

repulsive = nu*((1./d2 - 1/d0).^2);

repulsive (d2 > d0) = 0;


%% Display repulsive potential

% figure;
% m = mesh (repulsive);
% m.FaceLighting = 'phong';
% axis equal;
% 
% title ('Repulsive Potential');

%% Compute attractive force

goal = [2, 18, 7];

xi = 1/5;

attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 + (z - goal(3)).^2 );

% figure;
% m = meshz (attractive);
% m.FaceLighting = 'phong';
% axis equal;
% 
% title ('Attractive Potential');

%% Display 2D configuration space

% figure;
% imshow(~obstacle);
% 
% hold on;
% plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
% hold off;
% 
% axis ([0 ncols 0 nrows]);
% axis xy;
% axis on;
% 
% xlabel ('x');
% ylabel ('y');
% 
% title ('Configuration Space');

%% Combine terms

f = attractive + repulsive;
% 
% figure;
% m = mesh (f);
% m.FaceLighting = 'phong';
% axis equal;
% 
% title ('Total Potential');

%% Plan route
start = [1, 3, 3];
% f = attractive ;
route = GradientBasedPlanner (f, start, goal, 1000);
    
%% Plot route
figure
plot3(start(1),start(2),start(3),'k*')
hold on
plot3(goal(1),goal(2),goal(3),'r*')

plot3(route(:,1),route(:,2),route(:,3))

%% Plot the energy surface

figure;
m = mesh (f);
axis equal;

%% Plot ball sliding down hill

[sx, sy, sz] = sphere(20);

scale = 20;
sx = scale*sx;
sy = scale*sy;
sz = scale*(sz+1);

hold on;
p = mesh(sx, sy, sz);
p.FaceColor = 'red';
p.EdgeColor = 'none';
p.FaceLighting = 'phong';
hold off;

for i = 1:size(route,1)
    P = round(route(i,:));
    z = f(P(2), P(1));
    
    p.XData = sx + P(1);
    p.YData = sy + P(2);
    p.ZData = sz + f(P(2), P(1));
    
    drawnow;
    
    drawnow;
    
end

%% quiver plot
[gx, gy, gz] = gradient (-f);
skip = 20;

figure;
ncols=nrows;
xidx = 1:skip:ncols;
yidx = 1:skip:nrows;

quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);

axis ([1 ncols 1 nrows]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);