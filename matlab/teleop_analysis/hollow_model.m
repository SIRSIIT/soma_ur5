close all, clear all, clc
% Definition of the number of components used in GMM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load dataset 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('dataset.mat');
%% Hollow Objects

hol = [];

% Radii and heights of Mug, Bowl and Funnel
mug = [4.5;8];
bowl = [8;5];
funnel = [6;13];

hol = [(obj{1,10}.dataset)',(obj{1,11}.dataset)',(obj{1,12}.dataset)'];
for i=1:size(hol,2)
    hol(4:6,i) = quat2eul(quaternion(hol(4:7,i)'))';
end
hol(7,:)=[];


hol=hol.*[ones(3,1)*100;ones(5,1);ones(3,1)];
hol(9:11,:)=[];
a=size(obj{1,10}.dataset,1);b=size(obj{1,11}.dataset,1);c=size(obj{1,12}.dataset,1);
hol=vertcat((hol(1:3,:)),hol(4:8,:));
hol(9:10,:)=[ones(2,a).*mug,ones(2,b).*bowl,ones(2,c).*funnel]; 


hol = cluster_dataset(hol,size(hol,2)*1,2,'hol');

nbVar = size(hol,1);

ngrasp = 2;
 
 [Data_id, Centers] = kmeans([hol(3,:)', hol(6,:)'], ngrasp,'Replicates',5); 
 e = evalclusters([hol(3,:)', hol(6,:)'],'kmeans','silhouette','klist',[1:10]);
 
 figure
 plot(e)
% [Data_id, Centers] = kmeans([hol(3:6,:)'], ngrasp,'Replicates',5); 
    hol1 = hol(:,find(Data_id==1));
    hol2 = hol(:,find(Data_id==2));

    figure
    plot3(hol1(1,:),hol1(2,:),hol1(3,:),'*','color','r')
    hold on
    plot3(hol2(1,:),hol2(2,:),hol2(3,:),'+','color','b')
    
    hol1 = enhance_dataset(hol1,size(hol1,2)*5,2,'hol');
    hol2 = enhance_dataset(hol2,size(hol2,2)*5,2,'hol');
    
    
top = 43*5;
bot = 47*5;

h = {hol1,hol2};
for i=1:ngrasp
idx(i)=size(h{1,i},2);
end


hol1=h{1,find(idx==top)};
hol2=h{1,find(idx==bot)};

hol1 = hol1(:,hol1(2,:)<-0.5);
hol1 = hol1(:,hol1(6,:)<0);
hol2 = hol2(:,hol2(2,:)<0);
% hol2 = hol2(:,hol2(1,:)>-1.5);
% hol1 = enhance_dataset(hol1,size(hol1,2)*5,2,'hol');

figure
plot3(hol1(1,:),hol1(2,:),hol1(3,:),'*','color','r')
hold on
plot3(hol2(1,:),hol2(2,:),hol2(3,:),'+','color','b')
%plot3(hol3(1,:),hol3(2,:),hol3(3,:),'o','color','g')

new_hol = [hol1,hol2];
e = evalclusters([new_hol(3,:)', new_hol(6,:)'],'kmeans','silhouette','klist',[1:10]);
 
 figure
 plot(e)

%% BIC computation

b1 = bic(hol1,20,20,1);
b2 = bic(hol2,20,20,1);

%% Model
% Training of GMM by EM algorithm, initialized by k-means clustering.
 %Creation of a mixed model
 
nbStates_hol = 4;

[Priors1, Mu1, Sigma1] = EM_init_kmeans(hol1,8); %
tmp=Priors1;
[Priors1, Mu1, Sigma1, Pix, loglik] = EM(hol1, Priors1, Mu1, Sigma1);
% BIC = -2 * loglik + log(length(hol))* 7
tmp-Priors1

[Priors2, Mu2, Sigma2] = EM_init_kmeans(hol2,4); %
tmp=Priors2;
[Priors2, Mu2, Sigma2, Pix, loglik] = EM(hol2, Priors2, Mu2, Sigma2);
tmp-Priors2

%% GMR

mug = [4.5;8];
bowl = [8;5];
funnel = [6;10];

hol_size=[4;7];
% hol_size=[6;10];

[expHol1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  hol_size, [9:10], [1:8]);
[expHol2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  hol_size, [9:10], [1:8]);

H=eye(4);
res=50;
x=0;
y=0;
z=0;
phi=0; theta=0; psi=0;
H=H+[zeros(3) [x y z]';[0 0 0 0]];

obj_data.type='Cylinder';
H=eye(4);
obj_hol=SGcylinder(H,hol_size(2),hol_size(1),res);

figure
SGplotSolid(obj_hol);
hold on
axes_plot(0,0,0,0.5);

plotGMR_frame(expHol1(:,1));
plotGMR_frame(expHol2(:,1));

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$x$ [cm]','interpreter','latex','fontsize',18)
ylabel('$y$ [cm]','interpreter','latex','fontsize',18)
zlabel('$z$ [cm]','interpreter','latex','fontsize',18)
axis equal

%% Pose

r=3:9;
h=repmat(mean([5,8,10]),1,length(r));
hol_size = [r;h]; 

% h=3:2:15;
% r=repmat(mean([4.5,6,8]),1,length(h));
% hol_size = [r;h]; 

figure

[expHol1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  hol_size, [9:10], [1:8]);
[expHol2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  hol_size, [9:10], [1:8]);

t = linspace(0,pi)';
x = cos(t);
y = sin(t);
color = jet(size(expHol1,2)).*0.85;

for i=1:size(expHol1,2)
    obj_cyl=SGcylinder(H,hol_size(2,i),hol_size(1,i),res);
    SGplotSolid(obj_cyl,0.2,color(i,:));
    hold on
    plotGMR_frame(expHol1(:,i),[],color(i,:));
    
    plotGMR_frame(expHol2(:,i),[],color(i,:),0);
end
axis equal
% legend({'r = 3 cm','r = 4 cm','r = 5 cm','r = 6 cm','r = 7 cm','r = 8 cm','r = 9 cm'},'FontSize',20,'Interpreter','latex')
legend({'h = 3 cm','h = 5 cm','h = 7 cm','h = 9 cm','h = 11 cm','h = 13 cm', 'h = 15 cm'},'FontSize',20,'Interpreter','latex')

colormap(color)

xlabel('x[cm]','interpreter','latex','fontsize',24)
ylabel('y[cm]','interpreter','latex','fontsize',24)
zlabel('z[cm]','interpreter','latex','fontsize',24)
set(gca,'FontSize',24,'TickLabelInterpreter','latex');

%% Position

l=size(expHol1,2);
figure

for s=1:l
%     color = [1-s/l,0,1/l*s];
plotGMM3D(expHol1([1:3],s), expSigma1([1:3],[1:3],s), color(s,:) , 0.2,1);
hold on
end
plot3(expHol1(1,:), expHol1(2,:), expHol1(3,:), 'x', 'lineWidth', 5, 'color', [1 1 1])

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expHol2([1:3],s), expSigma2([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
hold on
end


plot3(0, 0, 0, 'x', 'lineWidth', 50, 'color', [1 0 0])

axis equal
xlabel('x[cm]','interpreter','latex','fontsize',24)
ylabel('y[cm]','interpreter','latex','fontsize',24)
zlabel('z[cm]','interpreter','latex','fontsize',24)
set(gca,'FontSize',24,'TickLabelInterpreter','latex');
% end
% hold on
% SGplotSolid(obj_sph,0);


%% Orientation

[~,idx]=max([hol_size(1,end)-hol_size(1,1),hol_size(2,end)-hol_size(2,1)]);

figure

subplot(3,1,1)
% for i=1:length(sph_size)
plot(hol_size(idx,:),rad2deg(expHol1(4,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(hol_size(idx,:),rad2deg(expHol2(4,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)

% end
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\phi [^{\circ}]$','interpreter','latex','fontsize',18)
yticklabels('auto')
% xlim([1 6])
subplot(3,1,2)
plot(hol_size(idx,:),rad2deg(expHol1(5,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(hol_size(idx,:),rad2deg(expHol2(5,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta [^{\circ}]$','interpreter','latex','fontsize',18)
% xlim([1 6])
subplot(3,1,3)
plot(hol_size(idx,:),rad2deg(expHol1(6,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(hol_size(idx,:),rad2deg(expHol2(6,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\psi [^{\circ}]$','interpreter','latex','fontsize',18)
% xlim([1 6])
% axis equal

legend({'top grasp','bottom grasp'},'FontSize',20,'Interpreter','latex')

%% Fingers

[~,idx]=max([hol_size(1,end)-hol_size(1,1),hol_size(2,end)-hol_size(2,1)]);

figure
subplot(2,1,1)
plot(hol_size(idx,:),expHol1(7,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on,
plot(hol_size(idx,:),expHol2(7,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_R$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])
subplot(2,1,2)
plot(hol_size(idx,:),expHol1(8,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on,
plot(hol_size(idx,:),expHol2(8,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_L$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])

legend({'top grasp','bottom grasp'},'FontSize',20,'Interpreter','latex')