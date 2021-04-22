close all, clear all, clc
% Definition of the number of components used in GMM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load dataset 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('dataset.mat');
%% Spherical Objects

sph = [];

% Radii of Apple, Peach and Strawberry
apple = 4;
peach = 3;
straw = 1.75;

sph = [(obj{1,1}.dataset)',(obj{1,2}.dataset)',(obj{1,3}.dataset)'];
for i=1:size(sph,2)
    sph(4:6,i) = quat2eul(quaternion(sph(4:7,i)'))'; % Quaternion to Euler angles
end
sph(7,:)=[];

sph=sph.*[ones(3,1)*100;ones(5,1);ones(3,1)];
sph(9:11,:)=[];
a=size(obj{1,1}.dataset,1);b=size(obj{1,2}.dataset,1);c=size(obj{1,3}.dataset,1);
sph=vertcat((sph(1:3,:)),sph(4:8,:));
sph(9,:)=[ones(1,a)*apple,ones(1,b)*peach,ones(1,c)*straw]; 

sph = cluster_dataset(sph,size(sph,2),0,'sph');

sph_f = sph(7:8,:);

nbVar = size(sph,1);

ngrasp = 2;
 
[Data_id, Centers] = kmeans([sph(3,:)', sph(6,:)'], ngrasp,'Replicates',5); 
e = evalclusters([sph(3,:)', sph(6,:)'],'kmeans','silhouette','klist',[1:10]);
 
figure
plot(e)

sph1 = sph(:,find(Data_id==1));
sph2 = sph(:,find(Data_id==2));
sph1 = sph1(:,sph1(2,:)<0);
sph2 = sph2(:,sph2(2,:)<0); 

s = {sph1,sph2};

top = 22;
bot = 66;

for i=1:ngrasp
idx(i)=size(s{1,i},2);
end
sph1=s{1,find(idx==top)};
sph2=s{1,find(idx==bot)};

   

figure
plot3(sph1(1,:),sph1(2,:),sph1(3,:),'*','color','r')
hold on
plot3(sph2(1,:),sph2(2,:),sph2(3,:),'+','color','b')

sph1 = enhance_dataset(sph1,size(sph1,2)*5,0,'sph');
sph2 = enhance_dataset(sph2,size(sph2,2)*5,0,'sph');

figure
plot3(sph1(1,:),sph1(2,:),sph1(3,:),'*','color','r')
hold on
plot3(sph2(1,:),sph2(2,:),sph2(3,:),'+','color','b')

%% BIC computation

b1 = bic(sph1,20,20,1);
b2 = bic(sph2,20,20,1);

% b1p = bic([sph1(1:3,:);sph1(9,:)],20,20,1);
% b1o = bic([sph1(1:6,:);sph1(9,:)],20,20,1);
% 
% b1p = bic([sph2(1:3,:);sph2(9,:)],20,20,1);
% b1o = bic([sph2(1:6,:);sph2(9,:)],20,20,1);

%% Training of GMM by EM algorithm, initialized by k-means clustering.
 %Creation of a mixed model

% Single Model

nbStates_sph = 4;
 
[Priors1, Mu1, Sigma1] = EM_init_kmeans(sph1, 4); %
tmp=Priors1;
[Priors1, Mu1, Sigma1, Pix, loglik] = EM(sph1, Priors1, Mu1, Sigma1);
BIC = -2 * loglik + log(length(sph1))*4;
tmp-Priors1

% figure
% plotGMM3D(Mu1([9,1:2],:), Sigma1([9,1:2],[9,1:2],:), [0 .8 0], .2,1);
% 
% figure
% plotGMM3D(Mu1([9,1 3],:), Sigma1([9,1 3],[9,1 3],:), [0 .8 0], .2,1);


[Priors2, Mu2, Sigma2] = EM_init_kmeans(sph2, nbStates_sph); %
tmp=Priors2;
[Priors2, Mu2, Sigma2, Pix, loglik] = EM(sph2, Priors2, Mu2, Sigma2);
tmp-Priors2
BIC = -2 * loglik + log(length(sph2))*4

% figure
% plotGMM3D(Mu2([9,1:2],:), Sigma2([9,1:2],[9,1:2],:), [0 .8 0], .2,1);
% 
% figure
% plotGMM3D(Mu2([9,1 3],:), Sigma2([9,1 3],[9,1 3],:), [0 .8 0], .2,1);

%% GMR

sph_size=[1:0.5:6]; %prugna

sph_size=[7]; %prugna

% Single model
[expSph1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  sph_size, [9], [1:8]);
[expSph2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  sph_size, [9], [1:8]);

H=eye(4);
res=50;
x=0;
y=0;
z=0;
phi=0; theta=0; psi=0;
H=H+[zeros(3) [x y z]';[0 0 0 0]];

obj_data.type='Sphere';
obj_sph=SGsphere(H,sph_size,res);
figure
SGplotSolid(obj_sph,0,[0 0 0]);
hold on
axes_plot(0,0,0,0.5);

plotGMR_frame(expSph1);
plotGMR_frame(expSph2);

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$x$ [cm]','interpreter','latex','fontsize',18)
ylabel('$y$ [cm]','interpreter','latex','fontsize',18)
zlabel('$z$ [cm]','interpreter','latex','fontsize',18)

%% Pose

figure
sph_size = [1:6];  
H=eye(4);
res=50;
[expSph1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  sph_size, [9], [1:8]);
[expSph2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  sph_size, [9], [1:8]);

% for i=1:size(expSph1,2)
% [expSph_F1_tmp, expSigma_F1]= GMR(Priors_F1, Mu_F1, Sigma_F1,  [expSph1(1:6,i);sph_size(i)], [[1:6],9], [7:8]);
% [expSph_F2_tmp, expSigma_F2]= GMR(Priors_F2, Mu_F2, Sigma_F2,  [expSph2(1:6,i);sph_size(i)], [[1:6],9], [7:8]);
% expSph_F1(:,i)=expSph_F1_tmp(:,1);
% expSph_F2(:,i)=expSph_F2_tmp(:,1);
% end

t = linspace(0,pi)';
x = cos(t);
y = sin(t);
color = jet(size(expSph1,2));



for i=1:size(expSph1,2)
    obj_sph=SGsphere(H,i,res);
    SGplotSolid(obj_sph,0.01,color(i,:));
    hold on
    plotGMR_frame(expSph1(:,i),[],color(i,:));
    
    plotGMR_frame(expSph2(:,i),[],color(i,:),0);
end
axis equal
legend({'r = 1 cm','r = 2 cm','r = 3 cm','r = 4 cm','r = 5 cm','r = 6 cm'},'FontSize',20,'Interpreter','latex')
colormap(color)

xlabel('x[cm]','interpreter','latex','fontsize',24)
ylabel('y[cm]','interpreter','latex','fontsize',24)
zlabel('z[cm]','interpreter','latex','fontsize',24)
set(gca,'FontSize',24,'TickLabelInterpreter','latex');

%% Position

l=size(expSph1,2);
figure

for s=1:l
%     color = [1-s/l,0,1/l*s];
plotGMM3D(expSph1([1:3],s), expSigma1([1:3],[1:3],s), color(s,:) , 0.2,1);
hold on
end
plot3(expSph1(1,:), expSph1(2,:), expSph1(3,:), 'x', 'lineWidth', 5, 'color', [1 1 1])

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expSph2([1:3],s), expSigma2([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
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

figure

subplot(3,1,1)
% for i=1:length(sph_size)
% expSph1(4:6,i)=rotm2eul(inv(rotx(-90))*eul2rotm(expSph1(4:6,i)'))';
% expSph2(4:6,i)=rotm2eul(inv(rotx(-90))*eul2rotm(expSph2(4:6,i)'))';
% end
sph_size_int = [1:.5:6];
phi_int1 = interp1(sph_size,rad2deg(expSph1(4,:)),sph_size_int);
phi_int2 = interp1(sph_size,rad2deg(expSph2(4,:)),sph_size_int);
theta_int = interp1(sph_size,rad2deg(expSph1(5,:)),sph_size_int);
psi_int = interp1(sph_size,rad2deg(expSph1(6,:)),sph_size_int);
% plot(x,v,'o',xq,vq1,':.');sph_size_int,phi_int1,':.', sph_size_int,phi_int2,':.',

plot(sph_size,rad2deg(expSph1(4,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(sph_size,rad2deg(expSph2(4,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
% end
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\phi [^{\circ}]$','interpreter','latex','fontsize',18)
yticklabels('auto')
xlim([1 6])
subplot(3,1,2)
plot(sph_size,rad2deg(expSph1(5,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(sph_size,rad2deg(expSph2(5,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta [^{\circ}]$','interpreter','latex','fontsize',18)
xlim([1 6])
subplot(3,1,3)
plot(sph_size,rad2deg(expSph1(6,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(sph_size,rad2deg(expSph2(6,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\psi [^{\circ}]$','interpreter','latex','fontsize',18)
xlim([1 6])
% axis equal

legend({'Top','Bottom'},'FontSize',20,'Interpreter','latex')

%% Fingers - Single Model

figure
subplot(2,1,1)
plot(sph_size,expSph1(7,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on,
plot(sph_size,expSph2(7,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_R$ [deg]','interpreter','latex','fontsize',18)
xlim([1 6])
subplot(2,1,2)
plot(sph_size,expSph1(8,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on,
plot(sph_size,expSph2(8,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_L$ [deg]','interpreter','latex','fontsize',18)
xlim([1 6])

legend({'Top','Bottom'},'FontSize',20,'Interpreter','latex')