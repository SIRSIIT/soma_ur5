% close all, clear all, clc
% % Definition of the number of components used in GMM.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % Load dataset 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load('dataset.mat');
% % load('cyl.mat');
% %% Cylindrical Objects
% 
% cyl = [];
% 
% % Radii and heights of Chips can, Candy tube and Small cylinder
% c_can = [3.3;23];
% c_tube = [1.8;22.5];
% s_cyl = [2;13];
% % c_can = [1;1];     
% % c_tube = [0;0.95];
% % s_cyl = [0.16;0]; 
% 
% cyl = [(obj{1,4}.dataset)',(obj{1,5}.dataset)',(obj{1,6}.dataset)'];
% % cyl = [(obj{1,4}.dataset)'];
% for i=1:size(cyl,2)
%     cyl(4:6,i) = quat2eul(quaternion(cyl(4:7,i)'))';
% end
% cyl(7,:)=[];
% 
% cyl=cyl.*[ones(3,1)*100;ones(5,1);ones(3,1)];
% cyl(9:11,:)=[];
% a=size(obj{1,4}.dataset,1);b=size(obj{1,5}.dataset,1);c=size(obj{1,6}.dataset,1);
% 
% cyl(9:10,:)=[ones(2,a).*c_can,ones(2,b).*c_tube,ones(2,c).*s_cyl]; 
% 
% nbVar = size(cyl,1);
% 
% 
% cyl_f = cyl(7:8,:);
%%
close all, clear all, clc

load('cyl_data.mat') % Fixed cylinder dataset
cyl = cluster_dataset(cyl,size(cyl,2)*5,2,'cyl');
 ngrasp = 3;
 
 [Data_id, Centers] = kmeans([cyl(3,:)', cyl(5,:)', cyl(6,:)'], ngrasp,'Replicates',5); 
%  e = evalclusters([cyl(3,:)', cyl(5,:)', cyl(6,:)'],'kmeans','silhouette','klist',[1:10]);
%  
%  figure
%  plot(e)
% [Data_id, Centers] = kmeans([cyl(3:6,:)'], ngrasp,'Replicates',5); 
    cyl1 = cyl(:,find(Data_id==1));
    cyl2 = cyl(:,find(Data_id==2));
    cyl3 = cyl(:,find(Data_id==3));
        
    cyl1 = enhance_dataset(cyl1,size(cyl1,2)*5,2,'cyl');
    cyl2 = enhance_dataset(cyl2,size(cyl2,2)*5,2,'cyl');
    cyl3 = enhance_dataset(cyl3,size(cyl3,2)*5,2,'cyl');
    
     figure
    plot3(cyl1(1,:),cyl1(2,:),cyl1(3,:),'*','color','r')
    hold on
    plot3(cyl2(1,:),cyl2(2,:),cyl2(3,:),'+','color','b')
    plot3(cyl3(1,:),cyl3(2,:),cyl3(3,:),'o','color','g')
 
    
    c = {cyl1,cyl2,cyl3};
    top = 14*5;
    side = 21*5;
    bot = 30*5;
    
    for i=1:ngrasp
    idx(i)=size(c{1,i},2);
    end
    cyl1=c{1,find(idx==top)};
    cyl2=c{1,find(idx==bot)};
    cyl3=c{1,find(idx==side)};
    
    cyl3 = cyl3(:,cyl3(1,:)>0);
    cyl3 = cyl3(:,cyl3(2,:)<0.25);

    figure
    plot3(cyl1(1,:),cyl1(2,:),cyl1(3,:),'*','color','r')
    hold on
    plot3(cyl2(1,:),cyl2(2,:),cyl2(3,:),'+','color','b')
    plot3(cyl3(1,:),cyl3(2,:),cyl3(3,:),'o','color','g')
    
 new_cyl=[cyl1,cyl2,cyl3];
%  e = evalclusters([new_cyl(3,:)', new_cyl(5,:)', new_cyl(6,:)'],'kmeans','silhouette','klist',[1:10]);
%  
%  figure
%  plot(e)

% Training of GMM by EM algorithm, initialized by k-means clustering.
 %Creation of a mixed model
%% BIC computation

b1 = bic(cyl1,20,20,1);
b2 = bic(cyl2,20,20,1);
b3 = bic(cyl3,20,20,1);

%% Model
 nbStates_cyl = 3; %BIC: 4

 
[Priors1, Mu1, Sigma1] = EM_init_kmeans(cyl1,nbStates_cyl); %
tmp=Priors1;
[Priors1, Mu1, Sigma1, Pix, loglik] = EM(cyl1, Priors1, Mu1, Sigma1);
% BIC = -2 * loglik + log(length(cyl1))* 7
tmp-Priors1

[Priors2, Mu2, Sigma2] = EM_init_kmeans(cyl2,nbStates_cyl); %
tmp=Priors2;
[Priors2, Mu2, Sigma2, Pix, loglik] = EM(cyl2, Priors2, Mu2, Sigma2);
% BIC = -2 * loglik + log(length(cyl1))* 7
tmp-Priors2

[Priors3, Mu3, Sigma3] = EM_init_kmeans(cyl3,5); %
tmp=Priors3;
[Priors3, Mu3, Sigma3, Pix, loglik] = EM(cyl3, Priors3, Mu3, Sigma3);
% BIC = -2 * loglik + log(length(cyl3))* 7
tmp-Priors3

% figure
% plotGMM3D(Mu1([4,1:2],:), Sigma1([4,1:2],[4,1:2],:), [0 .8 0], .2,1);
% figure
% plotGMM3D(Mu1([5,1:2],:), Sigma1([5,1:2],[5,1:2],:), [0 .8 0], .2,1);
% 
% figure
% plotGMM3D(Mu2([4,1:2],:), Sigma2([4,1:2],[4,1:2],:), [0 .8 0], .2,1);
% figure
% plotGMM3D(Mu2([5,1:2],:), Sigma2([5,1:2],[5,1:2],:), [0 .8 0], .2,1);

%% GMR

% cyl_size=[3.3;23];
cyl_size=[2;17];
cyl_size=[4.5;26.5];

[expCyl1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  [cyl_size], [9:10], [1:8]);
[expCyl2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  [cyl_size], [9:10], [1:8]);
[expCyl3, expSigma3]= GMR(Priors3, Mu3, Sigma3,  [cyl_size], [9:10], [1:8]);

H=eye(4);
res=50;
x=0;
y=0;
z=0;
phi=0; theta=0; psi=0;
H=H+[zeros(3) [x y z]';[0 0 0 0]];
% H=SGRotate(H,phi,theta,psi);
obj_data.type='Cylinder';
H=eye(4);
obj_cyl=SGcylinder(H,cyl_size(2),cyl_size(1),res);
figure
SGplotSolid(obj_cyl);
hold on
axes_plot(0,0,0,0.5);

plotGMR_frame(expCyl1(:,1));
plotGMR_frame(expCyl2(:,1));
plotGMR_frame(expCyl3(:,1));
% plotGMR_frame(expCyl4(:,1));

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$x$ [cm]','interpreter','latex','fontsize',18)
ylabel('$y$ [cm]','interpreter','latex','fontsize',18)
zlabel('$z$ [cm]','interpreter','latex','fontsize',18)
axis equal

%% Pose

% c_can = [3.3;23];
% c_tube = [1.8;22.5];
% s_cyl = [2;13];
r=1:6;
h=repmat(mean([13,22.5,23]),1,length(r));
cyl_size = [r;h]; 

% h=10:2.5:25;
% r=repmat(mean([1.8,2,3.3]),1,length(h));
% cyl_size = [r;h]; 

figure

[expCyl1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  cyl_size, [9:10], [1:8]);
[expCyl2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  cyl_size, [9:10], [1:8]);
[expCyl3, expSigma3]= GMR(Priors3, Mu3, Sigma3,  cyl_size, [9:10], [1:8]);

t = linspace(0,pi)';
x = cos(t);
y = sin(t);
color = jet(size(expCyl1,2)).*0.85;

for i=1:size(expCyl1,2)
    obj_cyl=SGcylinder(H,cyl_size(2,i),cyl_size(1,i),res);
    SGplotSolid(obj_cyl,0.2,color(i,:));
    hold on
    plotGMR_frame(expCyl1(:,i),[],color(i,:));
    
    plotGMR_frame(expCyl2(:,i),[],color(i,:),0);
    plotGMR_frame(expCyl3(:,i),[],color(i,:),0);
end
axis equal
% legend({'r = 1 cm','r = 2 cm','r = 3 cm','r = 4 cm','r = 5 cm','r = 6 cm'},'FontSize',20,'Interpreter','latex')
legend({'h = 10 cm','h = 12.5 cm','h = 15 cm','h = 17.5 cm','h = 20 cm','h = 22.5 cm', 'h = 25 cm'},'FontSize',20,'Interpreter','latex')

colormap(color)

xlabel('x[cm]','interpreter','latex','fontsize',24)
ylabel('y[cm]','interpreter','latex','fontsize',24)
zlabel('z[cm]','interpreter','latex','fontsize',24)
set(gca,'FontSize',24,'TickLabelInterpreter','latex');

%% Position

l=size(expCyl1,2);
figure

for s=1:l
%     color = [1-s/l,0,1/l*s];
plotGMM3D(expCyl1([1:3],s), expSigma1([1:3],[1:3],s), color(s,:) , 0.2,1);
hold on
end
plot3(expCyl1(1,:), expCyl1(2,:), expCyl1(3,:), 'x', 'lineWidth', 5, 'color', [1 1 1])

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expCyl2([1:3],s), expSigma2([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
hold on
end

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expCyl3([1:3],s), expSigma3([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
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

[~,idx]=max([cyl_size(1,end)-cyl_size(1,1),cyl_size(2,end)-cyl_size(2,1)]);

figure

subplot(3,1,1)
% for i=1:length(sph_size)
plot(cyl_size(idx,:),rad2deg(expCyl1(4,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cyl_size(idx,:),rad2deg(expCyl2(4,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cyl_size(idx,:),rad2deg(expCyl3(4,:)),'p','color',[0 1 0],'MarkerSize',10,'LineWidth',3)

% end
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\phi [^{\circ}]$','interpreter','latex','fontsize',18)
yticklabels('auto')
% xlim([1 6])
subplot(3,1,2)
plot(cyl_size(idx,:),rad2deg(expCyl1(5,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cyl_size(idx,:),rad2deg(expCyl2(5,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cyl_size(idx,:),rad2deg(expCyl3(5,:)),'p','color',[0 1 0],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta [^{\circ}]$','interpreter','latex','fontsize',18)
% xlim([1 6])
subplot(3,1,3)
plot(cyl_size(idx,:),rad2deg(expCyl1(6,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cyl_size(idx,:),rad2deg(expCyl2(6,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cyl_size(idx,:),rad2deg(expCyl3(6,:)),'p','color',[0 1 0],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\psi [^{\circ}]$','interpreter','latex','fontsize',18)
% xlim([1 6])
% axis equal

legend({'top grasp','bottom grasp','side grasp'},'FontSize',20,'Interpreter','latex')

%% Fingers 
[~,idx]=max([cyl_size(1,end)-cyl_size(1,1),cyl_size(2,end)-cyl_size(2,1)]);

figure
subplot(2,1,1)
plot(cyl_size(idx,:),expCyl1(7,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on,
plot(cyl_size(idx,:),expCyl2(7,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cyl_size(idx,:),expCyl3(7,:),'o','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_R$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])
subplot(2,1,2)
plot(cyl_size(idx,:),expCyl1(8,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on,
plot(cyl_size(idx,:),expCyl2(8,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cyl_size(idx,:),expCyl3(8,:),'o','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_L$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])

legend({'top grasp','bottom grasp','edge grasp'},'FontSize',20,'Interpreter','latex')

