close all, clear all, clc
% Definition of the number of components used in GMM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load dataset 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('dataset.mat');
%%
ta=teleop_analysis;

% cube=3*ones(3,1);
cube=[3;3;3];
jelly =[8.5;2.5;7];
box = [18.5;5.2;13.5]



jelly=[];
for i=1:size(ta.res,2)
    if ta.res(i).obj_id==8
        if ta.res(i).cub_ori == 1 
            jelly = [jelly , [2.5;8.5;7]]; %[8.5;2.5;7]
        elseif ta.res(i).cub_ori == 2 
            jelly = [jelly , [2.5;7;8.5]]; %[7;2.5;8.5]
        elseif ta.res(i).cub_ori == 3 
            jelly = [jelly , [7.5;8.5;2.5]]; %[8.5;7.5;2.5]
        end
    end
end
            
box=[];
for i=1:size(ta.res,2)
    if ta.res(i).obj_id==9
        if ta.res(i).cub_ori == 1 
            box = [box , [5.2;18.5;13.5]]; %18.5;5.2;13.5
        elseif ta.res(i).cub_ori == 2 
            box = [box , [5.2;13.5;18.5]]; %[13.5;5.2;18.5]
        elseif ta.res(i).cub_ori == 3 
            box = [box , [13.5;18.5;5.2]]; %[18.5;13.5;5.2]
        end
    end
end


cub = [];

nbStates_cub = 3;

cub = [(obj{1,7}.dataset)',(obj{1,8}.dataset)',(obj{1,9}.dataset)'];
for i=1:size(cub,2)
    cub(4:6,i) = quat2eul(quaternion(cub(4:7,i)'))';
end
cub(7,:)=[];

cub=cub.*[ones(3,1)*100;ones(5,1);ones(3,1)]; % Position in cm
cub(9:11,:)=[];
a=size(obj{1,7}.dataset,1);
cub=vertcat((cub(1:3,:)),cub(4:8,:));
cub(9:11,:)=[ones(3,a).*cube,jelly,box]; % Relative size of Cube, Gelatin box and Box

cub_f = cub(7:8,:);
% cub(7:8,:)=[];


nbVar = size(cub,1);

cub = cluster_dataset(cub,size(cub,2),2,'cub');
 ngrasp = 4;

[Data_id, Centers] = kmeans([cub(3:4,:)',cub(6,:)'], ngrasp,'Replicates',5); 
 e = evalclusters([cub(3,:)',cub(6,:)'],'kmeans','silhouette','klist',[1:10]);
%  
 figure
 plot(e)

cub1 = cub(:,find(Data_id==1));
cub2 = cub(:,find(Data_id==2));
cub3 = cub(:,find(Data_id==3));
cub4 = cub(:,find(Data_id==4));
% cub5 = cub(:,find(Data_id==5));
% cub6 = cub(:,find(Data_id==6));
cub1 = enhance_dataset(cub1,size(cub1,2)*5,2,'cub');
cub2 = enhance_dataset(cub2,size(cub2,2)*5,2,'cub');
cub3 = enhance_dataset(cub3,size(cub3,2)*5,2,'cub');
cub4 = enhance_dataset(cub4,size(cub4,2)*5,2,'cub');

% figure
% plot3(cub1(1,:),cub1(2,:),cub1(3,:),'*','color','r')
% hold on
% plot3(cub2(1,:),cub2(2,:),cub2(3,:),'+','color','b')
% plot3(cub3(1,:),cub3(2,:),cub3(3,:),'o','color','g')
% plot3(cub4(1,:),cub4(2,:),cub4(3,:),'.','color','k')

top_y = 26*5;%32*5;
bot_y = 22*5;%27*5;
top_x = 12*5;%15*5;
bot_x = 10*5;%14*5;

c = {cub1,cub2,cub3,cub4};
for i=1:ngrasp
idx(i)=size(c{1,i},2);
end

c{1,find(idx==top_y)}(3,:)=c{1,find(idx==top_y)}(11,:)*2/3;%+c{1,find(idx==top_y)}(11,:)/3+0.5*rand(1,top_y);
c{1,find(idx==bot_y)}(3,:)=-c{1,find(idx==bot_y)}(11,:)/3;%+c{1,find(idx==bot_y)}(11,:)/3-0.5*rand(1,bot_y);
c{1,find(idx==top_x)}(3,:)=c{1,find(idx==top_x)}(11,:)*2/3;%c{1,find(idx==top_x)}(11,:)/3+0.5*rand(1,top_x);
c{1,find(idx==bot_x)}(3,:)=-c{1,find(idx==bot_x)}(11,:)/3;%+c{1,find(idx==bot_x)}(11,:)/3-0.5*rand(1,bot_x);

cub1=c{1,find(idx==top_y)};
cub2=c{1,find(idx==bot_y)};
cub3=c{1,find(idx==top_x)};
cub4=c{1,find(idx==bot_x)};
cub3 = cub3(:,cub3(1,:)>0);
cub4 = cub4(:,cub4(1,:)>0);
cub3 = enhance_dataset(cub3,size(cub3,2)*5,2,'cub');
cub4 = enhance_dataset(cub4,size(cub4,2)*5,2,'cub');

figure
plot3(cub1(1,:),cub1(2,:),cub1(3,:),'*','color','r')
hold on
plot3(cub2(1,:),cub2(2,:),cub2(3,:),'+','color','b')
plot3(cub3(1,:),cub3(2,:),cub3(3,:),'o','color','g')
plot3(cub4(1,:),cub4(2,:),cub4(3,:),'.','color','k')

%% BIC computation

b1 = bic(cub1,20,20,1);
b2 = bic(cub2,20,20,1);
b3 = bic(cub3,20,20,1);
b4 = bic(cub4,20,20,1);

%% Model
% Training of GMM by EM algorithm, initialized by k-means clustering.
 %Creation of a mixed model
nbStates_cub = 4;

[Priors1, Mu1, Sigma1] = EM_init_kmeans(cub1,nbStates_cub); %
tmp = Priors1;
[Priors1, Mu1, Sigma1, Pix, loglik] = EM(cub1, Priors1, Mu1, Sigma1);
tmp - Priors1

[Priors2, Mu2, Sigma2] = EM_init_kmeans(cub2,nbStates_cub); %
tmp = Priors2;
[Priors2, Mu2, Sigma2, Pix, loglik] = EM(cub2, Priors2, Mu2, Sigma2);
tmp - Priors2

[Priors3, Mu3, Sigma3] = EM_init_kmeans(cub3,nbStates_cub); %
tmp = Priors3;
[Priors3, Mu3, Sigma3, Pix, loglik] = EM(cub3, Priors3, Mu3, Sigma3);
tmp - Priors3

[Priors4, Mu4, Sigma4] = EM_init_kmeans(cub4,nbStates_cub); %
tmp = Priors4;
[Priors4, Mu4, Sigma4, Pix, loglik] = EM(cub4, Priors4, Mu4, Sigma4);
tmp - Priors4

%% GMR - CORREGGERE CON OFFSET

cub_size=[7;9.5;7];

[expCub1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  [cub_size], [9:11], [1:8]);
[expCub2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  [cub_size], [9:11], [1:8]);
[expCub3, expSigma3]= GMR(Priors3, Mu3, Sigma3,  [cub_size], [9:11], [1:8]); %- 2cm su x
[expCub4, expSigma4]= GMR(Priors4, Mu4, Sigma4,  [cub_size], [9:11], [1:8]); %- 2.3cm su x

 

H=eye(4);
obj_data.type='Cube';
obj_cube=SGcube(H,cub_size(1),cub_size(2),cub_size(3));

figure
SGplotSolid(obj_cube,0.15,[0 0 0.6]);

hold on
axes_plot(0,0,0,0.5);

plotGMR_frame(expCub1(:,1));
plotGMR_frame(expCub2(:,1));
plotGMR_frame(expCub3(:,1));
plotGMR_frame(expCub4(:,1));

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$x$ [cm]','interpreter','latex','fontsize',18)
ylabel('$y$ [cm]','interpreter','latex','fontsize',18)
zlabel('$z$ [cm]','interpreter','latex','fontsize',18)
axis equal

%% Pose

% cube=[3;3;3];
% jelly =[2.5;8.5;7];
% box = [5.2;18.5;13.5]

x=2:8;
y=repmat(mean([3,8.5,18.5]),1,length(x));
z=repmat(mean([3,7,13.5]),1,length(x));


cub_size = [x;y;z]; 

% h=10:2.5:25;
% r=repmat(mean([1.8,2,3.3]),1,length(h));
% cyl_size = [r;h]; 

figure

[expCub1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  cub_size, [9:11], [1:8]);
[expCub2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  cub_size, [9:11], [1:8]);
[expCub3, expSigma3]= GMR(Priors3, Mu3, Sigma3,  cub_size, [9:11], [1:8]);
[expCub4, expSigma4]= GMR(Priors4, Mu4, Sigma4,  cub_size, [9:11], [1:8]);

t = linspace(0,pi)';
xx = cos(t);
yy = sin(t);
color = jet(size(expCub1,2)).*0.85;

for i=1:size(expCub1,2)
    obj_cube=SGcube(H,cub_size(1,i),cub_size(2,i),cub_size(3,i));

    SGplotSolid(obj_cube,0.2,color(i,:));

    hold on
    plotGMR_frame(expCub1(:,i),[],color(i,:));
    
    plotGMR_frame(expCub2(:,i),[],color(i,:),0);
    plotGMR_frame(expCub3(:,i),[],color(i,:),0);
    plotGMR_frame(expCub4(:,i),[],color(i,:),0);
end
axis equal
% legend({'r = 1 cm','r = 2 cm','r = 3 cm','r = 4 cm','r = 5 cm','r = 6 cm'},'FontSize',20,'Interpreter','latex')
% legend({'h = 10 cm','h = 12.5 cm','h = 15 cm','h = 17.5 cm','h = 20 cm','h = 22.5 cm', 'h = 25 cm'},'FontSize',20,'Interpreter','latex')

colormap(color)

xlabel('x[cm]','interpreter','latex','fontsize',24)
ylabel('y[cm]','interpreter','latex','fontsize',24)
zlabel('z[cm]','interpreter','latex','fontsize',24)
set(gca,'FontSize',24,'TickLabelInterpreter','latex');

%% Position

l=size(expCub1,2);
figure

for s=1:l
%     color = [1-s/l,0,1/l*s];
plotGMM3D(expCub1([1:3],s), expSigma1([1:3],[1:3],s), color(s,:) , 0.2,1);
hold on
end
plot3(expCub1(1,:), expCub1(2,:), expCub1(3,:), 'x', 'lineWidth', 5, 'color', [1 1 1])

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expCub2([1:3],s), expSigma2([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
hold on
end

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expCub3([1:3],s), expSigma3([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
hold on
end

for s=1:l
%     color = [0.5,1/l*s,1-s/l];
plotGMM3D(expCub4([1:3],s), expSigma4([1:3],[1:3],s), 1-(color(s,:)) , 0.2,1);
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

[~,idx]=max([cub_size(1,end)-cub_size(1,1),...
             cub_size(2,end)-cub_size(2,1),...
             cub_size(3,end)-cub_size(3,1)]);


figure

subplot(3,1,1)
% for i=1:length(sph_size)
plot(cub_size(idx,:),rad2deg(expCub1(4,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cub_size(idx,:),rad2deg(expCub2(4,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),rad2deg(expCub3(4,:)),'x','color',[0.5 0.5 0.5],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),rad2deg(expCub4(4,:)),'o','color',[.5 0 0],'MarkerSize',10,'LineWidth',3)

% end
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$x$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\phi [^{\circ}]$','interpreter','latex','fontsize',18)
yticklabels('auto')
% xlim([1 6])
subplot(3,1,2)
plot(cub_size(idx,:),rad2deg(expCub1(5,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cub_size(idx,:),rad2deg(expCub2(5,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),rad2deg(expCub3(5,:)),'x','color',[0.5 0.5 0.5],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),rad2deg(expCub4(5,:)),'o','color',[.5 0 0],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$x$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta [^{\circ}]$','interpreter','latex','fontsize',18)
% xlim([1 6])
subplot(3,1,3)
plot(cub_size(idx,:),rad2deg(expCub1(6,:)),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cub_size(idx,:),rad2deg(expCub2(6,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),rad2deg(expCub3(6,:)),'x','color',[0.5 0.5 0.5],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),rad2deg(expCub4(6,:)),'o','color',[.5 0 0],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$x[cm]','interpreter','latex','fontsize',18)
ylabel('$\psi [^{\circ}]$','interpreter','latex','fontsize',18)
% xlim([1 6])
% axis equal

legend({'$top_y$','$bot_y$','$top_x$','$bot_x$'},'FontSize',20,'Interpreter','latex')

%% Fingers

[~,idx]=max([cub_size(1,end)-cub_size(1,1),...
             cub_size(2,end)-cub_size(2,1),...
             cub_size(3,end)-cub_size(3,1)]);
         
figure
subplot(2,1,1)
plot(cub_size(idx,:),expCub1(7,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cub_size(idx,:),expCub2(7,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),expCub3(7,:),'x','color',[0.5 0.5 0.5],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),expCub4(7,:),'o','color',[.5 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_R$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])
subplot(2,1,2)
plot(cub_size(idx,:),expCub1(8,:),'x','color',[0 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(cub_size(idx,:),expCub2(8,:),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),expCub3(8,:),'x','color',[0.5 0.5 0.5],'MarkerSize',10,'LineWidth',3)
plot(cub_size(idx,:),expCub4(8,:),'o','color',[.5 0 0],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta_L$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])

legend({'$top_y$','$bot_y$','$top_x$','$bot_x$'},'FontSize',20,'Interpreter','latex')