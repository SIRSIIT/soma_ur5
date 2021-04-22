close all
%% Comparison - Sphere-Cylinder-Hollow-Cube
model = {'sphere_model.mat',...
         'cyl_model.mat',...
         'hollow_model.mat',...
         'cube_model.mat'};

r_sph=1:8;
sph_size = r_sph;

r_cyl=[1:6]
h_cyl=repmat(mean([13,22.5,23]),1,length(r_cyl));
cyl_size = [r_cyl;h_cyl]; 

r_hol=3:9;
h_hol=repmat(mean([5,8,10]),1,length(r_hol));
hol_size = [r_hol;h_hol]; 

x=0.5:11;
y=repmat(14,1,length(x)); %mean([3,8.5,18.5])
z=repmat(14,1,length(x)); %mean([3,7,13.5])
cub_size = [x;y;z]; 

for i=1:length(model)
     load(model{i});
     if i == 1
     [expSph1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  sph_size, [9], [1:8]);
     [expSph2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  sph_size, [9], [1:8]);
     elseif i == 2
     [expCyl1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  cyl_size, [9:10], [1:8]);
     [expCyl2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  cyl_size, [9:10], [1:8]);
     [expCyl3, expSigma3]= GMR(Priors3, Mu3, Sigma3,  cyl_size, [9:10], [1:8]);
     elseif i == 3
     [expHol1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  hol_size, [9:10], [1:8]);
     [expHol2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  hol_size, [9:10], [1:8]);
     elseif i == 4
     [expCub1, expSigma1]= GMR(Priors1, Mu1, Sigma1,  cub_size, [9:11], [1:8]);
     [expCub2, expSigma2]= GMR(Priors2, Mu2, Sigma2,  cub_size, [9:11], [1:8]);
     [expCub3, expSigma3]= GMR(Priors3, Mu3, Sigma3,  cub_size, [9:11], [1:8]);
     [expCub4, expSigma4]= GMR(Priors4, Mu4, Sigma4,  cub_size, [9:11], [1:8]);
     end
end


V_sph = 4/3*pi*r_sph.^3;
h_cyl=mean([13,22.5,23]);
V_cyl = h_cyl*pi*r_cyl.^2;
h_hol=mean([5,8,10]);
V_hol = h_hol*pi*r_hol.^2;
V_cub = x.*y.*z;

max_Vol = max([V_sph,V_cyl,V_hol,V_cub]);
min_Vol = min([V_sph,V_cyl,V_hol,V_cub]);

V_sph=(V_sph-min_Vol)/(max_Vol-min_Vol);
V_cyl=(V_cyl-min_Vol)/(max_Vol-min_Vol);
V_hol=(V_hol-min_Vol)/(max_Vol-min_Vol);
V_cub=(V_cub-min_Vol)/(max_Vol-min_Vol);
%%  Comparison
figure
subplot(3,1,1)

% plot(V_sph,rad2deg(expSph1(4,:)),'o',int,sph_int_phi,'--','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
% hold on
% plot(V_cyl,rad2deg(expCyl1(4,:)),'x',int,cyl_int_phi,'-.','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_hol,rad2deg(expHol1(4,:)),'d',int,hol_int_phi,':.','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
% plot(V_cub,rad2deg(expCub1(4,:)),'s',int,cuby_int_phi,'.','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_cub,rad2deg(expCub3(4,:)),'s',int,cubx_int_phi,'.','color',[0.5 0 .5],'MarkerSize',10,'LineWidth',3)
plot(V_sph,rad2deg(expSph1(4,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_sph,rad2deg(expSph2(4,:)),'o','color',[.6 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl1(4,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl2(4,:)),'x','color',[0 0 .6],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl3(4,:)),'x','color',[.5 0.5 .5],'MarkerSize',10,'LineWidth',3)

plot(V_hol,rad2deg(expHol1(4,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_hol,rad2deg(expHol2(4,:)),'d','color',[0 .6 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub1(4,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub2(4,:)),'s','color',[.6 0 .6],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub3(4,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub4(4,:)),'s','color',[0 .6 .6],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\phi$ [deg]','interpreter','latex','fontsize',18)
yticklabels('auto')
% xlim([1 6])
% title('Top Grasps')
subplot(3,1,2)


% plot(V_sph,rad2deg(expSph1(5,:)),'o',int,sph_int_theta,'--','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
% hold on
% plot(V_cyl,rad2deg(expCyl1(5,:)),'x',int,cyl_int_theta,'-.','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_hol,rad2deg(expHol1(5,:)),'d',int,hol_int_theta,':.','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
% plot(V_cub,rad2deg(expCub1(5,:)),'s',int,cuby_int_theta,'.','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_cub,rad2deg(expCub3(5,:)),'s',int,cubx_int_theta,'.','color',[0.5 0 .5],'MarkerSize',10,'LineWidth',3)


plot(V_sph,rad2deg(expSph1(5,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_sph,rad2deg(expSph2(5,:)),'o','color',[.6 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl1(5,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl2(5,:)),'x','color',[0 0 .6],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl3(5,:)),'x','color',[.5 0.5 .5],'MarkerSize',10,'LineWidth',3)

plot(V_hol,rad2deg(expHol1(5,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_hol,rad2deg(expHol2(5,:)),'d','color',[0 .6 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub1(5,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub2(5,:)),'s','color',[.6 0 .6],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub3(5,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub4(5,:)),'s','color',[0 .6 .6],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])

subplot(3,1,3)

% plot(V_sph,rad2deg(expSph1(6,:)),'o',int,sph_int_psi,'--','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
% hold on
% plot(V_cyl,rad2deg(expCyl1(6,:)),'x',int,cyl_int_psi,'-.','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_hol,rad2deg(expHol1(6,:)),'d',int,hol_int_psi,':.','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
% plot(V_cub,rad2deg(expCub1(6,:)),'s',int,cuby_int_psi,'.','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_cub,rad2deg(expCub3(6,:)),'s',int,cubx_int_psi,'.','color',[0.5 0 .5],'MarkerSize',10,'LineWidth',3)

plot(V_sph,rad2deg(expSph1(6,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_sph,rad2deg(expSph2(6,:)),'o','color',[.6 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl1(6,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl2(6,:)),'x','color',[0 0 .6],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl3(6,:)),'x','color',[.5 0.5 .5],'MarkerSize',10,'LineWidth',3)

plot(V_hol,rad2deg(expHol1(6,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_hol,rad2deg(expHol2(6,:)),'d','color',[0 .6 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub1(6,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub2(6,:)),'s','color',[.6 0 .6],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub3(6,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub4(6,:)),'s','color',[0 .6 .6],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\psi$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])
% axis equal
% legend({'top grasp','bottom grasp'},'FontSize',20,'Interpreter','latex')
%%
figure
subplot(3,1,1)

plot(V_sph,rad2deg(expSph1(4,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_sph,rad2deg(expSph2(4,:)),'o','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl1(4,:)),'x','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl2(4,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl3(4,:)),'x','color',[0 1 0],'MarkerSize',10,'LineWidth',3)

plot(V_hol,rad2deg(expHol1(4,:)),'d','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_hol,rad2deg(expHol2(4,:)),'d','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub1(4,:)),'s','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub2(4,:)),'s','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub3(4,:)),'s','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub4(4,:)),'s','color',[0 0 1],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\phi$ [deg]','interpreter','latex','fontsize',18)
yticklabels('auto')

subplot(3,1,2)

plot(V_sph,rad2deg(expSph1(5,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_sph,rad2deg(expSph2(5,:)),'o','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl1(5,:)),'x','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl2(5,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl3(5,:)),'x','color',[0 1 0],'MarkerSize',10,'LineWidth',3)

plot(V_hol,rad2deg(expHol1(5,:)),'d','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_hol,rad2deg(expHol2(5,:)),'d','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub1(5,:)),'s','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub2(5,:)),'s','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub3(5,:)),'s','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub4(5,:)),'s','color',[0 0 1],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\theta$ [deg]','interpreter','latex','fontsize',18)
yticklabels('auto')

subplot(3,1,3)

plot(V_sph,rad2deg(expSph1(6,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_sph,rad2deg(expSph2(6,:)),'o','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl1(6,:)),'x','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl2(6,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cyl,rad2deg(expCyl3(6,:)),'x','color',[0 1 0],'MarkerSize',10,'LineWidth',3)

plot(V_hol,rad2deg(expHol1(6,:)),'d','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_hol,rad2deg(expHol2(6,:)),'d','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub1(6,:)),'s','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub2(6,:)),'s','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub3(6,:)),'s','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,rad2deg(expCub4(6,:)),'s','color',[0 0 1],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\psi$ [deg]','interpreter','latex','fontsize',18)
yticklabels('auto')

%% Fingers - Comparison
figure
subplot(2,2,1)

plot(V_sph,(expSph1(7,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on

plot(V_cyl,(expCyl1(7,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_cyl,(expCyl3(7,:)),'x','color',[.5 .5 .5],'MarkerSize',10,'LineWidth',3)
plot(V_hol,(expHol1(7,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub1(7,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub3(7,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\gamma_R$ [deg]','interpreter','latex','fontsize',18)
yticklabels('auto')
% xlim([1 6])
subplot(2,2,2)
plot(V_sph,(expSph1(8,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_cyl,(expCyl1(8,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
% plot(V_cyl,(expCyl3(8,:)),'x','color',[.5 .5 .5],'MarkerSize',10,'LineWidth',3)
plot(V_hol,(expHol1(8,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub1(8,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub3(8,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\gamma_L$ [deg]','interpreter','latex','fontsize',18)
% xlim([1 6])
% legend({'top grasp','bottom grasp'},'FontSize',20,'Interpreter','latex')

% figure
subplot(2,2,3)

plot(V_sph,(expSph2(7,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_cyl,(expCyl2(7,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_hol,(expHol2(7,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub4(7,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub2(7,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)
set(gca,'FontSize',20,'TickLabelInterpreter','latex');
% xlabel('$r$ [cm]','interpreter','latex','fontsize',18)
ylabel('$\gamma_R$ [deg]','interpreter','latex','fontsize',18)
yticklabels('auto')
% xlim([1 6])
subplot(2,2,4)
plot(V_sph,(expSph2(8,:)),'o','color',[1 0 0],'MarkerSize',10,'LineWidth',3)
hold on
plot(V_cyl,(expCyl2(8,:)),'x','color',[0 0 1],'MarkerSize',10,'LineWidth',3)
plot(V_hol,(expHol2(8,:)),'d','color',[0 1 0],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub4(8,:)),'s','color',[0 1 1],'MarkerSize',10,'LineWidth',3)
plot(V_cub,(expCub2(8,:)),'s','color',[1 0 1],'MarkerSize',10,'LineWidth',3)

set(gca,'FontSize',20,'TickLabelInterpreter','latex');
xlabel('Normalized volume','interpreter','latex','fontsize',18)
ylabel('$\gamma_L$ [deg]','interpreter','latex','fontsize',18)