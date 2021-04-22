function out=bic(in,nStates,iteration,p)

if nargin==3
    p=0;
end

numObs = length(in);

for k=1:nStates 
    for i=1:iteration
[Priors, Mu, Sigma] = EM_init_kmeans(in, k);
[Priors, Mu, Sigma, ~, loglik] = EM(in, Priors, Mu, Sigma);
L(i,k) = loglik;
    end
AIC(:,k) = -2*L(:,k) +2*k;
BIC(:,k) = -2*(L(:,k)) + k*log(numObs);
end
    
    
     

out=BIC;

if p==1
% figure
% plot(mean(AIC,1),'r','LineWidth',3), hold on 
% plot(min(AIC),'b','LineWidth',3)
% plot(max(AIC),'b','LineWidth',3)
% xlabel("Number of Gaussians",'interpreter','latex','fontsize',18)
% ylabel("AIC",'interpreter','latex','fontsize',18)
% % xlim([0 27])
% set(gca,'FontSize',18,'TickLabelInterpreter','latex');

%  plot(AIC+std_AIC,'b'),plot(AIC-std_AIC,'b')
%  figure
%  plot(mean(BIC),'r'), hold on, plot(BIC+std(BIC),'b'),plot(BIC-std(BIC),'b')
figure
plot(mean(BIC,1),'r','LineWidth',3), hold on
plot(min(BIC),'b','LineWidth',3)
plot(max(BIC),'b','LineWidth',3)
xlabel("Number of Gaussians",'interpreter','latex','fontsize',18)
ylabel("BIC",'interpreter','latex','fontsize',18)
% xlim([0 27])
set(gca,'FontSize',18,'TickLabelInterpreter','latex');
   figure
 plot(mean(BIC),'r'), hold on, plot(mean(BIC)-std(BIC),'b'),plot(mean(BIC)+std(BIC),'b')
end