pos = pose(:,2:4);
ori = pose(:,5:8);
f = ft(:,2:4);
t = ft(:,5:7);
time = pose(:,1)-pose(1,1);
%%
ff =[];
tt = [];

for i=1:size(ori,1)
    q = quaternion(ori(i,4),ori(i,1),ori(i,2),ori(i,3));
    R = quat2rotm(q);
    ff(i,:) = 5*(R*f(i,:)')';
    tt(i,:) = 5*(R*t(i,:)')';
end
%%
figure,
plot(f(:,2)),hold on,plot(ff(:,3))
