function X = ori_ctrl(X,k1,k2)
% In case it is not working on MATLAB versions after the 2017a, insert
% quaternion in the vectors for all the quat2rotm function.
for i=1:7:length(X)
traj.p{i} = 10000*X(i:i+2);
traj.o{i} = quat2rotm(X(i+3:i+6));
end
traj.p=traj.p(~cellfun('isempty',traj.p));
traj.o=traj.o(~cellfun('isempty',traj.o));

ad(1)=0;
bd(1)=0;
ai(1)=0;
bi(1)=0;
for i=1:length(traj.p)
    % proportional action
    cp = cross(traj.o{i}(:,2),[0 0 1]);
    dp = dot(traj.o{i}(:,2),[0 0 1]);
    
    a(i) = k1*sign(cp(1))*(180-acosd(dp)); %1
    
    ep = cross(traj.o{i}(:,1),[1 0 0]);
    fp = dot(traj.o{i}(:,1),[1 0 0]);

    b(i) = -k2*sign(ep(2))*acosd(fp); 
    if i>1
        % derivative action
        ad(i) = 0.1*(a(i)-a(i-1))/2; 
        bd(i) = 0.1*(b(i)-b(i-1))/2; 
        
        % integral action (NOT USED)
        ai(i) = 0*0.45*(ai(i-1) + a(i-1));  
        bi(i) = 0*0.45*(bi(i-1) + b(i-1)); 
    end
    traj.o{i+1}=traj.o{i}*inv(rotz(b(i),'deg')*rotz(bi(i),'deg')*rotz(bd(i),'deg'))*inv(rotx(a(i),'deg')*rotx(ai(i),'deg')*rotz(ad(i),'deg'));
    eulAng_ctrl(i,:)=rad2deg(rotm2eul(traj.o{i}));
end
figure
time = 0:0.01:(length(eulAng_ctrl)-1)*0.01;
plot(time,eulAng_ctrl(:,1),'b'),hold on,plot(time,eulAng_ctrl(:,2),'r'),plot(time,eulAng_ctrl(:,3),'y')
plot(time,zeros(length(eulAng_ctrl)),'r'),plot(time,-90*ones(length(eulAng_ctrl)),'y')
legend('Yaw','Roll','Pitch')

j=1;
for i=1:7:length(X)
    X(i+3:i+6) = rotm2quat(traj.o{j});
    j=j+1;
end