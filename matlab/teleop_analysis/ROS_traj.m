% In case it is not working on MATLAB versions after the 2017a, insert
% quaternion in the vectors for all the quat2rotm function.
%% Receive the pre-grasp pose from ROS
clear all, close all, clc
rosinit('172.31.1.1',...
'NodeHost','172.31.1.5',...
'NodeName', '/obj_cluster');

% load('pgp/sph_bot');   %bot sph
% load('pgp/sph_top');   %top sph
% load('pgp/hol_bot');   %bot hol
% load('pgp/hol_top');   %top hol
% load('pgp/cub_top_y'); %top y cube 
% load('pgp/cub_bot_y'); %bot y cube
% load('pgp/cub_top_x'); %top x cube -90 -3.5 0 -5
% load('pgp/cub_bot_x'); %bot x cube -90 -2.3 1.23 -2
% load('pgp/cyl_bot');   %bot cyl
% load('pgp/cyl_top');   %top cyl
% load('pgp/cyl_side');  %side cyl
% load('pgp/sph2_bot');   %bot sph
% load('pgp/sph2_top');   %top sph
% load('pgp/hol2_bot');   %bot hol
% load('pgp/hol2_top');   %top hol
% load('pgp/cub2_top_y'); %top y cube 4 3 -5
% load('pgp/cub2_bot_y'); %bot y cube -90 -3.5 0 -5
% load('pgp/cub2_top_x'); %top x cube 
% load('pgp/cub2_bot_x'); %bot x cube -90 -2.3 1.23 -2
% load('pgp/cyl2_bot');   %bot cyl
% load('pgp/cyl2_top');   %top cyl
% load('pgp/cyl2_side');  %side cyl -45 4 
% expP(4)=0;
%% Input from keyboard to initialize the files
% expP(1)=input('x -> ');
% expP(2)=input('y -> ');
% expP(3)=input('z -> ');
% expP(4)=input('alfa -> ');
% expP(5)=input('beta -> ');
% expP(6)=input('gamm -> ');
% expP(7)=input('sx -> ');
% expP(8)=input('dx -> ');
% name = strcat('pgp\',input('insert name of the pre-grasp: ','s'));
% save(name, 'expP'); %shape-pg-dir
%%
theta_pub = rospublisher('/cmd_fingers', 'std_msgs/Int16MultiArray');
msg = rosmessage(theta_pub);
msg.Data = [0 0];
send(theta_pub,msg);


obj_pose_sub = rossubscriber('/poses_boxes'); 
obj_pose =  receive(obj_pose_sub);
objP = obj_pose.Poses.Position;
objO = obj_pose.Poses.Orientation;
objO = [objO.W objO.X objO.Y objO.Z];

Tobj_w=[quat2rotm([1 0 0 0])*rotz(-15,'deg'),[objP.X;objP.Y;objP.Z];zeros(1,3),1]; % [1 0 0 0] 
offset = [1;0;-5;0;0;0]; % offset wrt the base frame
pgp=expP(1:6)'+offset;
Tsc_obj = [eul2rotm(pgp(4:6)'),[pgp(1)/100;pgp(2)/100;pgp(3)/100]; zeros(1,3),1];
Tsc_w = Tobj_w*Tsc_obj;
Tsc_w_off = Tsc_w*[eye(3),[0;0;0];0 0 0 1]; % offset wrt the Paletta Frame
pgp= [Tsc_w_off(1:3,4)' rotm2quat(Tsc_w_off(1:3,1:3))]
save('Data/pgp','pgp');
rosshutdown;
%% Reproduction of the trajectory wrt the base
load('Data/rel_traj3')
load('Data/pgp')
PG=[quat2rotm(pgp(4:7)) pgp(1:3)'; 0 0 0 1]; % transform from object rf to base rf
X=zeros(7,nbDelta/fs+nbDeltaf/fs2);
% input to be inserted: t=top s=side b=bottom
op=input('Pre-grasp: ','s');
if op=='t'
    fprintf('top grasp... \n')
    for i=nbDelta/fs:-1:1
        basTpal = PG*[eul2rotm(expXt(5:7,i)') expXt(2:4,i); 0 0 0 1];
        X(:,i) = [basTpal(1:3,4); rotm2quat(basTpal(1:3,1:3))'];
    end
    for i=nbDelta/fs+1:nbDelta/fs+nbDeltaf/fs2
        basTpal = PG*[eul2rotm(expXt(5:7,i)') expXt(2:4,i); 0 0 0 1];
        X(:,i) = [basTpal(1:3,4); rotm2quat(basTpal(1:3,1:3))'];
    end
elseif op=='s' 
    fprintf('side grasp... \n')
    for i=nbDelta/fs:-1:1
        basTpal = PG*[eul2rotm(expXs(5:7,i)') expXs(2:4,i); 0 0 0 1];
        X(:,i) = [basTpal(1:3,4); rotm2quat(basTpal(1:3,1:3))'];
    end
    for i=nbDelta/fs+1:nbDelta/fs+nbDeltaf/fs2
        basTpal = PG*[eul2rotm(expXs(5:7,i)') expXs(2:4,i); 0 0 0 1];
        X(:,i) = [basTpal(1:3,4); rotm2quat(basTpal(1:3,1:3))'];
    end
elseif op=='b'
    fprintf('bottom grasp... \n')
    for i=nbDelta/fs:-1:1
        basTpal = PG*[eul2rotm(expXb(5:7,i)') expXb(2:4,i); 0 0 0 1];
        X(:,i) = [basTpal(1:3,4); rotm2quat(basTpal(1:3,1:3))'];
    end
    for i=nbDelta/fs+1:nbDelta/fs+nbDeltaf/fs2
        basTpal = PG*[eul2rotm(expXb(5:7,i)') expXb(2:4,i); 0 0 0 1];
        X(:,i) = [basTpal(1:3,4); rotm2quat(basTpal(1:3,1:3))'];
    end
end
X_new = reshape(X,1,size(X,2)*size(X,1));
save('Data/traj_compl','X_new')

%% send trajectory on ROS
f = load('Data/traj_compl');
close all
% control action to position the EE in a certain pose using ori_ctrl
Xf = f.X_new(6441:6441+1616+490*10); 
X2 = [f.X_new(6441+1617+490*10:6441+1616+490*15) ori_ctrl(f.X_new(6441+1617+490*15:25277-490*2),0.002,0.002)];
X2wc = [f.X_new(6441+1617:6441+1616+490*5) f.X_new(6441+1617+490*5:25277-490*5)];
Xf = [Xf X2];
Xfwc = [Xf X2wc];
save('Data/traj_plot','Xf','Xfwc')
for i=1:1:length(Xf)/7
    Xf(4+(i-1)*7:7*(i-1)+7)=[Xf((i-1)*7+5:(i-1)*7+7) Xf((i-1)*7+4)];
end
close all
%% pub
rosinit('172.31.1.1',...
'NodeHost','172.31.1.5',...
'NodeName', '/obj_cluster');
pub = rospublisher('/abs_traj', 'std_msgs/Float32MultiArray');
msg2 = rosmessage(pub);
msg2.Data = Xf;
send(pub,msg2);
pause(1);
rosshutdown;