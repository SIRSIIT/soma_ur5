% In case it is not working on MATLAB versions after the 2017a, insert
% quaternion in the vectors for all the quat2rotm function.
%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clear all, close all
nbStatesX = 2; %Number of states in the GMM for the constraints in task space
nbStatesXf = 2; %Number of states in the GMM for the constraints in task space

data = load('data/dtw_data_div.mat'); % Load data after dynamic time warping
t=[0 -0.03 -0.272]; %PalettaFrame2EEFrame
R=quat2rotm([0.0 0.0 0.0 1.0]); %PalettaFrame2EEFrame
nbData = data.max;
nbData2 = data.max2;
fs=10; % sample frequency for first part 
fs2=2; % sample frequency for second part
nbDelta = 10310; % number of samples 
nbDeltaf = 5160;
% Data extraction to make a unique vector for GMM
for i=1:size(data.x_exp,2)
    x_exp{i}=data.x_exp{i};
    x_exp{i}(1:nbData,5:8)=[data.x_exp{i}(1:nbData,8) data.x_exp{i}(1:nbData,5)...
        data.x_exp{i}(1:nbData,6) data.x_exp{i}(1:nbData,7)];
    x_exp2{i}=data.x_exp2{i};
    x_exp2{i}(1:nbData2,5:8)=[data.x_exp2{i}(1:nbData2,8) data.x_exp2{i}(1:nbData2,5)...
        data.x_exp2{i}(1:nbData2,6) data.x_exp2{i}(1:nbData2,7)];
    ee_pos{i}=x_exp{i}(nbData,2:8);
    basTobj{i} = [quat2rotm(ee_pos{i}(4:7)) ee_pos{i}(1:3)'; 0 0 0 1];
end
%% indexing top-bottom-side
for j=1:364
    eul(:,j)=rotm2eul(basTobj{j}(1:3,1:3));
    c(j)=eul(3,j); %abs(5)<= 0.165 && abs(6)>2.6 top
    b(j)=eul(2,j); %abs(5)<= 0.165 && abs(6)<=2.6 bottom
    a(j) = eul(1,j);
end
index_top = find(abs(c)>2.6 & abs(b)<=0.165);
index_bot = find(abs(c)<=2.6 & abs(b)<=0.165);
index_side = find(abs(b)>0.165 & a>=0);
%% top
for i=1:length(index_top)
    top_x{i} = x_exp{index_top(i)};
    top_x2{i} = x_exp2{index_top(i)};
    k=1;
    for j=nbData-nbDelta+1:fs:nbData
        basTpal = [quat2rotm(top_x{i}(j,5:8)) top_x{i}(j,2:4)'; zeros(1,3) 1];
        objTpal = basTobj{index_top(i)}\basTpal;
        new_pose(k-nbData+nbDelta,:) = [top_x{i}(j,1) objTpal(1:3,4)' rotm2eul(objTpal(1:3,1:3))];
        k=k+1;
        end
    k=1;
    for j=1:fs2:nbDeltaf
        basTpal = [quat2rotm(top_x2{i}(j,5:8)) top_x2{i}(j,2:4)'; zeros(1,3) 1];
        objTpal = basTobj{index_top(i)}\basTpal;
        new_pose2(k,:) = [top_x2{i}(j,1) objTpal(1:3,4)' rotm2eul(objTpal(1:3,1:3))];
        k=k+1;
    end
    DataXt(:,nbDelta/fs*(i-1)+1:nbDelta/fs*i)=new_pose';
    DataXtf(:,nbDeltaf/fs2*(i-1)+1:nbDeltaf/fs2*i)=new_pose2';
end
%% bottom
for i=1:length(index_bot)
    bot_x{i} = x_exp{index_bot(i)};
    bot_x2{i} = x_exp2{index_bot(i)};
    k=1;
    for j=nbData-nbDelta+1:fs:nbData
        basTpal = [quat2rotm(bot_x{i}(j,5:8)) bot_x{i}(j,2:4)'; zeros(1,3) 1];
        objTpal = basTobj{index_bot(i)}\basTpal;
        new_pose(k-nbData+nbDelta,:) = [bot_x{i}(j,1) objTpal(1:3,4)' rotm2eul(objTpal(1:3,1:3))];
        k=k+1;
        end
    k=1;
    for j=1:fs2:nbDeltaf
        basTpal = [quat2rotm(bot_x2{i}(j,5:8)) bot_x2{i}(j,2:4)'; zeros(1,3) 1];
        objTpal = basTobj{index_bot(i)}\basTpal;
        new_pose2(k,:) = [bot_x2{i}(j,1) objTpal(1:3,4)' rotm2eul(objTpal(1:3,1:3))];
        k=k+1;
    end
    DataXb(:,nbDelta/fs*(i-1)+1:nbDelta/fs*i)=new_pose';
    DataXbf(:,nbDeltaf/fs2*(i-1)+1:nbDeltaf/fs2*i)=new_pose2';
end

%% side
for i=1:length(index_side)
    side_x{i} = x_exp{index_side(i)};
    side_x2{i} = x_exp2{index_side(i)};
    k=1;
    for j=nbData-nbDelta+1:fs:nbData
        basTpal = [quat2rotm(side_x{i}(j,5:8)) side_x{i}(j,2:4)'; zeros(1,3) 1];
        objTpal = basTobj{index_side(i)}\basTpal;
        new_pose(k-nbData+nbDelta,:) = [side_x{i}(j,1) objTpal(1:3,4)' rotm2eul(objTpal(1:3,1:3))];
        k=k+1;
        end
    k=1;
    for j=1:fs2:nbDeltaf
        basTpal = [quat2rotm(side_x2{i}(j,5:8)) side_x2{i}(j,2:4)'; zeros(1,3) 1];
        objTpal = basTobj{index_side(i)}\basTpal;
        new_pose2(k,:) = [side_x2{i}(j,1) objTpal(1:3,4)' rotm2eul(objTpal(1:3,1:3))];
        k=k+1;
    end
    DataXs(:,nbDelta/fs*(i-1)+1:nbDelta/fs*i)=new_pose';
    DataXsf(:,nbDeltaf/fs2*(i-1)+1:nbDeltaf/fs2*i)=new_pose2';
end

%% Training of GMM by EM algorithm, initialized by k-means clustering
nbVarX = size(DataXtf,1); 
[PriorsXtf, MuXtf, SigmaXtf] = EM_init_regularTiming(DataXtf, nbStatesX);
[PriorsXt, MuXt, SigmaXt] = EM_init_regularTiming(DataXtf, nbStatesXf);
[PriorsXbf, MuXbf, SigmaXbf] = EM_init_regularTiming(DataXbf, nbStatesX);
[PriorsXb, MuXb, SigmaXb] = EM_init_regularTiming(DataXb, nbStatesXf);
[PriorsXsf, MuXsf, SigmaXsf] = EM_init_regularTiming(DataXsf, nbStatesX);
[PriorsXs, MuXs, SigmaXs] = EM_init_regularTiming(DataXs, nbStatesXf);

[PriorsXtf, MuXtf, SigmaXtf] = EM(DataXtf, PriorsXtf, MuXtf, SigmaXtf);
[PriorsXt, MuXt, SigmaXt] = EM(DataXt, PriorsXt, MuXt, SigmaXt);
[PriorsXbf, MuXbf, SigmaXbf] = EM(DataXbf, PriorsXbf, MuXbf, SigmaXbf);
[PriorsXb, MuXb, SigmaXb] = EM(DataXb, PriorsXb, MuXb, SigmaXb);
[PriorsXsf, MuXsf, SigmaXsf] = EM(DataXsf, PriorsXsf, MuXsf, SigmaXsf);
[PriorsXs, MuXs, SigmaXs] = EM(DataXs, PriorsXs, MuXs, SigmaXs);
%% Retrieval of generalized trajectories through GMR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expXt(1,:) = [DataXt(1,1:nbDelta/fs) DataXtf(1,1:nbDeltaf/fs2)];
expXb(1,:) = [DataXb(1,1:nbDelta/fs) DataXbf(1,1:nbDeltaf/fs2)];
expXs(1,:) = [DataXs(1,1:nbDelta/fs) DataXsf(1,1:nbDeltaf/fs2)];

PriorsXt = cat(2,PriorsXt,PriorsXtf)./2;
MuXt = cat(2,MuXt,MuXtf);
SigmaXt = cat(3,SigmaXt,SigmaXtf);

PriorsXb = cat(2,PriorsXb,PriorsXbf)./2;
MuXb = cat(2,MuXb,MuXbf);
SigmaXb = cat(3,SigmaXb,SigmaXbf);

PriorsXs = cat(2,PriorsXs,PriorsXsf)./2;
MuXs = cat(2,MuXs,MuXsf);
SigmaXs = cat(3,SigmaXs,SigmaXsf);

[expXt(2:nbVarX,:), expSigmaXt] = GMR(PriorsXt,MuXt,SigmaXt,expXt(1,:),[1],[2:nbVarX]);
[expXb(2:nbVarX,:), expSigmaXb] = GMR(PriorsXb,MuXb,SigmaXb,expXb(1,:),[1],[2:nbVarX]);
[expXs(2:nbVarX,:), expSigmaXs] = GMR(PriorsXs,MuXs,SigmaXs,expXs(1,:),[1],[2:nbVarX]);
%% save for traj
save('data/rel_traj4','expXt','expSigmaXt','expXs','expSigmaXs','expXb','expSigmaXb','nbDelta','nbDeltaf','fs','fs2');

%% Plot of the GMR representation of the constraints
% load('data/rel_traj4');
% bot = load('pgp/cyl2_bot');   %bot cyl
% top = load('pgp/cyl2_top');   %top cyl
% side = load('pgp/cyl2_side');  %side cyl -45 4 
expXs2(2:7,:)=expXs(2:7,:);
expXs2(1,:)=linspace(0,15,3611);
expXt2(2:7,:)=expXt(2:7,:);
expXt2(1,:)=linspace(0,15,3611);
expXb2(2:7,:)=expXb(2:7,:);
expXb2(1,:)=linspace(0,15,3611);
basTs = [eul2rotm(side.expP(4:6)) side.expP(1:3)'/100;0 0 0 1];
basTt = [eul2rotm(top.expP(4:6)) top.expP(1:3)'/100;0 0 0 1];
basTb = [eul2rotm(bot.expP(4:6)) bot.expP(1:3)'/100;0 0 0 1];
objTw=[quat2rotm([1 0 0 0]),[0.01;0.62;0.14];zeros(1,3),1];
for i=1:length(expXb2)
    pgTs=[eul2rotm(expXs2(5:7,i)') expXs2(2:4,i);0 0 0 1];
    pgTt=[eul2rotm(expXt2(5:7,i)') expXt2(2:4,i);0 0 0 1];
    pgTb=[eul2rotm(expXb2(5:7,i)') expXb2(2:4,i);0 0 0 1];
    Ts = objTw*basTs*pgTs;
    Tt = objTw*basTt*pgTt;
    Tb = objTw*basTb*pgTb;
    expXs2(2:7,i)=[Ts(1:3,4); rotm2eul(Ts(1:3,1:3))'];
    expXt2(2:7,i)=[Tt(1:3,4); rotm2eul(Tt(1:3,1:3))'];
    expXb2(2:7,i)=[Tb(1:3,4); rotm2eul(Tb(1:3,1:3))'];
end
figure
for i=1:6
  subplot(2,3,i); hold on; box on;
  plotGMM(expXs2([1,i+1],:), expSigmaXs(i,i,:), [0 0 .5], 3);
  plotGMM(expXt2([1,i+1],:), expSigmaXt(i,i,:), [.5 0 0], 3);
  plotGMM(expXb2([1,i+1],:), expSigmaXb(i,i,:), [0 .5 0], 3);
  vline(expXb2(1,1031),'k--')
  xlabel('t');
  ylabel('$\$');
  xlim([-1 16]);
end