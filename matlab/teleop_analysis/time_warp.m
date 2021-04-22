% [x_exp,theta_exp]=time_warp()
clear all,close all,clc
data = load('data/dataset.mat'); %load obj cells
j=0;
max = 0;
max2 = 0;

% delta is the time delta between two consecutive points of the trajectory
delta=data.obj{1,1}.traj{1,1}{1,1}(2,1)-data.obj{1,1}.traj{1,1}{1,1}(1,1);

% for cycle to extract the trajectories from the data structure
for k=1:size(data.obj,2)
    object = data.obj{1,k};
    act_size = size(object.traj,1);
    for i=1:act_size
        j=j+1;
        x{j}=cell2mat(object.traj{i}(2));
        x2{j}=x{j}(object.ppg_idx(i)+1:end,:);
        x{j}=x{j}(1:object.ppg_idx(i),:);
        ee_pos{j}=object.ee_pose{i};
        if size(x{j},1)>max
            max=size(x{j},1);
            index = j;
        end
        if size(x2{j},1)>max2
            max2=size(x2{j},1);
            index2 = j;
        end
    end
end
time = 0:delta:delta*(max-1);
time2 = delta*max:delta:delta*max+delta*(max2-1);

% Application of dtw function, if i is equal to the max index we just
% include the data as it is
for i=1:size(x,2)
    if index~=i
        [dist,ix,iy]=dtw(x{index}(:,2)',x{i}(:,2)','squared',1);
        if length(iy)>max
            iy(max+1:end)=[];
        end
        x_exp{i}=[time' ones(length(iy),7).*x{i}(iy,2:8)];
    end
    if index2~=i
        [dist2,ix2,iy2]=dtw(x2{index2}(:,2)',x2{i}(:,2)','squared',1);
        if length(iy2)>max2
            iy2(max2+1:end)=[];
        end
        x_exp2{i}=[time2' ones(length(iy2),7).*x2{i}(iy2,2:8)];
    end
end
x_exp{index}=[time' ones(length(ix),7).*x{index}(ix,2:8)];
x_exp2{index2}=[time2' ones(length(ix2),7).*x2{index2}(ix2,2:8)];
save('data/dtw_data_div.mat','x_exp','x_exp2','ee_pos','max','max2');%'ft_exp','theta_exp'
