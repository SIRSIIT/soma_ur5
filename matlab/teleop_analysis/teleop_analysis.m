classdef teleop_analysis
    properties
        res
        props
        succ
        objs
    end
    
    %Notes: forces were plotted from exp 30 17-07
    %obj traj from 35 (slide_2019-04-12-17-52-18.bag)
    
    methods
        
        function obj=teleop_analysis(filename)
            if(nargin==0)
                filename='scoop_teleop_Feb5.txt'
%                 filename='scoop_teleop.txt'
            end
            %obj.res
            set(groot,'defaulttextinterpreter','latex');
            set(groot, 'defaultAxesTickLabelInterpreter','latex');
            set(groot, 'defaultLegendInterpreter','latex');
            
            obj.objs={'Apple','Peach','Strawberry','Chips can','Candy tube', 'Small cylinder','Cube','Gelatin box','Box','Mug','Bowl','Funnel'};
           
            
            obj.res=obj.load_file(filename);
            obj.props=obj.load_properties();
%             obj.succ=obj.get_success_rate();
            
            
            %plot_succ_rates(res,properties);
            
        end
        
      function [obj] = dataset_generation(self) %exp,closure
%               pose=self.load_txt_files('.ft','data/',{filename});
              
            obj_id=[self.res.obj_id];
            objects=[self.res.object];
            ft_base=[self.res.ftbase];
            filename={self.res.filename};
            obj_ori={self.res.obj_ori};
            
            for n=1:max(obj_id)
                for i=1:length(self.res)
                        if obj_id(i) == n
%                             if strcmp(filename{i},'scoop_teleop_2021-02-15-17-20-44.bag')
%                                 a=1;
%                             end
                        js=self.load_txt_files('.js','data2/',{filename{i}}); %Joint States
                        ee_pose=self.load_txt_files('.pose','data2/',{filename{i}}); %EE Pose
                        ee_ft2=self.load_txt_files('.ft','data2/',{filename{i}}); %EE Force/Torque
                        box_pose2=self.load_txt_files('.box_pose','data2/',{filename{i}}); %Box Pose
    %                     cmd_gripper=self.load_txt_files('.cmd_gripper','data/',{filename{i}}); %Cmd Gripper
%                         cmd_gripper=self.load_txt_files('.dbg_cmd_gripper','data2/',{filename{i}}); %DBG Cmd Gripper
                        motor_status=self.load_txt_files('.motor_status','data2/',{filename{i}}); %DBG Cmd Gripper
%                         cluster=self.load_cluster('.cluster','data/',{filename{i}}); %Cluster
                        cmd_fingers=self.load_txt_files('.cmd_fingers','data2/',{filename{i}}); %Cmd Fingers
                        cluster=-1;
%                         if isempty(cmd_gripper{1})
%                             cmd_gripper=self.load_txt_files('.cmd_gripper','data/',{filename{i}}); %Cmd Gripper
%                         end
%                         h = floor(size(js{1},1)/size(box_pose2{1},1)); 
%                         res = size(js{1},1) - h*size(box_pose2{1},1);
% 
%                         box_pose=[]; 
%                         for j=1:size(box_pose2{1},1)
%                             % Adding samples to box_pose
%                             box_pose = [box_pose ; [box_pose2{1}(j,2:8); ones(h-1,1)*box_pose2{1}(j,2:8)]]; 
%                         end
                        if size(box_pose2{1})>2
%                             box_pose = median(box_pose2{1}(1:ceil(size(box_pose2{1},1)/2),2:8));
                            box_dim = median(box_pose2{1}(1:ceil(size(box_pose2{1},1)/2),9:11));
                        else
%                             box_pose = box_pose2{1}(1:ceil(size(box_pose2{1},1)/2),2:8);
                            box_dim = box_pose2{1}(1:ceil(size(box_pose2{1},1)/2),9:11);
                        end
                        
                        if ft_base(i)==0
                            % Transform F/T data from Paletta to Base Frame
                            ee_ft{1} = paletta2baseframe(self,ee_pose,ee_ft2);
                        else
                            ee_ft{1}=ee_ft2{1};
                        end
                        
%                         d = abs(length(ee_ft{1,1})-length(js{1,1})); 
%                         if d>0
%                             M=max(size(ee_ft{1,1},1),size(js{1,1},1));
%                             if M-size(ee_ft{1},1)==0
%                                 ee_ft{1,1}=ee_ft{1,1}(1:end-d,:);
%                                 box_pose = box_pose(1:end-d,:);
%                             else
%                                 js{1,1}=js{1,1}(1:end-d,:);
%                                 box_pose = box_pose(1:end-d,:);
%                             end
%                         end
% 
%                         d = abs(length(ee_pose{1,1})-length(js{1,1})); 
%                         if d>0
%                             M=max(size(ee_pose{1,1},1),size(js{1,1},1));
%                             if M-size(ee_pose{1},1)==0
%                                 ee_pose{1,1}=ee_pose{1,1}(1:end-d,:);
%     %                         else
%     %                             js{1,1}=js{1,1}(1:end-d,:);
%                             end
%                         end
                        
                        if isempty(cmd_fingers{1,1})==1
                            theta=[0 0];
                        else
                            theta=cmd_fingers{1,1}(end,end-1:end).*[-1 1];
                        end
                        time=js{1}(:,1)-js{1}(1,1);
                        
                        motor_status{1,1}(:,1)=motor_status{1,1}(:,1)-js{1}(1,1);
                        motor_status{1,1}=motor_status{1,1}(:,1:2);
                        
%                         box_pose2{1,1}(:,1)=box_pose2{1,1}(:,1)-box_pose2{1}(1,1);
%                         box_pose = [box_pose ; ones(res,1)*box_pose(end,:)];
    %                     exp{i} = [time, js{1}(:,2:end), box_pose,...
    %                               ee_pose{1}(:,2:end), ee_ft{1}];                     
%                         if isempty(ee_ft{1})
                            aux = min([length(time),length(js{1}),length(ee_pose{1})]);
%                             filename{i}
%                             [length(time),length(js{1}),length(ee_pose{1})]
%                             length(js{1})- length(ee_pose{1})
                            obj{n}.exp{i} = [time(1:aux), js{1}(1:aux,2:end), ee_pose{1}(1:aux,2:end)]; %cs ,box_pose(1:aux,:)
%                         else
%                             aux = min([length(time),length(js{1}),length(ee_pose{1}),...
%                                    length(ee_ft{1})]);
%                             obj{n}.exp{i} = [time(1:aux), js{1}(1:aux,2:end), ee_pose{1}(1:aux,2:end),...
%                                          ee_ft{1}(1:aux,2:end)]; %cs ,box_pose(1:aux,:)
%                         end
%                         cs=vertcat(cmd_gripper{1}(:,2),...
%                         cmd_gripper{1}(end,2)*ones(aux-length(cmd_gripper{1}),1));
                        
                        obj{n}.clst{i} = cluster;
                        obj{n}.name = self.objs{n};
                        obj{n}.filename{i} = self.res(i).filename;
                        obj{n}.theta{i}=theta;
                        obj{n}.obj_dim{i}=box_dim;
%                         box_pose = median(box_pose(1:end/2,:));
                        obj{n}.obj_pose{i}=box_pose2{1};
                        obj{n}.ft{i}=ee_ft;
                        obj{n}.obj_ori{i}=obj_ori{i};
                        obj{n}.motor_status{i}=motor_status;
%                         obj{n}.cs{i}=vertcat(cmd_gripper{1}(:,2),...
%                         cmd_gripper{1}(end,2)*ones(length(obj{n}.exp{i})-length(cmd_gripper{1}),1));
%                         closure{i} = [time(1:aux),obj{n}.cs];
                        %16*(cmd_gripper{1}(:,1)-cmd_gripper{1}(1,1)), 
                        obj{n}.exp=obj{n}.exp(~cellfun('isempty',obj{n}.exp));
                        obj{n}.clst=obj{n}.clst(~cellfun('isempty',obj{n}.clst));
                        obj{n}.filename=obj{n}.filename(~cellfun('isempty',obj{n}.filename));
%                         obj{n}.cs=obj{n}.cs(~cellfun('isempty',obj{n}.cs));
                        obj{n}.theta=obj{n}.theta(~cellfun('isempty',obj{n}.theta));
                        obj{n}.obj_dim=obj{n}.obj_dim(~cellfun('isempty',obj{n}.obj_dim));
                        obj{n}.obj_pose=obj{n}.obj_pose(~cellfun('isempty',obj{n}.obj_pose));
                        obj{n}.ft=obj{n}.ft(~cellfun('isempty',obj{n}.ft));
                        obj{n}.obj_ori=obj{n}.obj_ori(~cellfun('isempty',obj{n}.obj_ori));
                        obj{n}.motor_status=obj{n}.motor_status(~cellfun('isempty',obj{n}.motor_status));
                end
            end
            end
      end
      
      function out=load_cluster(self,type,path_to,files)
            msg = rosmessage('sensor_msgs/PointCloud2');
            msg_x = rosmessage('sensor_msgs/PointField');
            msg_y = rosmessage('sensor_msgs/PointField');
            msg_z = rosmessage('sensor_msgs/PointField');
            for i=1:length(files)
                % Breakpoint strncmp(files{i}, FILENAME, 30)
                disp([files{i},type,'.txt'])
                fid=fopen([path_to,files{i},type,'.txt']);
                %out{i}=[1e-9*(dd(:,1)-dd(1,1)),dd(:,2:4),(dd(:,1)-dd(1,1))/(dd(end,1)-dd(1,1)),dd(:,1)];
               out=fgetl(fid);
            end
%             while ischar(out)
                if ~isempty(out) >0 && ischar(out)
                    res_cell=strsplit(out,',');
                    msg.Header.Seq = str2double(res_cell{1,2});
                    msg.Header.Stamp.Sec = str2double(res_cell{1,1});
                    msg.Header.Stamp.Nsec = str2double(res_cell{1,3});
    %                 msg.Header.FrameId = "";
                    msg.Height = str2double(res_cell{1,4});
                    msg.Width = str2double(res_cell{1,5});
                    msg_x.Name = res_cell{1,6};
                    msg_x.Offset = str2double(res_cell{1,7});
                    msg_x.Datatype = str2double(res_cell{1,8});
                    msg_x.Count = str2double(res_cell{1,9});
                    msg_y.Name = res_cell{1,10};
                    msg_y.Offset = str2double(res_cell{1,11});
                    msg_y.Datatype = str2double(res_cell{1,12});
                    msg_y.Count = str2double(res_cell{1,13});
                    msg_z.Name = res_cell{1,14};
                    msg_z.Offset = str2double(res_cell{1,15});
                    msg_z.Datatype = str2double(res_cell{1,16});
                    msg_z.Count = str2double(res_cell{1,17});
                    msg.Fields = [msg_x;msg_y;msg_z];
                    msg.IsBigendian = false;
                    msg.IsDense = true;
                    msg.PointStep = str2double(res_cell{1,19});
                    msg.RowStep = str2double(res_cell{1,20});
                    for i=21:length(res_cell)
                        msg.Data(i-20) = str2double(res_cell{1,i});
                    end
                    out = self.ptCloud2CH(msg);   
                end
      end
      
      function out=ptCloud2CH(self,cluster)
        
        %RF: reconstruction factor (0.15-0.3) - banana=0.15
        pcobj = pointCloud(readXYZ(cluster));
        gridStep = 0.01;
        objA = pcdownsample(pcobj,'gridAverage',gridStep);
        xyzPoints = double(objA.Location);
        xyzPoints = [double(objA.Location),ones(size(xyzPoints,1),1)];
        % KinectFrame to World (rosrun tf tf_echo world kinect2_link)
        R = quat2rotm([0.332, -0.561, 0.672, -0.352]); 
        t = [1.2 0.7 1.21];
        T = [R,t';zeros(1,3),1];

        xyzPoints_w = (T*xyzPoints')';
%         obj_center_world = [median(xyzPoints_world(:,1));median(xyzPoints_world(:,2));median(xyzPoints_world(:,3))];

%         R_table = quat2rotm([tbl.Pose.Orientation.W, tbl.Pose.Orientation.X, tbl.Pose.Orientation.Y, tbl.Pose.Orientation.Z]);
%         t0 = [0 0 0];
%         T_table = [R_table,t0';zeros(1,3),1];
%         % T_table = T1*T;
%         Rz=[-1 0 0;0 -1 0;0 0 1];
%         Tz = [Rz,zeros(3,1);zeros(1,3),1];

    % Offset removal

        xyzPoints_w = xyzPoints_w + [-min(xyzPoints_w(:,1)),-min(xyzPoints_w(:,2)),-min(xyzPoints_w(:,3)),0];
        xyzPoints_w = xyzPoints_w + [-0.5*max(xyzPoints_w(:,1)),0,0,0];

        % Object "reconstruction"

        % xyzPoints = obj_rec(xyzPoints,RF);

        k=MyCrustOpen(xyzPoints_w(:,1:3));

        object.vertices = xyzPoints_w;
        object.vertices(:,4)=[];
        object.center = [(max(object.vertices(:,1))+min(object.vertices(:,1)))/2;(max(object.vertices(:,2))+min(object.vertices(:,2)))/2;(max(object.vertices(:,3))+min(object.vertices(:,3)))/2];
        % object.center = [mean(object.vertices(:,1));mean(object.vertices(:,2));mean(object.vertices(:,3))];
        % object.center = [median(object.vertices(:,1));median(object.vertices(:,2));median(object.vertices(:,3))];

        xyzPoints_w = xyzPoints_w + [-object.center(1),-object.center(2),-object.center(3),0];

        % Object structure
        object.faces = k;
        object.vertices = xyzPoints_w;
        object.vertices(:,4)=[];
        object.type = "sph";
        object.center = [(max(object.vertices(:,1))+min(object.vertices(:,1)))/2;(max(object.vertices(:,2))+min(object.vertices(:,2)))/2;(max(object.vertices(:,3))+min(object.vertices(:,3)))/2];
        out = object;
%         figure
%         trisurf(object.faces,...
%                 object.vertices(:,1),...
%                 object.vertices(:,2),...
%                 object.vertices(:,3),'facecolor','c','edgecolor','b');
      end
              

%                     experiment.obj_id=find(strcmp(self.objs,res_cell{2}));
%                     if(isempty(experiment.obj_id))
%                         continue
%                     end
%                     experiment.object=string(res_cell{2});
%                     experiment.filename=res_cell{1};
%                     obj=0;
%                     
%                     if string(res_cell{3})=='1'
%                         experiment.success=1;
%                     elseif string(res_cell{3})=='0'
%                         experiment.success=0;
%                     else
%                         error(['Error loading file: str: ' out])
%                     end
%                     
%                     if string(res_cell{5})=='1'
%                         experiment.ftbase=1;
%                     elseif string(res_cell{5})=='0'
%                         experiment.ftbase=0;
%                     else
%                         error(['Error loading file: str: ' out])
%                     end
%                     results=[results,experiment];
      
%     obj{i}.CH =obj_w{i}.clst;
% end
      function obj = world2obj(self,obj_w)
          for i=1:length(obj_w)  
            for j=1:length(obj_w{i}.exp)
%             [~,idx]=min(obj_w{i}.exp{j}(:,21)); % Find pre-grasp pose - OLD
%               idx=self.pre_grasp_pose(obj_w{i}.exp{j}(:,21),obj_w{i}.filename{j}); % Find pre-grasp pose
              idx=self.pre_grasp_pose(obj_w{i}.exp{j}(:,1),obj_w{i}.motor_status{j}{:},obj_w{i}.filename{j}); % Find pre-grasp pose
%               obj_pose=(obj_w{i}.exp{j}(1,22:28));
              obj_pose2=(obj_w{i}.obj_pose{j});
              ee_pose=(obj_w{i}.exp{j}(idx,8:14));
%               self.plot_world_ee_obj(ee_pose,obj_pose,i);
              for k=1:round(size(obj_pose2,1)/2)
%                   if obj_pose2(k,8)>0.94
                    d(k,:)=norm(ee_pose(1:3)-obj_pose2(k,2:4));
%                   end
              end
              [~,idxx]=min(d);
              d=[];
              obj_pose=obj_pose2(idxx,2:8);
              R_obj=quat2rotm(quaternion([obj_pose(end),obj_pose(4:6)]));
              R_ee=quat2rotm(quaternion([ee_pose(end),ee_pose(4:6)]));
              t=[obj_pose(1:3),zeros(1,4)];
              new_ee_pose=ee_pose-t;
              if obj_w{i}.obj_ori{j}==1
%                 new_ee_pose=ee_pose-t;
                new_ee_pose(4:end)=rotm2quat(inv(R_obj)*R_ee); %inv(R_obj)*
              else
%                 new_ee_pose=[(rotx(-90)*(new_ee_pose(1:3))')' zeros(1,4)];% - [(roty(-90)*t(1:3)')' 0 0 0 0];
%                 new_ee_pose(4:end)=rotm2quat(roty(90)*R_ee*inv(R_obj));
                  new_ee_pose(4:end)=rotm2quat(rotx(90)*inv(R_obj)*R_ee); %inv(R_obj)*
%                   new_ee_pose(4:end)=rotm2quat(inv(R_obj)*R_ee);
              end
              obj{i}.ee_pose{j,:}=new_ee_pose;
              obj{i}.theta{j}=obj_w{i}.theta{j}';
              obj{i}.dataset(j,:)=[new_ee_pose,obj_w{i}.theta{j},obj_w{i}.obj_dim{j}];
              obj{i}.ppg_idx(j)=idx;
              traj_q = (obj_w{i}.exp{j}(1:end,1:7));    %1->idx
              traj_t = [(obj_w{i}.exp{j}(1:end,1)),...  %1->idx
                        (obj_w{i}.exp{j}(1:end,8:14))]; %1->idx
              obj{i}.traj{j,:} = {traj_q,traj_t,obj_w{i}.ft{j}{:}};
            end
            obj{i}.name=obj_w{i}.name;
            obj{i}.filename=obj_w{i}.filename';
            obj{i}.CH=obj_w{i}.clst;
            obj{i}.obj_pose=[0 0 0 1 0 0 0];
          end
      end
      
      function plot_world_ee_obj(self,ee_pose,obj_pose,i)
          figure(i)
          R_ee=quat2rotm(quaternion([ee_pose(end),ee_pose(4:6)]))*0.1;
            R_ee=R_ee*0.1;
            quiver3(ee_pose(1),ee_pose(2),ee_pose(3),...
                R_ee(1,1),R_ee(2,1),R_ee(3,1),...
                'r','filled','LineWidth',3)
            hold on
            quiver3(ee_pose(1),ee_pose(2),ee_pose(3),...
                R_ee(1,2),R_ee(2,2),R_ee(3,2),...
                'g','filled','LineWidth',3)
            quiver3(ee_pose(1),ee_pose(2),ee_pose(3),...
                R_ee(1,3),R_ee(2,3),R_ee(3,3),...
                'b','filled','LineWidth',3)
            plot3(obj_pose(1),obj_pose(1),obj_pose(3),'r*','MarkerSize',10)
      end
      function idx=pre_grasp_pose(self,time,grip,filename)
%           close all
filename
          if strcmp(filename,'scoop_teleop_2021-02-22-11-14-23.bag')
              a=1;
          end
          grip(:,2)=smooth(grip(:,2),30);
%           grip(find(grip(:,2)>2500),2)=grip(find(grip(:,2)>2500)-1,2);
%           grip(find(grip(:,2)>2500),2)=grip(find(grip(:,2)>2500)-1,2);
          grip(find(grip(:,2)>2500),2)=2499;
          grip(:,2)=smooth(grip(:,2),60);
%           figure,plot(grip(:,1),grip(:,2))
          
          for i=50:length(grip)
            d(i) = (grip(i,2) - grip(i-49,2))/50;
          end
%           [~,index]=min(d);
          index = find(d<-5,1,'first');
          idx = find(abs(time-grip(index,1))<=9*10^-2);
          
          idx=idx(1);

%           
%           figure, plot(grip),hold on, plot(d), plot(index,0,'r*')
      end
      
      function plot_obj_ee_pose(self,obj)
            for i=1:length(obj)  
                figure(i)
%                 for j=1:length(obj{i}.CH)
%                     if isstruct(obj{i}.CH{j}) ==1
%                         idx = j;
% %                         k(i)=j;
%                         break;
%                     end
%                 end
%                 trisurf(obj{i}.CH{1,idx}.faces,...
%                         obj{i}.CH{1,idx}.vertices(:,1),...
%                         obj{i}.CH{1,idx}.vertices(:,2),...
%                         obj{i}.CH{1,idx}.vertices(:,3),'facecolor','c','edgecolor','b');
               hold on
                R=quat2rotm(obj{i}.obj_pose(4:7))*10^-1;
                    quiver3(0,0,0,...
                            R(1,1),R(2,1),R(3,1),...
                            'r','filled','LineWidth',6)
                hold on
                    quiver3(0,0,0,...
                            R(1,2),R(2,2),R(3,2),...
                            'g','filled','LineWidth',6)
                    quiver3(0,0,0,...
                            R(1,3),R(2,3),R(3,3),...
                            'b','filled','LineWidth',6)
                for j=1:length(obj{i}.ee_pose)    
                    hold on
%                     R=quat2rotm([obj{i}.ee_pose{j}(4:7)])*10^-1;
%                     quiver3(obj{i}.ee_pose{j}(1),obj{i}.ee_pose{j}(2),obj{i}.ee_pose{j}(3),...
%                             R(1,1),R(2,1),R(3,1),...
%                             'r','filled','LineWidth',3)
%                     hold on
%                     quiver3(obj{i}.ee_pose{j}(1),obj{i}.ee_pose{j}(2),obj{i}.ee_pose{j}(3),...
%                             R(1,2),R(2,2),R(3,2),...
%                             'g','filled','LineWidth',3)
%                     quiver3(obj{i}.ee_pose{j}(1),obj{i}.ee_pose{j}(2),obj{i}.ee_pose{j}(3),...
%                             R(1,3),R(2,3),R(3,3),...
%                             'b','filled','LineWidth',3)
                    ori = [obj{i}.ee_pose{j}(1),0,0;
                           0,obj{i}.ee_pose{j}(2),0;
                           0,0,obj{i}.ee_pose{j}(3)];
                    dir = quat2rotm([obj{i}.ee_pose{j}(4:7)])*10^-1;
                    quiver3(ori(1,1),ori(2,2),ori(3,3),...
                            dir(1,1),dir(2,1),dir(3,1),...
                            'r','filled','LineWidth',3)
                    hold on
                    quiver3(ori(1,1),ori(2,2),ori(3,3),...
                            dir(1,2),dir(2,2),dir(3,2),...
                            'g','filled','LineWidth',3)
                    quiver3(ori(1,1),ori(2,2),ori(3,3),...
                            dir(1,3),dir(2,3),dir(3,3),...
                            'b','filled','LineWidth',3)
                end
                axis('equal')
            end
      end
      function ft_base = paletta2baseframe(self,pose,ft)
                    ori=pose{1}(:,5:8);
                    f = ft{1}(:,2:4);
                    t = ft{1}(:,5:7);
                    
                    m = min(size(ori,1),size(f,1));
                    for i=1:m
                        q = quaternion(ori(i,4),ori(i,1),ori(i,2),ori(i,3));
                        R = quat2rotm(q);
                        f_base(i,:) = 5*(R*f(i,:)')';
                        t_base(i,:) = 5*(R*t(i,:)')';
                    end
                    ft_base=[f_base,t_base];
      end
     

      function out=load_txt_files(self,type,path_to,files)
            out={};
            if nargin<4
                if nargin<3
                    path_to='.';
                    if nargin < 2
                        error('enter type of file (e.g. .pose2d)')
                    end
                end
                files2=dir([path_to,'/*',type,'.txt']);
                files={files.name};
            end
            for i=1:length(files)
                % Breakpoint strncmp(files{i}, FILENAME, 30)
                disp([files{i},type,'.txt'])
                dd=load([path_to,files{i},type,'.txt'],'ascii');
                %out{i}=[1e-9*(dd(:,1)-dd(1,1)),dd(:,2:4),(dd(:,1)-dd(1,1))/(dd(end,1)-dd(1,1)),dd(:,1)];
                if size(dd,1)>0
                    out{i}=[1e-9*(dd(:,1)),dd(:,2:end)];
                else
                    out{i}=[];
                end
            end
        end
        
        function savetopdf(self,filename)
            f=gcf;
            f.Position=[-1919           1        1920         997];
            f.PaperUnits='points';
            f.Units='pixels';
            f.PaperSize=f.Position(3:4);
            print(filename,'-dpdf','-fillpage');
            %      print(filename,'-dsvg');
        end
        
        function fancy_plot(self,ax)
            ax.LineWidth=1.0;
            for i=1:size(ax.Children,1)
                ax.Children(i).LineWidth=2.0;
            end
            %ax.Legend.FontSize
            %ax.FontSize=18;
            
        end
        
        function succ=get_success_rate(self)
            success=[self.res.success];
            %n_obj=max([self.res.obj_id]);
            n_obj=length(self.objs)
            succ=zeros(n_obj,2);
            for i=1:n_obj
                for j=1:2
                    ss=success(find([self.res.obj_id]==i & [self.res.strategy]==j));
                    succ(i,j)=size(find(ss==1),2)/size(ss,2);
                end
            end
        end
        
        function properties=load_properties(self)
            properties=[];
            
            properties(1).name='Apple';
            properties(1).size=[0.063 0.075 0.075]';
            properties(1).weight=0.061;
            
            properties(2).name='Chips can';
            properties(2).size=[0.035 0.185 0.033]';
            properties(2).weight=0.049;
            
            properties(3).name='Mug';
            properties(3).size=[0.14 0.18 0.051]';
            properties(3).weight=0.069;
            
            properties(4).name='Bowl';
            properties(4).size=[0.057 0.065 0.065]';
            properties(4).weight=0.144;
            
            properties(5).name='Funnel';
            properties(5).size=[0.057 0.126 0.159]';
            properties(5).weight=0.3;
            
            properties(6).name='Gelatin Box';
            properties(6).size=[0.074 0.153 0.125]';
            properties(6).weight=0.062;
            
            properties(7).name='Screwdriver';
            properties(7).size=[0.044 0.225 0.285]';
            properties(7).weight=0.012;
            
            properties(8).name='Banana';
            properties(8).size=[0.065 0.114 0.182]';
            properties(8).weight=0.144;
            
            properties(9).name='Clamp';
            properties(9).size=[0.09 0.09 0.342]';
            properties(9).weight=0.473;
            
            properties(10).name='Spray Bottle';
            properties(10).size=[0.05 0.05 0.185]';
            properties(10).weight=0.473;
            
            properties(11).name='Candy Tube';
            properties(11).size=[0.036 0.036 0.228]';
            properties(11).weight=0.473;
            
            properties(12).name='Teddy Bear';
            properties(12).size=[0.2 0.22 0.173]';
            properties(12).weight=0.473;
            
            properties(13).name='Water Bottle';
            properties(13).size=[0.072 0.237 0.072]';
            properties(13).weight=0.473;
            
            properties=properties(1:length(self.objs));
            
        end
 
        
        function id=get_obj_id(self,string_in)
            id=find(upper(string({self.props.name}))==upper(string_in));
            if isempty(id)
                error('invalid object')
            end
        end
        
        function results=load_file(self,file)
            %load(file,'ascii')
            fid=fopen(file);
            fgetl(fid);
            out=fgetl(fid);
            results=[];
            while ischar(out)
                experiment=[];
                out=fgetl(fid);
%                 disp(out)
                if ~isempty(out) >0 && ischar(out)
                    res_cell=strsplit(out,'\t');
                    if(length(res_cell)<4)
                        disp(res_cell)
                        continue
                    end
                    
                    experiment.obj_id=find(strcmp(self.objs,res_cell{2}));
                    if(isempty(experiment.obj_id))
                        continue
                    end
                    experiment.object=string(res_cell{2});
                    experiment.filename=res_cell{1};
                    obj=0;
                    
                    if string(res_cell{3})=='1'
                        experiment.success=1;
                    elseif string(res_cell{3})=='0'
                        experiment.success=0;
                    else
                        error(['Error loading file: str: ' out])
                    end
                    
                    if string(res_cell{5})=='1'
                        experiment.ftbase=1;
                    elseif string(res_cell{5})=='0'
                        experiment.ftbase=0;
                    else
                        error(['Error loading file: str: ' out])
                    end
                    
                    if string(res_cell{6})=='1'
                        experiment.obj_ori=1;
                    elseif string(res_cell{6})=='0'
                        experiment.obj_ori=0;
                    else
                        error(['Error loading file: str: ' out])
                    end
                    
                    if string(res_cell{7})=='1'
                        experiment.cub_ori=1;
                    elseif string(res_cell{7})=='2'
                        experiment.cub_ori=2;
                    elseif string(res_cell{7})=='3'
                        experiment.cub_ori=3;
                    elseif string(res_cell{7})=='0'
                        experiment.cub_ori=0;
                    else
                        error(['Error loading file: str: ' out])
                    end
                    results=[results,experiment];
                end
            end
        end
    end
end