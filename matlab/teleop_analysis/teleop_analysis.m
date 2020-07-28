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
%                 filename='results_January16.txt'
                filename='scoop_teleop.txt'
            end
            %obj.res
            set(groot,'defaulttextinterpreter','latex');
            set(groot, 'defaultAxesTickLabelInterpreter','latex');
            set(groot, 'defaultLegendInterpreter','latex');
            
            obj.objs={'Apple','Banana','Box','CBox','Red Box','Blue Box','Orange Box','Pasta Pack','Plastic Bottle','Spray Bottle','Candy Tube','Teddy Bear','Water bottle'};
           
            
            obj.res=obj.load_file(filename);
            obj.props=obj.load_properties();
%             obj.succ=obj.get_success_rate();
            
            
            %plot_succ_rates(res,properties);
            
        end
        
      function [exp,a] = dataset_generation(self)
%               pose=self.load_txt_files('.ft','data/',{filename});
              
            obj_id=[self.res.obj_id];
            objects=[self.res.object];
            filename={self.res.filename};
           
            for i=1:length(self.res)
                
                    js=self.load_txt_files('.js','data/',{filename{i}}); %Joint States
                    ee_pose=self.load_txt_files('.pose','data/',{filename{i}}); %EE Pose
                    ee_ft=self.load_txt_files('.ft','data/',{filename{i}}); %EE Force/Torque
                    box_pose=self.load_txt_files('.box_pose','data/',{filename{i}}); %Box Pose
                    
                    time=js{1}(:,1)-js{1}(1,1);
                    
                    h = floor(size(js,1)/size(box_pose,1));
                    res = size(js,1) - k*size(box_pose,1);
                    
                    for j=1:size(box_pose,1)-1
                        for k=1:h 
                            box_pose(j+1)
                        end
                    end
                    exp{i} = [time, js{1}(:,2:end)]; % 
                    bpos{i} = box_pose{1}(:,2:4);
                    
            end
                
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
            
            properties(2).name='Banana';
            properties(2).size=[0.035 0.185 0.033]';
            properties(2).weight=0.049;
            
            properties(3).name='Box';
            properties(3).size=[0.14 0.18 0.051]';
            properties(3).weight=0.069;
            
            properties(4).name='CBox';
            properties(4).size=[0.057 0.065 0.065]';
            properties(4).weight=0.144;
            
            properties(5).name='Red Box';
            properties(5).size=[0.057 0.126 0.159]';
            properties(5).weight=0.3;
            
            properties(6).name='Blue Box';
            properties(6).size=[0.074 0.153 0.125]';
            properties(6).weight=0.062;
            
            properties(7).name='Orange Box';
            properties(7).size=[0.044 0.225 0.285]';
            properties(7).weight=0.012;
            
            properties(8).name='Pasta Pack';
            properties(8).size=[0.065 0.114 0.182]';
            properties(8).weight=0.144;
            
            properties(9).name='Plastic Bottle';
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
                    results=[results,experiment];
                end
            end
        end
    end
end