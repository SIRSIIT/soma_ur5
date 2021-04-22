function Data = cluster_dataset(Data,N,k,obj)

M = size(Data,2);  

            switch obj
                case 'cyl'
                     b = [];
                     t = [];
                     s = [];
            for i=1:M
%                 if Data(7,i)<=-60 || Data(8,i)>=60 
%                     h = [h; i];
%                 end
                if abs(Data(5,i))<=0.165 && abs(Data(6,i))<=2.6 
                    b = [b; i];
                
                elseif abs(Data(5,i))<=0.165 && abs(Data(6,i))>2.6 
                    t = [t; i];
                
                else % abs(Data(5,i))<=0.1 && Data(6,i)>2.1 
                    s = [s; i];
                end
                
            end

            for i=1:length(b) % BOTTOM
                dir = eul2rotm(Data(4:6,b(i))');
                m = dir(2,3)/dir(1,3);
                r = Data(9,b(i))*2;
                min_r = 100;
                off = 0.5*rand(1);
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                x=linspace(0,10,50);
                y=m*linspace(0,10,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [x(j)*sgn,0,0;0,y(j)*sgn,0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end
                    Data(1,b(i)) = ori(1,1);
                    Data(2,b(i)) = ori(2,2);
                    Data(3,b(i)) = -Data(10,b(i))/2.3+off;%Data(10,b(i))/2.5 + 0.5*rand(1);
            end
            
            for i=1:length(t) %TOP
                dir = eul2rotm(Data(4:6,t(i))');
                m = dir(2,3)/dir(1,3);
                r = Data(9,t(i))*1.4;
                min_r = 100;
                off = 0.5*rand(1);
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                x=linspace(0,5,50);
                y=m*linspace(0,5,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [x(j)*sgn,0,0;0,y(j)*sgn,0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end
                    Data(1,t(i)) = ori(1,1);
                    Data(2,t(i)) = ori(2,2);
                    Data(3,t(i)) = Data(10,t(i))/2+off;%Data(10,t(i))/2.5 + 0.5*rand(1);
            end
            for i=1:length(s) %SIDE
                dir = eul2rotm(Data(4:6,s(i))');
                m= dir(2,2)/dir(1,2);
                r = Data(9,s(i))*1.2;
                min_r = 100;
                sgn = sign(dot(dir(:,2),[1 0 0]));
                off = 0.5*rand(1);
                 x=linspace(0,7,50);
                y=m*linspace(0,7,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [sgn*x(j),0,0;0,sgn*y(j),0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end

                    Data(1,s(i)) = ori(1,1);
                    Data(2,s(i)) = ori(2,2);
                    Data(3,s(i)) = off;%Data(10,s(i)) + 0.5*rand(1);
            end
%             for r=1:3
%                 j=1;
%                 for i=size(Data,2)+1:size(Data,2)+length(h)
%                     Data(:,i) = [Data(1:end-k-2,h(j)) + 0.1*rand(size(Data,1)-k-2,1);...
%                                  Data(end-k-1:end,h(j))];
%                     j=j+1;
%                 end
%             end
%                      for i=size(Data,2)+1:N % Cylinder
% %                          s1=sign(rand(1)-0.5);
% %                          s2=sign(rand(1)-0.5);
%                          r = 1.8 + (3.3-1.8).*rand(1)*0;
% %                          h = 0*3*rand(1);
%                          h = 13 + (23-13).*rand(1);
%                 Data(:,i) = [Data(1,i-M)*r/Data(end-k+1,i-M);...
%                              Data(2,i-M)*r/Data(end-k+1,i-M);...
%                              Data(3,i-M)*h/Data(end-k+2,i-M);...
%                              Data(4:end-k,i-M);...
%                              round(r);...
%                              round(h)];
%                 end
                    
                case 'cub'
                     y = [];
                     x = [];
                     z = [];
            for i=1:M

                if abs(Data(4,i))<=.5%.78
                    y = [y; i];
                
                elseif abs(Data(4,i))>=1.1
                    x = [x; i];
                else
                    z = [z; i];
                end
                
            end
            
            for i=1:length(x)
                dir = eul2rotm(Data(4:6,x(i))');
                m= dir(2,2)/dir(1,2);
                l = (Data(9,x(i))/2)*1.5;
                min_r = 100;
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                off = 0.5*rand(1);
                xx=linspace(-20,20,50);
                yy=m*linspace(-20,20,50);
                for j=1:length(xx)
                    if xx(j)>l+off && sqrt(yy(j)^2+xx(j)^2)<min_r
                        ori = [sgn*xx(j),0,0;0,sgn*yy(j),0;0,0,0];
                        min_r = sqrt(yy(j)^2+xx(j)^2);
                    end
                end

                    Data(1,x(i)) = ori(1,1);
                    Data(2,x(i)) = ori(2,2);
                    if Data(6,x(i)) > -2.5
                        Data(3,x(i)) = -20;%Data(3,x(i))-Data(11,x(i))/3-off;
                    else
                        Data(3,x(i)) = 30;%Data(3,x(i))+Data(11,x(i))/3+off;
                    end
            end
            
            for i=1:length(y)
                dir = eul2rotm(Data(4:6,y(i))');
                m= dir(2,2)/dir(1,2);
                l = (Data(10,y(i))/2)*1.5;
                min_r = 100;
                sgn = -sign(dot(dir(:,3),[0 1 0]));
                off = 0.5*rand(1);
                xx=1/m*linspace(-20,20,50);
                yy=linspace(-20,20,50);
                for j=1:length(xx)
                    if yy(j)>l+off && sqrt(yy(j)^2+xx(j)^2)<min_r
                        ori = [sgn*xx(j),0,0;0,sgn*yy(j),0;0,0,0];
                        min_r = sqrt(yy(j)^2+xx(j)^2);
                    end
                end

                    Data(1,y(i)) = ori(1,1);
                    Data(2,y(i)) = ori(2,2);
                    if Data(6,y(i)) > -2.5
                        Data(3,y(i)) = -40;%Data(3,y(i))-Data(11,y(i))/3-off;
                    else
                        Data(3,y(i)) = 10;%Data(3,y(i))+Data(11,y(i))/3+off;
                    end
%                     if ori(2,2)<-25
%                         a=1;
%                     end
            end
            Data(:,z) = [];
            %             M+29 -> escludi cubo 
                
%             for i=size(Data,2)+1:N % CUBE
%                 x = 5*rand(1)*sign(rand(1)-0.5);
%                 y = 5*rand(1)*sign(rand(1)-0.5);
%                 z = 5*rand(1)*sign(rand(1)-0.5);
%                 Data(:,i) = [Data(1,i-M+29)- x/2;...
%                              Data(2,i-M+29)- y/2;...
%                              Data(3,i-M+29)- z/2;...
%                              Data(4:end-k-1,i-M+29);...
%                              Data(end-k,i-M+29)+ x;...
%                              Data(end-k+1,i-M+29)+ y;...
%                              Data(end-k+2,i-M+29)+ z];
%             end 

                case 'hol'
%                     for i=size(Data,2)+1:N % Hollow
%                 r = 2*rand(1)*sign(rand(1)-0.5);
%                 h = 2*rand(1)*sign(rand(1)-0.5);
%                 Data(:,i) = [Data(1,i-M)- r/2;...
%                              Data(2,i-M)- r/2;...
%                              Data(3,i-M)+ h/2;...
%                              Data(4:end-k,i-M);...
%                              Data(end-k+1,i-M)+ r;...
%                              Data(end-k+2,i-M)+ h];
%                 end
                     b = [];
                     t = [];
                     
            for i=1:M

                if abs(Data(6,i))<=2.6  %abs(Data(5,i))<=0.165 && 
                    b = [b; i];
                
                elseif  abs(Data(6,i))>2.6  %abs(Data(5,i))<=0.165 &&
                    t = [t; i];
                end
                
            end
%             for i=1:length(h)
%                     Data(3,h(i)) = -Data(10,h(i))/2.5 + 0.5*rand(1);
%             end
            for i=1:length(b) % BOTTOM
                dir = eul2rotm(Data(4:6,b(i))');
                m = dir(2,3)/dir(1,3);
                r = Data(9,b(i))*2;
                min_r = 100;
                off = 0.5*rand(1);
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                x=linspace(0,10,50);
                y=m*linspace(0,10,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [x(j)*sgn,0,0;0,y(j)*sgn,0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end
                    Data(1,b(i)) = ori(1,1);
                    Data(2,b(i)) = ori(2,2);
                    Data(3,b(i)) = -Data(10,b(i))/2.2-off;%Data(10,b(i))/2.5 + 0.5*rand(1);
            end
            
            for i=1:length(t) %TOP
                dir = eul2rotm(Data(4:6,t(i))');
                m = dir(2,3)/dir(1,3);
                r = Data(9,t(i))*1.4;
                min_r = 100;
                off = 0.5*rand(1);
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                x=linspace(0,10,10);
                y=m*linspace(0,10,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [x(j)*sgn,0,0;0,y(j)*sgn,0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end
                    Data(1,t(i)) = ori(1,1);
                    Data(2,t(i)) = ori(2,2);
                    Data(3,t(i)) = Data(10,t(i))/2+off;%Data(10,t(i))/2.5 + 0.5*rand(1);
            end
            
%              for i=size(Data,2)+1:N 
%                          r = 4.6 + (8-4.5).*rand(1)*0;
% %                          h = 0*3*rand(1);
%                          h = 5 + (13-8).*rand(1);
%                 Data(:,i) = [Data(1,i-M)*r/Data(end-k+1,i-M);...
%                              Data(2,i-M)*r/Data(end-k+1,i-M);...
%                              Data(3,i-M)*h/Data(end-k+2,i-M);...
%                              Data(4:end-k,i-M);...
%                              round(r);...
%                              round(h)];
%              end
                case 'sph'
                     b = [];
                     t = [];
            for i=1:M

                if abs(Data(6,i))<=2.6 
                    b = [b; i];
                
                else
                    t = [t; i];
                end
                
            end

            for i=1:length(b) % BOTTOM
                dir = eul2rotm(Data(4:6,b(i))');
                m = dir(2,3)/dir(1,3);
                r = Data(9,b(i))*2;
                min_r = 100;
                off = 0.5*rand(1);
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                x=linspace(0,10,50);
                y=m*linspace(0,10,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [x(j)*sgn,0,0;0,y(j)*sgn,0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end
                    Data(1,b(i)) = ori(1,1);
                    Data(2,b(i)) = ori(2,2);
                    Data(3,b(i)) = -Data(9,b(i))/2.3+off;%Data(10,b(i))/2.5 + 0.5*rand(1);
            end
            
            for i=1:length(t) %TOP
                dir = eul2rotm(Data(4:6,t(i))');
                m = dir(2,2)/dir(1,2);
                r = Data(9,t(i))*1.2;
                min_r = 100;
                off = 0.5*rand(1);
                sgn = -sign(dot(dir(:,3),[1 0 0]));
                x=linspace(0,10,50);
                y=m*linspace(0,10,50);
                for j=1:length(x)
                    if sqrt(y(j)^2+x(j)^2)>r+off && sqrt(y(j)^2+x(j)^2)<min_r
                        ori = [x(j)*sgn,0,0;0,y(j)*sgn,0;0,0,15];
                        min_r = sqrt(y(j)^2+x(j)^2);
                    end
                end
                    Data(1,t(i)) = ori(1,1);
                    Data(2,t(i)) = ori(2,2);
                    Data(3,t(i)) = Data(9,t(i))/2+off;%Data(10,t(i))/2.5 + 0.5*rand(1);
            end
            end
        end
    
%         for i=size(Data,2)+1:N
%         Data(:,i) = [Data(1:end-k,i-M) + 0.1*rand(size(Data,1)-k,1);Data(end-k+1:end,i-M)];
%     end  
