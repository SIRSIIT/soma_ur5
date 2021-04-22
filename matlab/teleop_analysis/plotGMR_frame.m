function plotGMR_frame(data,data2,color,visibility)

if nargin<2
    o = eul2rotm(data(4:6)')*1; 
    color = [1 0 0];
elseif nargin==2
    o = eul2rotm(data2(1:3,1)')*1;
    color = [1 0 0];
else
     o = eul2rotm(data(4:6)')*0.75; 
end
if nargin < 4
    visibility = 1;
end

    p = [data(1),0,0;0,data(2),0;0,0,data(3)];
        
    if visibility == 0
        plot3(p(1,1),p(2,2),p(3,3),'Marker','*','MarkerSize',12,'color',color,'LineWidth',8,'HandleVisibility','off');
    else
        plot3(p(1,1),p(2,2),p(3,3),'Marker','*','MarkerSize',12,'color',color,'LineWidth',8);%,'HandleVisibility','off');
    end
    
    quiver3(p(1,1),p(2,2),p(3,3),...
            o(1,1),o(2,1),o(3,1),...
            'r','filled','LineWidth',6,'HandleVisibility','off')
    hold on
    quiver3(p(1,1),p(2,2),p(3,3),...
            o(1,2),o(2,2),o(3,2),...
            'g','filled','LineWidth',6,'HandleVisibility','off')
    quiver3(p(1,1),p(2,2),p(3,3),...
            o(1,3),o(2,3),o(3,3),...
            'b','filled','LineWidth',6,'HandleVisibility','off')
end