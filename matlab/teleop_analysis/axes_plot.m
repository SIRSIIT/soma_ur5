function axes_plot(x0,y0,z0,l,alpha,gamma)
if nargin == 3
    l=1;
    alpha=0;
    gamma=0;
elseif nargin == 4
    alpha=0;
    gamma=0;
end
    ori = [x0,0,0;0,y0,0;0,0,z0];
    dir = rotx(alpha)*rotz(gamma)*[1,0,0;0,1,0;0,0,1]*l;
    quiver3(ori(1,1),ori(2,2),ori(3,3),...
            dir(1,1),dir(1,2),dir(1,3),...
            'r','filled','LineWidth',6)
    hold on
    quiver3(ori(1,1),ori(2,2),ori(3,3),...
            dir(2,1),dir(2,2),dir(2,3),...
            'g','filled','LineWidth',6)
    quiver3(ori(1,1),ori(2,2),ori(3,3),...
            dir(3,1),dir(3,2),dir(3,3),...
            'b','filled','LineWidth',6)
%     quiver3(dir(2,:),new_dir(2,:),'g','filled')
%     quiver3(dir(3,:),new_dir(3,:),'b','filled')
end
