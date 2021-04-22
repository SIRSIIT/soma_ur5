function plottami(T)
T(1:3,1:3)=T(1:3,1:3)*1e-2;
quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),'r','filled','LineWidth',6)
hold on
quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),'g','filled','LineWidth',6)
quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),'b','filled','LineWidth',6)
plot3(T(1,4),T(2,4),T(3,4),'*','color','k')
end