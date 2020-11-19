// MATLAB usage
std::vector<double> x(200);
std::vector<double> y(200);
std::vector<double> z(200);

meshgen::mesh_grid<double, 0, 3> X;
meshgen::mesh_grid<double, 1, 3> Y;
meshgen::mesh_grid<double, 2, 3> Z;

%x[0]=obj_pos.x;y[0]=obj_pos.y;z[0]=obj_pos.z;

std::tie(X, Y, Z) = meshgen::meshgrid(x, y, z);

start = [ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z];
goal = [obj_pos.x, obj_pos.y, obj_pos.z];

f = k * ( (x - goal(1)).^2 + (y - goal(2)).^2 + (z - goal(3)).^2 );

siz = size(f);
gy  = zeros(size(f),class(f)); % case of singleton dimension
h = x(1,:,end)'; 
n = siz(1);

gy(1,:) = -(f(2,:) - f(1,:))/(h(2)-h(1));
gy(n,:) = -(f(n,:) - f(n-1,:))/(h(end)-h(end-1));

gy(2:n-1,:) = -(f(3:n,:)-f(1:n-2,:)) ./ (h(3:n) - h(1:n-2));

    % N-D case
    ndim=3;
for k = 2:ndim
        n = siz(k);
        newsiz = [prod(siz(1:k-1)) siz(k) prod(siz(k+1:end))];
        nf = reshape(f,newsiz);
%         h = reshape(loc{k},1,[]);
        h=h';
        g  = zeros(size(nf),class(nf)); % case of singleton dimension
        
        % Take forward differences on left and right edges
        if n > 1
            g(:,1,:) = (nf(:,2,:) - nf(:,1,:))/(h(2)-h(1));
            g(:,n,:) = (nf(:,n,:) - nf(:,n-1,:))/(h(end)-h(end-1));
        end
        
        % Take centered differences on interior points
        if n > 2
            h = h(3:n) - h(1:n-2);
            g(:,2:n-1,:) = (nf(:,3:n,:) - nf(:,1:n-2,:)) ./ h;
        end
end        

