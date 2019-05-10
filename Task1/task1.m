function x = task1(q_init, q_goal, n, link_length, C_obs)

num_nodes = 100;
% Define the origin of the serial link
serial_link = struct('Origin',[0,0],'Link_length',link_length,'DoF',n);

% build 
joint_angles = zeros(n,num_nodes);
for k = 1:num_nodes
    joint_angles(:,k) = -pi + 2*pi*rand(n,1);
end

joint_angles = [q_init,joint_angles,q_goal];
vertices = [];
for k = 1:num_nodes+2
    collision = isintersect_seriallink_obs(joint_angles(:,k), serial_link, C_obs);
    if ~collision
        vertices = [vertices joint_angles(:,k)];
    end
end

num_vertices = length(vertices);
G = zeros(num_vertices);
for i = 1 : num_vertices
    idx = knnsearch(vertices, vertices(:,i),'K',5);
    for j = 1:length(idx)
        
    


            