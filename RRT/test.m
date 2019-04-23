%% Test RRT:
close all
% arguments:
q_init = [1,1]';
q_goal = [8,8]';
num_nodes = 300;
box_size =10;
delta_q =0.5;
C_obs = {[3,6,6,3;0,0,5,5],[0,2,2;6,6,8]};
threshold = 0.5;
% build RRT: 
% Path returns the nodes on the path
% G: G.Vertex, G.Edges
[Path,G] = build_RRT(q_init,q_goal, num_nodes, delta_q, box_size, ...
    C_obs, threshold);
% Visualize the obs, nodes, edges and the path
for i = 1:length(C_obs)
    obs_i = cell2mat(C_obs(i));
    plot([obs_i(1,:),obs_i(1,1)],[obs_i(2,:),obs_i(2,1)],'b','LineWidth',1);
    axis([0,box_size,0,box_size]);
    hold on
end
for i = 1:size(G.Edge,2)
    plot(G.Edge(1:2:3,i),G.Edge(2:2:4,i),'k');
    hold on
end
scatter([q_init(1),q_goal(1)],[q_init(2),q_goal(2)],'gd','filled');
scatter(G.Vertex(1,:),G.Vertex(2,:),'k');
hold on
if Path ~= -1
    plot(Path(1,:),Path(2,:),'r','LineWidth',2);
end