%% Test visibility_graph:
clear all; clc;close all

% Input C_obs, q_init and q_goal for testing:
%--------------------------------------------------------------------------
% Several C_obs for testing
% 1. Similar test case in handout:
%C_obs = {[1 2 3 3 0 0; 0 0 2 3 3 2];[0 2 3 2 0 -1; -7 -7 -5 -3 -3 -5]};
% 2. C_obs intersects:
C_obs = {[0 1 1 0; 0 0 4 4],[-1 7 -1 -3; 0 2 3 1],[1.2 3 3 1; 0 0 3 3]};
% 3. One more test case:
% C_obs = {[-3 0 0 -3; 0 0 1 1],[1 2 2 1; 0 0 1 1],[0 1 1 0; 1 1 2 2]};

q_goal = [-1,0]';
q_init = [7,2]';

% Generate the visibility graph, note: q_init:2x1, q_goal:2x1, 
% C_obs:{[2xN1],[2xN2],...}
% ------------------------------------------------------------------------
[G,G_w,V_path,V] = visibility_graph(q_init,q_goal,C_obs);

%--------------------------------------------------------------------------
%Visualize the graph
for i = 1:length(C_obs)
    obs_i = cell2mat(C_obs(i));
    fill(obs_i(1,:),obs_i(2,:),'y');
    hold on
end
scatter([q_init(1),q_goal(1)],[q_init(2),q_goal(2)],'rd','filled');
hold on
% 
for i = 1:size(G,2)
    for j = i+1:size(G,2)
        if G(i,j) == 1
            plot([V(1,i),V(1,j)],[V(2,i),V(2,j)],'g');
            hold on
        end
    end
end
% 
if V_path == -1
    warning('Path does not exist!')
else
    for i = 1:size(V_path,2)-1
        plot([V_path(1,i),V_path(1,i+1)],[V_path(2,i),V_path(2,i+1)],'r','LineWidth',1);
        hold on
    end
end

axis square;