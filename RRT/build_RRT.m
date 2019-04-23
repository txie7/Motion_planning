function [Path, G] = build_RRT(q_init,q_goal, num_nodes, delta_q, box_size, ...
    C_obs, threshold)
% Input format:
%   q_init: initial config, 2x1 vector
%   q_goal: goal config, 2x1 vector
%   num_nodes: maximum number of nodes: scalar
%   delta_q: scalar
%   box_size: scalar,size of the square open space
%   C_obs: cell array of obstacles, {[obs1:2xN1],[obs2:2xN2],...}
%   threshold: the distance to connect the goal node
% Output: 
%   G with G.Vertex and G.Edge, where G.vertex is[2xN] and G.Edge is
%   [4x(N-1)].
%   Path
close all
% Initialize the tree
G = struct('Vertex',q_init,'Edge',[],'Parent',[0]);

num_obstacles = length(C_obs);
for k =1:num_nodes
    rng('shuffle');
    while(1)
        q_rand = box_size*rand(2,1);      
        [q_near,idx] = Nearest_Vertex(q_rand,G.Vertex);
        q_new = q_near + (q_rand-q_near)*delta_q/norm(q_rand-q_near);
        q_new = max([0, 0]',min([10,10]',q_new));
        b1 = false;
        b2 = false;
        % Check collision
        for i = 1:num_obstacles
            obs_i = cell2mat(C_obs(i));
            b1 = isintersect_linepolygon([q_new,q_new],obs_i);
            b2 = isintersect_linepolygon([q_new,q_near],obs_i);
            if b1 == true || b2 == true
                break
            end
        end 
        % Add vertex and edge when collision-free
        if (b1 == false && b2 == false)
            G.Vertex = [G.Vertex,q_new];
            G.Edge = [G.Edge,[q_near;q_new]];
            G.Parent = [G.Parent,idx];
            break
        end
    end
    % Stop adding node when the distance between goal and the newest
    % vertex added is within a threshold. Check collision. If
    % collision-free, then the tree is complete
    b = true;    
    if norm(G.Vertex(:,end) - q_goal) <= threshold
        b = false;
        for i = 1:num_obstacles
            obs_i = cell2mat(C_obs(i));
            b = isintersect_linepolygon([G.Vertex(:,end),q_goal],obs_i);
            if b == true
                break
            end
        end
        if b == true
            break
        end
        
    end
    
    if b == false
        G.Edge = [G.Edge,[G.Vertex(:,end);q_goal]];
        G.Vertex = [G.Vertex,q_goal];
        G.Parent = [G.Parent,length(G.Vertex)-1];
        break
    end
end

% Return the path if found
if k == num_nodes && norm(G.Vertex(:,end)-q_goal)~= 0
    warning('Did not find the path!')
    Path = -1;
else
    Path = q_goal;
    parent_node = length(G.Vertex);
    while(1)
        parent_node = G.Parent(parent_node);
        if parent_node ==0
            break
        end
        Path = [G.Vertex(:,parent_node),Path];
    end
end
    

end


        
            
            
    
    
    
