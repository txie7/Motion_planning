function [Path, G] = build_PRM(q_init,q_goal,num_nodes,box_size,C_obs,K)
%Input format:
%   q_init: initial config, 2x1 vector
%   q_goal: goal config, 2x1 vector
%   num_nodes: maximum number of nodes: scalar
%   box_size: scalar,size of the square open space
%   C_obs: cell array of obstacles, {[obs1:2xN1],[obs2:2xN2],...}
%   K: k number of neighbours
%Output: 
%   G: G.Vertex is the array of vertices, G.Edge is a [4,N] array of edges,
%      G.Graph is the weighted adjacency matrix.
%   Path: return a path

G = struct('Vertex',[q_init,q_goal],'Edge',[],'Graph',inf*ones(num_nodes));
num_obs = length(C_obs);

% Generate random nodes
for i = 1:num_nodes-2
    q_rand = box_size*rand(2,1);
    G.Vertex = [G.Vertex, q_rand];
end

% while the edge hasn't been checked and it is collision free, add the edge
% into G.Edge. Change the G.Graph accordingly.
for i = 1:num_nodes
    Idx = knnsearch(G.Vertex',G.Vertex(:,i)','K',K);
    for j = 1:length(Idx)
        if G.Graph(i,Idx(j))== inf 
            b = false;
            % check collision
            for n = 1: num_obs
                obs_i = cell2mat(C_obs(n));
                b = isintersect_linepolygon([G.Vertex(:,i),G.Vertex(:,Idx(j))],obs_i);
                if b == true
                    break
                end
            end
            if b== true
                continue
            else
                % Weighted adjacency matrix
                G.Graph(i,Idx(j)) = norm(G.Vertex(:,i)-G.Vertex(:,Idx(j)));
                G.Graph(Idx(j),i) = norm(G.Vertex(:,i)-G.Vertex(:,Idx(j)));
                G.Edge = [G.Edge,[G.Vertex(:,i);G.Vertex(:,Idx(j))]];
            end
        end
            
    end
        
end

% The diagonal element should be 0 in the graph
for i = 1:num_nodes
    G.Graph(i,i) = 0;
end
% Graph search to find the path
[Path_idx,~] = dijkstra(G.Graph,1,2);
if Path_idx == -1
    warning('Did not find the path!')
    Path = -1;
else
    Path = G.Vertex(:,Path_idx);
end

end