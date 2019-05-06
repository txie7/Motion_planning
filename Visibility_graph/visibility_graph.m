function [G,G_w,V_path,Vertices] = visibility_graph(q_init,q_goal,C_obs)
%Inputs: 
% C_obs: C_obs = {[obs1],[obs2],...} a cell array of all obstacles
% q_init: 2x1 vector, the initial configuration
% q_goal: 2x1 vector, the goal configuration
%Outpus:
% G: adjacency matrix
% G_w: weighted adj matrix
% V_path: the vertices on the path
% Vertices: the set of all vertices
%==========================================================================

% Put all the vertices into a whole matrix. And keep track of from which
% obstacle the vertex comes from.
num_obs = length(C_obs);
V = [q_init;0]; % V is the set of all vertices (init, goal and obs)
for i = 1:num_obs
    obs = cell2mat(C_obs(i)); % obs is a 2xN matrix
    % Record which obs the vertex belongs to (wrote it into the 3rd row)
    V = [V,[obs;i*ones(1,size(obs,2))]];  
end
V = [V,[q_goal;0]];
Vertices = V(1:2,:);

% For each vertex in the space, find the visible vertices (vertices in the 
% obstacle is not yet included at this stage).
G = zeros(size(V,2));
for i = 1:size(V,2)
    q = V(:,i);
    visible_V = rotational_sweep_plane(q,V,C_obs);
    G(i,:) = visible_V;
end

% Complete the adjacent matrix by adding in vertices visible from the same
% obstacle.
for i = 1:size(V,2)
    for j = 1:size(V,2)
        if G(i,j) == 1 || i==j
            continue
        else
            if G(j,i) == 1 
                G(i,j) = 1;
                continue
            end
            if V(3,i) == V(3,j) && V(3,i)>0
                obs = cell2mat(C_obs(V(3,i)));
                [~,which_v_i] = ismember(V(1:2,i)',obs','rows');
                [~,which_v_j] = ismember(V(1:2,j)',obs','rows');
                if abs(which_v_i - which_v_j) == 1 || ...
                        (which_v_i == 1 && which_v_j == size(obs,2))|| ...
                        (which_v_j == 1 && which_v_i == size(obs,2))
                    G(i,j) = 1;
                    G(j,i) = 1;
                    for n = 1:num_obs
                        if n == V(3,i)
                            continue
                        end
                        obs_n = cell2mat(C_obs(n));
                        if isintersect_linepolygon([V(1:2,i),V(1:2,j)],obs_n)
                            for t=0:0.01:1
                                pt = V(1:2,i) + t*(V(1:2,j)-V(1:2,i));                            
                                [in,on] = inpolygon(pt(1),pt(2),obs_n(1,:),obs_n(2,:));
                                if in ==1 && on == 0
                                    G(i,j) = 0;
                                    G(j,i) = 0;
                                    break
                                end
                                
                            end
                        end
                    end
                end
            end
        end
    end
end

% Check if the points are in any obstacle
for n = 1:2
    if n == 1
        q = q_init;
    else
        q = q_goal;
    end
    for i= 1:num_obs
        obs = cell2mat(C_obs(i));
        [in,on] = inpolygon(q(1),q(2),obs(1,:),obs(2,:));
        if in == 1 && on == 0
            G(n,:) = zeros(1,size(V,2));
            G(:,n) = zeros(1,size(V,2))';
            break
        end
                   
    end
    
end



% Generate the weighted adjacency matrix
G_w = inf*ones(size(V,2));
for i = 1:size(V,2)
    for j = 1:size(V,2)
        if i == j
            G_w(i,j) = 0;
        else 
            if G(i,j) == 1
                v1 = V(1:2,i);
                v2 = V(1:2,j);
                G_w(i,j) = norm(v1-v2);
            end
        end
    end
end

V_path_nodes = dijkstra(G_w,1,size(V,2));
if V_path_nodes == -1
    V_path = -1;
    warning('Path does not exist!')
else
    V_path = Vertices(:,V_path_nodes);
end
end


    

        
