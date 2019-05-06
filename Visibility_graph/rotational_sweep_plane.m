function visible_V = rotational_sweep_plane(q_input,V,C_obs)
% Input: 
% - q_input: 3x1 vector, input vertex
% - V : all vertices and the obstacle the vertex belongs to
% - C_obs: cell array of obstacles
% Output:
% - visible_V: 1xnum_vertices vector (1:visible V, 0: invisible)
%==========================================================================
q = q_input(1:2);
visible_V = zeros(1,size(V,2));
x = [1 0]';
num_obs = length(C_obs);
num_vertices = size(V,2);

% Sort all the vertices according to the angle:
angles = zeros(1, num_vertices); 
for i = 1:num_vertices
    v = V(1:2,i) - q;
    if norm(v) == 0
        if V(3,i) == q_input(3)
            angles(i) = inf;
        else
            angles(i) = 0;
        end
    else
        theta = acos(v'*x/norm(v));
        if v(2) < 0
            theta = 2*pi - theta;
        end
        angles(i) = theta;
    end
end
[~,idx] = sort(angles,'ascend');
% if the angle is the same, sort it by distance to q
for i = 1:num_vertices - 1
    if angles(idx(i)) == angles(idx(i+1))
        v1 = V(1:2,idx(i));
        v2 = V(1:2,idx(i+1));
        if norm(v2-q) < norm(v1-q)
            temp = idx(i+1);
            idx(i+1) = idx(i);
            idx(i) = temp;
        end
    end
end
%==========================================================================    

% initialize the T with the edges intersect x-pos ccw
T = BinarySearchTree();
x_pos = [q(1)+1e8 q(2)]';
x_pos_ccw = [q(1)+1e8 q(2)+1e-3]';
for i = 1:num_obs
    obs = cell2mat(C_obs(i));
    if isintersect_linepolygon([q+[1e-8 0]',x_pos],obs) % if the line intersect the polygon
        for j = 1:size(obs,2)-1 % find the corresponding edges and insert
            m = obs(:,j);
            n = obs(:,j+1);
            l2 = [m,n];
            l1 = [q,x_pos];
            l3 = [q,x_pos_ccw];
            if isintersect_linepolygon(l1,l2) && isintersect_linepolygon(l3,l2)
                key = cal_key(l1,l2);
                if key > 0
                    T.Insert(key,[m;n]);
                end                
            end
        end
    
        m = obs(:,end);
        n = obs(:,1);
        l2 = [m,n];
        l1 = [q,x_pos];
        l3 = [q,x_pos_ccw];
        if isintersect_linepolygon(l1,l2) && isintersect_linepolygon(l3,l2)
            key = cal_key(l1,l2);
            if key > 0
                T.Insert(key,[m;n]);
            end            
        end    
    end    
end
%==========================================================================

% Sweep the line
for i = 1:num_vertices-1
    index = idx(i);
    v = V(1:2,index);
    which_obs = V(3,index);
    % if two points identical, visible
    if norm(v-q) == 0
        visible_V(index) = 1;
        continue
    end
    %----------------------------------------------------------------------
    % Update the trees
    l1 = [q,v];
    keys = T.Sort();
    if ~isempty(keys)
        edge = zeros(4,length(keys));
        new_keys = zeros(1,length(keys));
        for j = 1:length(keys)
            t = T.Search(keys(j));
            edge(:,j) = t.value;
            new_keys(j) = cal_key(l1,[edge(1:2,j),edge(3:4,j)]);
            T.Delete(t);
        end
        for j = 1:length(new_keys)
            T.Insert(new_keys(j),edge(:,j));
        end
    end
    % ---------------------------------------------------------------------
    % Delete edges that the sweep line is leaving   
    if which_obs == 0
        key= round(norm(v-q),4);
        T.Insert(key,[v;v]);
    else
        obs = cell2mat(C_obs(which_obs));
        dist = round(norm(v-q),4);
        nodes_w_same_dist = length(find(T.Sort()== dist));
        edges = zeros(4,nodes_w_same_dist);
        count = 0;
        while(T.ContainsKey(dist))
            count = count +1;
            t = T.Search(dist); % The edge already existed
            edges(:,count) = t.value;
            T.Delete(t);
        end
        % Recover some edges deleted. (These edges belong to other
        % obstacles but also intersect with v. And they are on LHS.)
        for m = 1:count
            e1 = edges(1:2,m);
            e2 = edges(3:4,m);
            if isequal(e1,v)
                z = cross([v-q;0],[e2-v;0]);
                if z(3) > 0
                    T.Insert(dist,edges(:,m));
                end
            end
            if isequal(e2,v)
                z = cross([v-q;0],[e1-v;0]);
                if z(3) > 0
                    T.Insert(dist,edges(:,m));
                end
            end
        end
        %------------------------------------------------------------------    
        % Then adding neighboring edges accordingly
        [~,v_in_obs] =  ismember(v',obs','rows');
        v_prev = v_in_obs - 1;
        v_next = v_in_obs + 1;
        if v_in_obs == 1
            v_prev = size(obs,2);
        end       
        if v_in_obs == size(obs,2)
            v_next = 1;
        end
        v_prev = obs(:,v_prev);
        v_next = obs(:,v_next);
        
        z_prev = cross([v-q;0],[v_prev-v;0]);
        if z_prev(3) > 0
            T.Insert(round(norm(v-q),4),[v_prev;v]);
        end

        z_next = cross([v-q;0],[v_next-v;0]);
        if z_next(3) > 0
            T.Insert(round(norm(v-q),4),[v;v_next]);
        end

        
    end
    %----------------------------------------------------------------------
    % Check visibility(Do not connect vetices from the same obs at this stage)
    closest_edge = T.Minimum();
    
    block_v = false;
    block_q = false;
    if which_obs ~= 0        
        obs = cell2mat(C_obs(which_obs));
        block_v = iscollinear(q,v,obs);       
    end
    if q_input(3) ~= 0        
        obs = cell2mat(C_obs(q_input(3)));
        block_q = iscollinear(v,q,obs);     
    end
                
    if ~block_v && ~block_q && ~issegment_in_obs(q,v,C_obs)           
        if isnan(closest_edge) 
            if which_obs == 0
                visible_V(index) = 1;
            else
                if which_obs ~= q_input(3)
                    visible_V(index) = 1;
                end
            end
        else

            [key,~] = closest_edge.key_value();
            if  key >= round(norm(v-q),4)
                if which_obs == 0
                    visible_V(index) = 1;
                else
                    if which_obs ~= q_input(3)
                        visible_V(index) = 1;
                    end
                end
            end
        end
    end
    %----------------------------------------------------------------------
    % Delete the node if it's a point (q_init/q_goal)
    if which_obs == 0
        t = T.Search(round(norm(v-q),4));
        while (~isequal(t.value,[v;v]))
            t = T.NextLargest(t);            
        end
        T.Delete(t);
    end

end
T.Clear();       
end
    

