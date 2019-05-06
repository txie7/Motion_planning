function [shortest_path,distance] = dijkstra(G, n_init, n_goal)

path_exist = true;

num = size(G,1); % number of nodes in the graph
dist= inf(1,num);% distance 
dist_for_check = inf(1,num);%This dist_for_check is an extra array used to check the minimum distance
prev = zeros(1,num);%Prev
U = 1:1:num;

dist(n_init) = 0; %Setup the distance to zero for the initial node
dist_for_check(n_init) = 0;


while(find(U == n_goal))%while the goal is still in the set U
    [min_value,C] = min(dist_for_check); %find the node with the minimum distance
    if min_value == inf
        path_exist = false;
        break
    end
    U(C) = 0; %Remove the node from U by setting it to 0
    dist_for_check(C) = inf; 
    
    for i = 1:num % Update the dist for neighbours
        if G(C,i) < inf %When G(C,i) is less than inf, the two nodes are connected.
            alt = dist(C) + G(C,i);
            if alt < dist(i)
                dist(i) = alt;
                dist_for_check(i) = alt;
                prev(i) = C;
            end
        end            
    end
end

if path_exist == true
    % Construct the shortest path
    shortest_path = zeros(1,num);
    index = n_goal;
    i = 1;
    shortest_path(i) = n_goal;
    while (prev(index)~= n_init)
        i = i+1;
        shortest_path(i) = prev(index);
        index = prev(index);
    end
    %Output the shortest path
    shortest_path = fliplr([reshape(nonzeros(shortest_path),...
        [1,size(nonzeros(shortest_path))]),n_init]);
    distance = dist(n_goal);
else
    shortest_path = -1;
    distance = -1;
end

end


