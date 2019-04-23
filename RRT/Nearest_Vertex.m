function [v_new,idx] = Nearest_Vertex(q,G)

d = inf;
num_nodes = size(G,2);
idx = 0;

for i = 1: num_nodes
    if norm(q-G(:,i))<d 
        v_new = G(:,i);
        d = norm(q-G(:,i));
        idx = i;
    end
end

end