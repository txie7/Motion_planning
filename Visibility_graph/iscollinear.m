function b = iscollinear(q,v,obs)

% This function is used to check if the points q,v are blocked by some 
% collinear point obs_vx in the obstacle
% E.g     obs_v4------obs_v3
%           |           |
%           |           |
%     q---obs_v1--------v
% The case above, b = true.
% E.g     obs_v4------obs_v3
%           |           |
%           |           |
%     q-----v---------obs_v2
% The case above, b = false. 


b = false;

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

if isequal(v,q) || isequal(v_prev,q) || isequal(v_next,q)
    b = false;
else
    z_prev = dot(v-v_prev,q-v_prev)/(norm(v-v_prev)*norm(v_prev-q))+1;
    z_next = dot(v-v_next,q-v_next)/(norm(v-v_next)*norm(v_next-q))+1;

    if z_prev<1e-8 || z_next <1e-8
        b = true;
    end
end

midpoint = 1e-8*(q-v)+v;
[in_obs,on_obs] = inpolygon(midpoint(1),midpoint(2),obs(1,:),obs(2,:));
if in_obs == 1 && on_obs == 0
    b =true;
end

end
