function b = issegment_in_obs(q,v,C_obs)

b = false;

for i = 1:length(C_obs)
    obs = cell2mat(C_obs(i));
    q_in_obs = inpolygon(q(1),q(2),obs(1,:),obs(2,:));
    v_in_obs = inpolygon(v(1),v(2),obs(1,:),obs(2,:));
    if q_in_obs && v_in_obs
        midpoint = 1e-8*(v-q)+q;
        [in_obs,on_obs] = inpolygon(midpoint(1),midpoint(2),obs(1,:),obs(2,:));
        if in_obs == 1 && on_obs == 0
            b =true;
        end
        midpoint = 1e-8*(q-v)+v;
        [in_obs,on_obs] = inpolygon(midpoint(1),midpoint(2),obs(1,:),obs(2,:));
        if in_obs == 1 && on_obs == 0
            b =true;
        end
    end  

end


