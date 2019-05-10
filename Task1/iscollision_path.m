function b = iscollision_path(q, q_next,serial_link, C_obs)

% interpolation to check whether the path is collision-free
delta_q = (q_next - q)/50;
b = false;
for i = 1:50
    temp = q + delta_q*i;
    b = isintersect_seriallink_obs(temp,serial_link,C_obs);
    if b
        break
    end
end
end