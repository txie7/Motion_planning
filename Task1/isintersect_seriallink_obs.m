function collision = isintersect_seriallink_obs(joint_angles,serial_link, C_obs)

n = serial_link.DoF;
origin = serial_link.Origin;
link_length = serial_link.Link_length;

l = zeros(2,2,n);
r1 = rotz(joint_angles(1));
t1 = r1*[link_length(1);0;0];
l(:,:,1) = [origin, t1(1:2)+ origin];
 
for i  = 2:n
    r = rotz(joint_angles(i));
    t = r*[link_length(i);0;0];
    l(:,:,i) = [l(:,2,i-1),t(1:2)+l(:,2,i-1)];
end
collision = false;
for i = 1: length(C_obs)
    obs = cell2mat(C_obs(i));
    collision = false;
    for j = 1:n
        collision = isintersect_linepolygon(l(:,:,j),obs);
        if collision 
            break
        end
    end
    if collision
        break
    end
end
        

end