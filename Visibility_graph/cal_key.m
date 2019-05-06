function key = cal_key(l1, l2)
% l1 (pq) and l2 (mn) are two lines 
% The key is the distance between point p and the intersection
p = l1(:,1);
q = l1(:,2);
m = l2(:,1);
n = l2(:,2);

if norm(m-p)==0 || norm(n-p)== 0 
    key = 0;
else
    theta_m = acos((m-p)'*(q-p)/(norm(m-p)*norm(q-p)));
    y_m = norm(m-p)*sin(theta_m);

    theta_n = acos((n-p)'*(q-p)/(norm(n-p)*norm(q-p)));
    y_n = norm(n-p)*sin(theta_n);
    % y_m and y_n should be >= 0
    if y_m == 0 && y_n ==0
        key = min(norm(m-p),norm(n-p));
    else
        x_intersect = y_m/(y_m+y_n)*(n(1)-m(1))+m(1);
        y_intersect = y_m/(y_m+y_n)*(n(2)-m(2))+m(2);

        key = norm([x_intersect;y_intersect]-p);
    end
end

key = round(key,4); % round for comparison

end
 


