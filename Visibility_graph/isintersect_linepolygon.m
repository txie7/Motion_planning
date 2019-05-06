function b = isintersect_linepolygon(S,Q)
% S: 2x2 line segment
% Q: 2xN vertices

Q = [Q, Q(:,1)];% expand Q 
p0 = S(:,1);
p1 = S(:,2);

if p0 == p1 % S is a point
    b = inpolygon(p0(1),p0(2),Q(1,:),Q(2,:)); % Check if it is in polygon
    return
else
    tE = 0;
    tL = 1;
    ds = p1-p0;
    for i = 1:size(Q,2)-1
        % Compute the outward normal vector:
        z = [0 0 1];
        ni = cross(z,[(Q(:,i)-Q(:,i+1))' 0]);
        ni = ni/norm(ni);
        ni = ni(1:2)';
        
        N = -(p0-Q(:,i))'*ni;
        D = ds'*ni;
        if D == 0
            if N<0
                b = false;
                return
            end
        end
        t = N/D;
        if D < 0
            tE = max(tE,t);
            if tE > tL
                b = false;
                return
            end
        end
        if D > 0
            tL = min(tL,t);
            if tL<tE
                b = false;
                return
            end
        end
    end
    if tE<=tL
        b = true;
        return 
    else
        b = false;
        return 
    end
       
end

            
        