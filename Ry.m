function out = Ry(theta)
    out = [cos(theta) 0 sin(theta);
           0 1 0;
           -1*sin(theta) 0 cos(theta)];
end