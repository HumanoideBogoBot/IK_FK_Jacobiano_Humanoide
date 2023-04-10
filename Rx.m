function out = Rx(theta)
    out = [1 0 0;
           0 cos(theta) -1*sin(theta);
           0 sin(theta) cos(theta)];
end