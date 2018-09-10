function [ B ] = transform( B,t,r )
%Function to transform matrix B -- translation t, rotation angle r

[num, dim] = size(B);

if dim == 2
    B = [cos(-r), sin(-r); -sin(-r), cos(-r)]*B'-repmat(t',1,num);
    B = B';
end

if dim == 3
    B = r*B' + repmat(t',1,num);
    B = B';

end

