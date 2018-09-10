function [ B,r,t ] = randtrans( A )
%Function to randomly(ish) transform input matrix A to produce a "test"
%matrix B

theta = (2*rand-1)*60*pi/180; %rotation about z
gamma = (2*rand-1)*60*pi/180; %rotation about y
alpha = (2*rand-1)*60*pi/180; %rotation about x

[num, dim] = size(A);
t = randi([-1,1],dim,1);

Rz = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0,0,1];
Ry = [cos(gamma), 0, sin(gamma); 0,1,0; -sin(gamma), 0, cos(gamma)];
Rx = [1,0,0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
r = Rz*Ry*Rx;
B = r*A'+repmat(t,1,num);
B = B';

end

