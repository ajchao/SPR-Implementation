function [ R_p, T_p ] = perturb( B_tr,p,k,i )
% Generates random perturbations based on iteration within loop and specified
% num of perturbations, p

%Find max range of B_tr to use as its size
size = max(range(B_tr)); %avoid using functions as variable names
T_p = normrnd(0,(0.1*size*((i+1-k)/i)),[p,3]);

%generate random angles in degrees
%Around x
a = normrnd(0,(10*((i+1-k)/i)),[1,p]);
%Around y
b = normrnd(0,(10*((i+1-k)/i)),[1,p]);
%Around z
c = normrnd(0,(10*((i+1-k)/i)),[1,p]);

%convert to radians
a = a.*(pi()/180); %pi/180 -- can store as variable, then use variable instead of multiple computations for speed
b = b.*(pi()/180);
c = c.*(pi()/180);

%initialize R_p
R_p = zeros(3,3,p);

for j = 1:p
    %rotation matrix about x
    r1 = [1,0,0; 0, cos(a(j)), -sin(a(j)); 0, sin(a(j)), cos(a(j))]; %eul2rotm function (euler angles to rotation matrix
    %rotation matrix about y
    r2 = [cos(b(j)), 0, sin(b(j)); 0,1,0; -sin(b(j)), 0, cos(b(j))];
    %rotation matrix about z
    r3 = [cos(c(j)), -sin(c(j)), 0; sin(c(j)), cos(c(j)), 0; 0,0,1];
    
    %Perturbation Rotation
    R_p(:,:,j) = r1*r2*r3;
end
end

