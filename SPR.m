close all; 
clear;
clc;

%Import and replace 3-D point clouds: A is the geometric model, B_org is sensor measurements
A=dlmread('bunny_S.txt');

%sample A to create B
id = randi([1,length(A)],1,25);
B_org = A(1:30,:);

% Randomly transformed point cloud B ("collected test data")
%B_org = original B point cloud of randomly selected and randomly
%transformed points
%r_true = true rotation B_org underwent from A
%t_true = true translation B_org underwent from A
[B_org, r_true, t_true] = randtrans(B_org);
B_tr = B_org; %tracking transformations of B_org

%kdtree model
Mdl = KDTreeSearcher(A); %unsure if kdtree is implemented correctly...

%nearest neighbour cluster
%NN = nn2(B_tr, Mdl, A);
idx = knnsearch(A,B_tr);
NN = A(idx,:);

%Number of pertubations:
p = 10;

%Max iterations:
i = 30;

%Error threshold goal:
rms = 1e-6;

%Setting up variables to be used in the while loop:
r = eye(3);
%e = err(B_tr, NN);
e = sum(sqrt(sum((NN-B_tr).^2,2)))/length(B_tr);
k = 1;

%first align centroids of A and B before entering SPR loop
cen_B = sum(B_org)/length(B_org);
cen_A = sum(A)/length(A);
t = cen_A - cen_B;
B_tr = B_org + repmat(t,length(B_org), 1);

%Loop
while e > rms && k < i
    %1. Perturbations
    [R_p, T_p] = perturb(B_tr,p,k,i);
    R_p(:,:,p+1) = eye(3);
    T_p(p+1,:) = [0,0,0];
    
    %2. Find local optimum
    [R_l, T_l, e] = solvespr(R_p, T_p, B_tr, A, p, Mdl);
    
    %3. Add to overall rotation and translation
    t = t+T_l;
    r = r*R_l;
    
    %4. Change B_tr to most recent optimal:
    B_tr = transform(B_org, t, r);
    
    %increase k
    k = k+1;
end

%r and t are the resultant rotation and translation to register B to A

%% Visualize

figure

%subplot(1,2,1)
scatter3(A(:,1), A(:,2), A(:,3),'.r')
hold on
scatter3(B_org(:,1), B_org(:,2), B_org(:,3),'b','fill')
scatter3(B_tr(:,1), B_tr(:,2), B_tr(:,3),'k','fill')
axis equal