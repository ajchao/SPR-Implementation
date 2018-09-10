function [ R_l, T_l, e_best ] = solvespr( R_p, T_p, B_tr, A, p, Mdl )
%This function takes the generated random perturbations and performs ICP on
%each configuration, returning the resulting orientation with the lowest
%error (distance from B_tr to its NN cloud)

%error tracker:
e_best = inf;
R_l = eye(3);
T_l = [0,0,0];

for j = 1:p+1
    
    %orient matrix to random perturbation
    %tracking variables (to avoid running errors
    a1 = R_p(:,:,j);
    a2 = T_p(j,:);
    
    C = (a1*(B_tr'))' + repmat(a2,length(B_tr),1);
    
    %Solve ICP of C to A
    [t, r, e] = solveicp(C, A, Mdl);
    
    if e < e_best
        e_best = e;
        R_l = r;
        T_l = t;
    end

end

