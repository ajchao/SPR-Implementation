function [ TR, RO, e ] = solveicp( B, A, Mdl )
%Function takes in A and B clusters, registers B to A using ICP, returns
%resultant transformation matrix and error calculation

k = 0;
C_tr = B;
idx = knnsearch(A,B);
NN = A(idx,:);
%NN = nn2(C_tr, Mdl, A);
TR = [0,0,0];
RO = eye(3);
e = sum(sqrt(sum((NN-C_tr).^2,2)))/length(C_tr);
%e = err(C_tr,NN);

while e > 0.0001 && k < 30
    
    %NN = nn2(C_tr, Mdl, A);
    %NN = nn(C_tr, A);
    idx = knnsearch(Mdl,C_tr);
    NN = A(idx,:);
    
    [T, R] = solve(C_tr,NN);
    
    TR = TR+T;
    RO = RO*R;
    C_tr = transform(B,TR,RO);
    e = sum(sqrt(sum((NN-C_tr).^2,2)))/length(C_tr);
    %e = err(C_tr,NN);
    k = k+1;
end

end

