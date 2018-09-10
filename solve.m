function [ t, r ] = solve( B, NN )
%Function that finds translation, t, and rotation, r, that minimizes error
%between point set B and it nearest neighbour set

num = length(B);

%Find centroids
cen_B = sum(B)/num;
cen_NN = sum(NN)/num;

%Set centroids as reference origin for both clusters
B2 = B-repmat(cen_B,num,1);
NN2 = NN-repmat(cen_NN,num,1);

%Calculate S-values -- refer to paper: ?Method for registration of 3-D
%shapes? by P. J. Besl and N. D. McKay
Sxx = B2(:,1)'* NN2(:,1);
Syy = B2(:,2)'* NN2(:,2);
Sxy = B2(:,1)'* NN2(:,2);
Syx = B2(:,2)'* NN2(:,1);

Szz = B2(:,3)'* NN2(:,3);
Szy = B2(:,3)'* NN2(:,2);
Szx = B2(:,3)'* NN2(:,1);
Syz = B2(:,2)'* NN2(:,3);
Sxz = B2(:,1)'* NN2(:,3);

%N-matrix
N = [ Sxx+Syy+Szz, Syz-Szy, -Sxz+Szx, Sxy-Syz;
    -Szy+Syz, Sxx-Szz-Syy, Sxy+Syx, Sxz+Szx;
    Szx-Sxz, Syx+Sxy, Syy-Szz-Sxx, Syz+Szy;
    -Syx+Sxy, Szx+Sxz, Szy+Syz, Szz-Syy-Sxx];

%find largest positive eval of N
[eval, idx] = max(eig(N));

%quaternion
[evec, eval] = eig(N);
q = evec(:,idx);

Qbar = [q(1), -q(2), -q(3), -q(4);
        q(2), q(1), q(4), -q(3);
        q(3), -q(4), q(1), q(2);
        q(4), q(3), -q(2), q(1)];

Q = [q(1), -q(2), -q(3), -q(4);
    q(2), q(1), -q(4), q(3);
    q(3), q(4), q(1), -q(2);
    q(4), -q(3), q(2), q(1)];

%rotation matrix:
r = Qbar'*Q;
r = r(2:4,2:4);

%translation matrix:
t = cen_NN - (r*cen_B')';

end

