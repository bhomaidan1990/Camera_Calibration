%=========================================
% Epipole Function
%=========================================
function epip = epipole(Fs)
% SVD Decompostition of (Fs)^T
[~,~,V]  = svd(Fs');
% Get the epipole from the last vector of the SVD
epi = V(:,end);
% Reshape the Vector into Matrix
epip = [ 0     -epi(3)  epi(2);
    epi(3)   0     -epi(1);
    -epi(2)  epi(1)    0  ];

end