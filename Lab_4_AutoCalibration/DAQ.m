% ===================================
% Dual Absolute Quadric Cost Function
% ===================================
function cost = DAQ(params)

Aj = [params(1)   params(5) params(2) ;
    
    0        params(3) params(4) ;
    0           0         1     ];

Ai = [params(1) params(5)   params(2) ;
    0      params(3)   params(4) ;
    0           0         1     ];
% W^-1 (IAC Image of the Absolute Conic)
W_inv = Ai * Aj';
%----------------
%Find plane at infinity from MQM' ~ w (Dual Absolute Quadric)
Plane_at_infinity = PlaneAtInfinity(W_inv);

%Find H_Infty = [e21]F+e21*n'
Homography_at_infty = H_Infty(Plane_at_infinity);
%----------------
% Initialization
%----------------
global Fs;
% Initialize the cost as a vector
% (N-1 * N-2)/2: 9*8/2 = 36
vector_size = (size(Fs,3)-1)*(size(Fs,4)-2)/2;
cost = zeros(1, vector_size);

% Cost Function

k = 0;
loop_size = 3 * vector_size;

Second_Term = W_inv / norm(W_inv,'fro');

for i=1:3:loop_size
    k = k+1;
    First_Term  = Homography_at_infty(:,i:i+2) * W_inv * ((Homography_at_infty(:,i:i+2))');
    First_Term  = First_Term / norm(First_Term, 'fro');

    cost(k) = norm(First_Term - Second_Term,'fro');

end
end

