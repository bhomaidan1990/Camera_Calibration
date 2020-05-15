function P_infty = PlaneAtInfinity(W_inv)

global PPM;

% Symbolic variables

X =  sym('X', 'real');
Y =  sym('Y', 'real');
Z =  sym('Z', 'real');
L2 = sym('L2','real');

N = [X; Y; Z];
% Quadric
Q = [W_inv       ,   (W_inv * N)    ;
    (N' * W_inv) , (N' * W_inv * N)];
% Get one only camera matrix (any)
P = PPM(:, :, 3);

% Autocalibration equation
Calibration = P * Q * P';
% func = cross(W_inv, Calibration);
% solution = Optimize(zeros(3,3),@func);
% solve linear equations
solution = solve(Calibration(1, 1) == (L2 * W_inv(1, 1)), ...
    Calibration(2, 2) == (L2 * W_inv(2, 2)), ...
    Calibration(3, 3) == (L2 * W_inv(3, 3)), ...
    Calibration(1, 3) == (L2 * W_inv(1, 3)));

P_infty = [double(solution.X(1));...
    double(solution.Y(1));...
    double(solution.Z(1))];
end