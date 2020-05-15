function output = Optimize(A,func)
%------------------------------
% General Optimization options
%------------------------------
%------------------------------------------
% Nonlinear least-squares optimization
%------------------------------------------
% A is the initial guess.
% A(1,1) = a  = ku*f.
% A(2,2) = B  = kv*f.
% A(1,3) = u0.
% A(3,3) = v0.
% A(1,2) = skew
% f is the focal length.
% (ku,kv)magnification factors:
% the number of pixels per unit distance in u and v directions;
% C(u0,v0) is The principal point.

options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'Display','off','FunctionTolerance',1e-16,'Tolx',1e-16,...
    'MaxFunctionEvaluations', 1000, 'MaxIterations',39,...
    'OptimalityTolerance',1e-16);
%------------------------------
% NonLinear Optimization
%------------------------------
output_line = lsqnonlin(func,[A(1,1), A(1,3), A(2,2), A(2,3), A(1,2)],...
    [],[],options);
%------------------------------------------------------------------------
% A = [a         skew       u0      ;
%       0           B        v0     ;
%       0           0        1     ];
output = Reshape(output_line);
end