function output = Reshape(input)
%---------------------
%  Reshape Intrinsics
%---------------------
% K = [a         skew       u0     ;
%      0           B        v0     ;
%      0           0        1     ];

output = [input(1) input(5) input(2) ;
    0       input(3)   input(4)   ;
    0           0         1    ];

end