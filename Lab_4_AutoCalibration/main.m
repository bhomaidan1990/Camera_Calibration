%=======================
% Author: Belal Hmedan
%=======================
%---------------------
% clear and close all
%---------------------
clearvars
close all
clc
%---------------------
% Data type long
%---------------------
format long g
%---------------------
% Read data
%---------------------
load('data.mat')
%---------------------------
% Display The Initial Guess
%---------------------------
disp('=======================================================')
disp('Initial Intrinsic parameters: ');
disp(A);
disp('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')

%=========================================================================
DualAbsoluteQuadric = Optimize(A,@DAQ);
%---------------------
% Display The Results
%---------------------
disp('=======================================================')
disp('Dual Absoute Quadric cost function: ');
disp(DualAbsoluteQuadric);
disp('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx')



