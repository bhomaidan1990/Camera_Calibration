function H_Inf = H_Infty(planeInf)

global Fs;
k = 1;
% (3 x 3) x ((N-1)*(N-2) /2)
H_Inf = zeros(3,3*(size(Fs,3)-1)*(size(Fs,4)-2)/2);%(3*3)*36

for i = 2:size(Fs,3)
    for j = i+1:size(Fs,4)
        
        [~, ~, V]  = svd(Fs(:,:,i,j)');
        epip = V(:,end);
        
        H_Inf(:,k:k+2) =  epipole(Fs(:,:,i,j)) * Fs(:,:,i,j)+ epip * planeInf';
        k = k+3;        
    end
end       
end