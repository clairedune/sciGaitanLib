function [M] = homogeneousMatrixFromPos(pose)
M = homogeneousMatrix(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));
M = M.* (abs(M)>%eps);     
endfunction
