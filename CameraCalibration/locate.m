function [ M ] = locate( camera, image )
%LOCATE Summary of this function goes here
%   Detailed explanation goes here
[iC,wC] = enterKnownShape(image);

[M] = RTMatrixSolver(camera,iC,wC);
end

