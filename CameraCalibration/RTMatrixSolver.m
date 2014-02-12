function [M] = RTMatrixSolver( camera, imagePoints, worldPoints )
%This is the solver that determines the translation and rotation matrix
%needed to align a known shape to is corresponding image
%The camera should be the 3x4 camera matrix for the current camera
%imagePoints should be the 3xN matrix corresponding to X,Y,1 for each point
%world Points should be the 4xN matrix corresponding to X,Y,Z,1 for each
%starting location on the known shape


    M = zeros(4);
    options = optimset('MaxFunEvals',100000,'MaxIter',100000);
  
    %Uses the quadratic solver with the function shown below
    fsolve(@test,[0 0 0 0 0 0],options);
    
    function out = test(Q)
        %The first three inputs are the x, y, and z angles for the rotation
        %of the object
        %The next three inputs are the x, y, and z translations
        
        Q(1:3)= Q(1:3)./180*pi;
        rX = [1         0           0           0;...
              0         cos(Q(1))   -sin(Q(1))  0;...
              0         sin(Q(1))   cos(Q(1))   0;...
              0         0           0           1];
          
        rY = [cos(Q(2)) 0           sin(Q(2))   0;...
              0         1           0           0;...
              -sin(Q(2)) 0          cos(Q(2))   0;...
              0         0           0           1];
          
        rZ = [cos(Q(3)) -sin(Q(3))  0           0;...
              sin(Q(3)) cos(Q(3))   0           0;...
              0         0           1           0;...
              0         0           0           1];          
        T = [1 0 0 Q(4);...
            0 1 0 Q(5);...
            0 0 1 Q(6);...
            0 0 0 1];
        
        %M is the current version of the translation and rotation matrix
        M = T*rZ*rY*rX;
        
        %This is where the current points would be positioned in the image
        %given the current M and camera matrix
        calcImagePoints = camera*M*worldPoints;
        
        %This is the error that we are trying to minimize in this solver
        out = zeros(size(imagePoints,2)*2,1);
        
        for i=1:size(calcImagePoints,2);
            calcImagePoints(:,i) = calcImagePoints(:,i)./calcImagePoints(3,i);
            out(i*2-1) = (calcImagePoints(1,i)-imagePoints(1,i))^2;
            out(i*2) = (calcImagePoints(2,i)-imagePoints(2,i))^2;
        end
    end
end


