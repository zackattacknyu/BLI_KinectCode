function C=cameraMatrix(Pw,Pi)
%This function generates a camera calibration matrix when the 4xN world
%points and the 3xN image points are provided
   M=zeros(size(Pi,2)*2,12);
   i=1;
    if size(Pw,2)==size(Pi,2)
       for n=1:size(Pi,2)
           M(i,:)=[-Pw(1,n) -Pw(2,n) -Pw(3,n) -Pw(4,n) 0 0 0 0 Pi(1,n)*Pw(1,n) Pi(1,n)*Pw(2,n) Pi(1,n)*Pw(3,n) Pi(1,n)*Pw(4,n)];
           i=i+1;
           M(i,:)=[0 0 0 0 -Pw(1,n) -Pw(2,n) -Pw(3,n) -Pw(4,n) Pi(2,n)*Pw(1,n) Pi(2,n)*Pw(2,n) Pi(2,n)*Pw(3,n) Pi(2,n)*Pw(4,n)];
           i=i+1;
       end
    end
    C=zeros(3,4);
    [S,U,V]=svd(M);
    C(1,:)=[V(1,12)/V(12,12) V(2,12)/V(12,12) V(3,12)/V(12,12) V(4,12)/V(12,12)];
    C(2,:)=[V(5,12)/V(12,12) V(6,12)/V(12,12) V(7,12)/V(12,12) V(8,12)/V(12,12)];
    C(3,:)=[V(9,12)/V(12,12) V(10,12)/V(12,12) V(11,12)/V(12,12) V(12,12)/V(12,12)];
end