function [ E,K ] = getExternal( Calibration )
%This takes a input camera calibration matrix and decomposes it into its
%intrinsic parameter matrix as well as the external parameter matrix
    M=Calibration(1:3,1:3);
    [K,R]=rq(M);

    if(K(3,3)<0)
        K(:,3)=-K(:,3);
    end
    if(K(2,2)<0)
        K(:,2)=-K(:,2);
    end
    if(K(1,1)<0)
        K(1,1)=-K(1,1);
    end
    E=K\Calibration;
    E(4,4)=1;
end

