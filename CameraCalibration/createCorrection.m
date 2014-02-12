function [ Correction,KinInt,OtherInt ] = createCorrection( )
%This was designed to create a correction to move from one camera to
%another

Images{1,1}=uigetfile({'*.jpg;*.png'},'Kinect Color Image');

Images{1,2}=uigetfile({'*.jpg;*.png'},'Other Camera Color Image');

for i=1:2
    filename=[Images{1,i} '.C.txt'];
    if exist(filename,'file')==0
        Calibration(:,:,i)=calibrationMatrix(Images{1,i});
        dlmwrite(filename,Calibration(:,:,i));        

    else
        Calibration(:,:,i)=dlmread(filename);
    end



end

[KinExt,KinInt] = getExternal(Calibration(1:3,1:4,1));
[OtherExt,OtherInt] = getExternal(Calibration(1:3,1:4,2));


Correction=KinExt/OtherExt;%KinExt*inv(OtherExt)
Correction(1:3,4)=Correction(1:3,4)./1000;
dlmwrite('correction.txt',Correction);
end