function [camera,calibration,worldCoordinates]=calibrationMatrix(filename)
%This function allows the user to provide a image filename. It will then
%walk through the process of selecting image points and ask the user for
%the known 3D location of the point. At the end it calculates the
%calibration matrix and returns it. This will also save a copy of the
%calibration to file so that the same calibration does not need to be
%redone

inputImage=imread(filename);
evalResponse = input('How many calibration points are there?');

calibration=ones(3,evalResponse);
worldCoordinates=ones(4,evalResponse);

    
for i=1:evalResponse
    figure(i);
    imshow(inputImage);
    hold on
    if i>1
        scatter(calibration(1,1:i-1),calibration(2,1:i-1));
    end
    pause;
    [calibration(1,i),calibration(2,i)] = ginput(1);
    prompt = {'Enter X:','Enter Y:','Enter Z:'};
    answer = inputdlg(prompt,num2str(i),1,{'0','0','0'});
    close;
    if(size(answer,1)==3)
        worldCoordinates(1,i)=str2double(answer(1));
        worldCoordinates(2,i)=str2double(answer(2));
        worldCoordinates(3,i)=str2double(answer(3));
    else
        i=i-1;
    end
end
camera=cameraMatrix(worldCoordinates,calibration);
dlmwrite(strcat(filename,'.C.txt'),camera);
end
