function [ imageCoordinates,worldCoordinates ] = enterKnownShape( filename )
%ENTERKNOWNSHAPE Gives the image and realworld coordinates for a know shape
%   The two outputs from here can be used to calculate the position of the
%   shape. This should be called from the locate method which will
%   automatically generate the RTMatrix

inputImage=imread(filename);

%Corresponds to the corners of the metal block. Any known shape can be used
worldCoordinates = [0       0   -75  -75   -75 ;...
                    30      30  30    30    0 ;...
                    -47.6   0   0    -47.6  0 ;...
                    1       1   1      1    1];
                
numberOfPoints = size(worldCoordinates,2);

imageCoordinates = ones(3,numberOfPoints);

for i=1:numberOfPoints%evalResponse
    figure(i);
    imshow(inputImage);
    hold on
    if i>1
        scatter(imageCoordinates(1,1:i-1),imageCoordinates(2,1:i-1));
    end
    pause;
    [imageCoordinates(1,i),imageCoordinates(2,i)] = ginput(1);
    close;
end

end

