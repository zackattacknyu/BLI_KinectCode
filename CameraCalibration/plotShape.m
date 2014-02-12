function plotShape( c,M,shape,imageFile )
%This can be used to visually verify that the RT matrix M is valid. You
%just provide the locations for the 3D points as a 4xN matrix shape. Also
%provide the imageFile that you want to plot the points onto. 

imagePoints = c*M*shape;
imagePoints(1,:)=imagePoints(1,:)./imagePoints(3,:);
imagePoints(2,:)=imagePoints(2,:)./imagePoints(3,:);
imagePoints(3,:)=imagePoints(3,:)./imagePoints(3,:);
image(imread(imageFile))
hold on
scatter(imagePoints(1,:),imagePoints(2,:));
end

