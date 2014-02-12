function [ VertexCoordinates,images ] = findFiducial( objFile )
%The goal of this file is allow the uses to provide an obj file and then
%select the fiducial on a corresponding image. This will this determine the
%location of the closest vertex to the selected point

%NOT COMPLETED

%%%%%%%Process the mtl file here%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fid = fopen([objFile '.mtl']);

fgetl(fid);
fgetl(fid);
fgetl(fid);

line = fgetl(fid);
images={};
while(ischar(line))
    fgetl(fid);% Ka 0.2 0.2 0.2
    fgetl(fid);% Kd 0.8 0.8 0.8
    fgetl(fid);% Ks 1 1 1
    fgetl(fid);% d 1
    fgetl(fid);% Ns 75
    fgetl(fid);% illum 2
    im = textscan(fgetl(fid),'%*s %s');% map_Kd 3.png
    fgetl(fid);% ###
    images(end+1,:)=[im{1} {0}];
    line = fgetl(fid);
end

%%%%%%%Process the OBJ file here%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fid = fopen([objFile '.obj']);

fgetl(fid);
fgetl(fid);
vertices = textscan(fgetl(fid),'%*s %*s %d');
faces = textscan(fgetl(fid),'%*s %*s %d');
fgetl(fid);
fgetl(fid);
fgetl(fid);
fgetl(fid);
VertexCoordinates = zeros(vertices{1},3);
for i=1:vertices{1}
    t=textscan(fgetl(fid),'%*s %f %f %f');
    VertexCoordinates(i,:)=[t{1} t{2} t{3}];
end
for i=1:vertices{1}+1
    fgetl(fid);
end
for j=1:size(images,1)
    textures = textscan(fgetl(fid),'%*s %d %*s %*s %*s %*s %*d');
    textures
    TextureValues = zeros(textures{1},2);
    images(j,2)= textures;
    for i=1:textures{1}
        t=textscan(fgetl(fid),'%*s %f %f');
        TextureValues(i,:)=[t{1} t{2}];
    end
end

end

