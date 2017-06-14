% testing 3D optical flow

close all; clear all; clc

saveVolume = true;

% compute flow

optFlow(15) = struct('ux',[],...
                 'uy',[],...
                 'uz',[],...
                 'vol',[]);

hWait = waitbar(0, 'Computing optical flow.');
nVols = 15;
             
fileName = 'volume_20160624_160455_160722_133710_%d.mat';
for i = 1:nVols
    msg = sprintf('Computing optical flow. Step %d of %d',i,nVols);
    waitbar((i-1)/nVols, hWait, msg);
    
    vol1 = load( sprintf( fileName, i ) );
    vol1 = vol1.CdZeroed;
    
    vol2 = load( sprintf( fileName, mod(i,15) + 1 ) ); % wrap around
    vol2 = vol2.CdZeroed;
        
    [ux,uy,uz] = HS3D(vol1, vol2, 1, 10); % smoothing, iterations

    optFlow(i).ux = ux;
    optFlow(i).uy = uy;
    optFlow(i).uz = uz;
    optFlow(i).vol = vol1;
    
end
close(hWait)

%%
% Enhance the quiver plot visually by downsizing vectors
%   -f : downsizing factor
f=3;
    
opengl hardware
h = figure;
set(h,'renderer','opengl');
set(h, 'color', 'w')

for i = 1:nVols
    volIdx = i;
    ux = optFlow(volIdx).ux;
    uy = optFlow(volIdx).uy;
    uz = optFlow(volIdx).uz;
    
    x = ux(1:f:size(ux,1), 1:f:size(ux,2), 1:f:size(ux,3));
    y = uy(1:f:size(uy,1), 1:f:size(uy,2), 1:f:size(uy,3));
    z = uz(1:f:size(uz,1), 1:f:size(uz,2), 1:f:size(uz,3));
    
    [X,Y,Z] = meshgrid(1:size(x,2), 1:size(x,1), 1:size(x,3));
    q = quiver3(X,Y,Z,x,y,z,10); %axis([1 size(x,2) 1 size(x,1) 1 size(x,3)]);
    
    %// Compute the magnitude of the vectors
    mags = sqrt(sum(cat(2, q.UData(:), q.VData(:), ...
        reshape(q.WData, numel(q.UData), [])).^2, 2));
    
    %// Get the current colormap
    currentColormap = colormap(gca);
    
    %// Now determine the color to make each arrow using a colormap
    [~, ~, ind] = histcounts(mags, size(currentColormap, 1));
    
    %// Now map this to a colormap to get RGB
    cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
    cmap(:,:,4) = 255;
    cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);
    
    %// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
    set(q.Head, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'
    
    %// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
    set(q.Tail, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:2,:,:), [], 4).');

%     % plot US data
%     vol = optFlow(volIdx).vol;
%     [Xv,Yv,Zv] = meshgrid(1:size(vol,2), 1:size(vol,1), 1:size(vol,3));
%     nonZeroIdx = vol > 1;
%     Xv = Xv(nonZeroIdx);
%     Yv = Yv(nonZeroIdx);
%     Zv = Zv(nonZeroIdx);
%     vol = vol(nonZeroIdx);
%     s = scatter3(Xv(:),Yv(:),Zv(:),1,vol(:));
%     alpha(s,0.5);

    
    pause(0.5)
    
    delete(q)
    %delete(s)
end

%% save the flow volumes
% if(saveVolume)
%     for i = 1:nVols
%         newFileName = sprintf( fileName, i );
%         newFileName = [newFileName(1:end-4), '_flow'];
%         flowVolume = optFlow(i).vol; % !!! not the correct data, need to generate this based on vectors
%         save([newFileName,'.mat'], 'flowVolume');
%         
%         saveStitched2RawFile(flowVolume, [newFileName,'.raw']);
%     end
% end

return
%% Test the nonrigid toolbox by D.Kroon
figure

% Register the images
Ireg = image_registration(optFlow(2).vol,optFlow(1).vol);

% Show the results
%showcs3(optFlow(2).vol);
%showcs3(optFlow(1).vol);
showcs3(Ireg);

%% Visualize Deformed Volume

data = smooth3(Ireg,'box',5);
patch(isocaps(data,.5),...
   'FaceColor','interp','EdgeColor','none');
p1 = patch(isosurface(data,.5),...
   'FaceColor',[0.5,0.5,0.5],'EdgeColor','none');
isonormals(data,p1)
view(3); 
axis vis3d tight
camlight left; 
colormap bone
lighting gouraud

%% Look at the Optical Flow of the Registered (Deformed) Volume

vol1 = optFlow(1).vol;
vol2 = Ireg;

[ux,uy,uz] = HS3D(vol1, vol2, 1, 10); % smoothing, iterations

ux = ux - optFlow(1).ux;
uy = uy - optFlow(1).uy;
uz = uz - optFlow(1).uz;

x = ux(1:f:size(ux,1), 1:f:size(ux,2), 1:f:size(ux,3));
y = uy(1:f:size(uy,1), 1:f:size(uy,2), 1:f:size(uy,3));
z = uz(1:f:size(uz,1), 1:f:size(uz,2), 1:f:size(uz,3));

[X,Y,Z] = meshgrid(1:size(x,2), 1:size(x,1), 1:size(x,3));
q = quiver3(X,Y,Z,x,y,z,10); %axis([1 size(x,2) 1 size(x,1) 1 size(x,3)]);

%// Compute the magnitude of the vectors
mags = sqrt(sum(cat(2, q.UData(:), q.VData(:), ...
    reshape(q.WData, numel(q.UData), [])).^2, 2));

%// Get the current colormap
currentColormap = colormap(gca);

%// Now determine the color to make each arrow using a colormap
[~, ~, ind] = histcounts(mags, size(currentColormap, 1));

%// Now map this to a colormap to get RGB
cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
cmap(:,:,4) = 255;
cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);

%// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
set(q.Head, ...
    'ColorBinding', 'interpolated', ...
    'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'

%// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
set(q.Tail, ...
    'ColorBinding', 'interpolated', ...
    'ColorData', reshape(cmap(1:2,:,:), [], 4).');