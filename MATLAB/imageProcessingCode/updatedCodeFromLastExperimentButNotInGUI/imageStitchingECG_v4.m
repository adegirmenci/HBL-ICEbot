clear all; close all; clc;

% Written by Alperen Degirmenci
% Last updated : 6/22/16


% v1: Used in the first pig trial
% v2: Works with ECG data collected using the C++ GUI, for trial #2
% v3: Trying out other methods of interpolation - take average of pixels in
%     each bin, then do interpolation, instead of passing everything to the
%     interpolator
% v4: Binning + cosine transform inpainting 

nStudy = 25;

nPerSweep = 150; % 120 % frames per sweep

im_source = 'sequoia_size2_800x600';%'sequoia_size3'; % 'acuson', 'sequoia_size3'

% Save threshed or not
save_threshed = false;
doConvHull = false;
doPreThreshold = false;
preThresh = 0;
reconstrScale = 2.0; % higher numbers reduce the size of the reconstructed volume

plotVolume = 0; % show 3D volume at the end

% ----------------------------------------------------------------------- %
% ---------------- DON'T EDIT WITHOUT CONSULTING ALPEREN ---------------- %
% ----------------------------------------------------------------------- %

if(nStudy < 10)
    dataLoc = ['../00',num2str(nStudy),'/'];
    imLoc = ['../00',num2str(nStudy),'cropped/'];
elseif(nStudy < 100)
    dataLoc = ['../0',num2str(nStudy),'/'];
    imLoc = ['../0',num2str(nStudy),'cropped/'];
else
    error('nStudy too big')
end

sweep_BB_files = dir([dataLoc, '*_BB_CT.mat']);
EM_time_files = dir([dataLoc, '*_time.mat']);
EM_files = dir([dataLoc, 'e3study_data_*.mat']);

if(isempty(sweep_BB_files))
    error('sweep_BB_files does not exist here')
elseif(length(sweep_BB_files) ~= 1)
    error('Too many sweep_BB_files')
end

if(isempty(EM_time_files))
    error('EM_time_files does not exist here')
elseif(length(EM_time_files) ~= 1)
    error('Too many EM_time_files')
end

if(isempty(EM_files))
    error('EM_files does not exist here')
elseif(length(EM_files) ~= 1)
    error('Too many EM_files')
end

% ----------------------------------------------------------------------- %
% -------------- SERIOUSLY, DON'T EVEN THINK ABOUT IT! :) --------------- %
% ----------------------------------------------------------------------- %

stitch = load([dataLoc, sweep_BB_files(1).name]);
EM_data = load([dataLoc, EM_files(1).name]);
stitch.EM_epoch = EM_data.e3study.timestamp;
EM_time = load([dataLoc, EM_time_files(1).name]);
stitch.time_EM = EM_time.time;

if( ~isequal(length(stitch.time_EM),size(stitch.T_BB_CT,3)) )
    error('EM data and time length mismatch')
end

% get 4x4 - not needed anymore, we are saving the MAT file as 4x4xN
% stitch.imageLocEM4x4 = getImage4x4(stitch.T_BB_CT); % crystal location in BB

%images = dir([imLoc, '*.jpg']); % get image file names
[images, EM_timeIdx, ECG_time, EM_timePctg] = selectFramesECG3(dataLoc, imLoc, ...
                                nPerSweep, stitch.time_EM, stitch.EM_epoch);

stitch.imageLocEM4x4 = cell(length(images),1);
for i = 1:length(images)
    % interpolate
    em1 = stitch.T_BB_CT(:,:,EM_timeIdx(i));
    em2 = stitch.T_BB_CT(:,:,EM_timeIdx(i)+1);
    emInt = trinterp(em1,em2,EM_timePctg(i));
    
    stitch.imageLocEM4x4{i} = emInt;
end

% define limits of stitch
stitch.startFrame = 1;%1;
stitch.endFrame = length(images);
stitch.nFrameJump = 1;
stitch.frameIdx = stitch.startFrame:stitch.nFrameJump:stitch.endFrame;
stitch.endFrame = stitch.frameIdx(end);
stitch.nFrames = length(stitch.frameIdx);

% initialize container
stitch.imageOriginal = cell(stitch.nFrames,1);

j = 1;
for i = stitch.startFrame:stitch.nFrameJump:stitch.endFrame
    fileName = images(i).fileName;

    % Load image to memory
    stitch.imageOriginal{j} = imread([imLoc,fileName]);
    
    imH = size(stitch.imageOriginal{j},1);
    imW = size(stitch.imageOriginal{j},2);
    if(j == 1)
        % get frame size
        stitch.imHeightOrig = imH;
        stitch.imWidthOrig = imW;
    else
        % check image size
        if(~(stitch.imHeightOrig == imH) || ~(stitch.imWidthOrig == imW) )
            error('Images have different sizes')
        end
    end
    
    j = j + 1;  
end
fprintf('Loaded images\n')

figure
for i = 1:5
subplot(2,3,i); imshow(stitch.imageOriginal{i})
end

%%

% calculate pixel size
stitch.usPlaneLength = 76.6-3.9; %76.0; % mm
stitch.pixSize = stitch.usPlaneLength/stitch.imHeightOrig; % mm/pix
% generate image to EM transformation matrix
stitch.T_CT_IMG = [0 1 0 0;...
            0 0 -1 0;...
           -1 0 0 stitch.imWidthOrig/2.0*stitch.pixSize;...
            0 0 0 1];

% initialize container 
stitch.imageLocXYZval = cell(stitch.nFrames,1);
Npixels = stitch.imHeightOrig * stitch.imWidthOrig; % N pixels
%[X,Y] = meshgrid(0:stitch.imHeightOrig-1,0:stitch.imWidthOrig-1); % mesh of img idx
[X,Y] = meshgrid(0:stitch.imWidthOrig-1,0:stitch.imHeightOrig-1); % mesh of img idx

load('mask2.mat')
% Sequoia Size 4
if( strcmp(im_source, 'sequoia_size4') )
    load('mask_size4.mat'); % load mask saved using createMask
end
% Sequoia Size 3
if( strcmp(im_source, 'sequoia_size3') )
    load('mask_size3.mat'); % load mask saved using createMask
end
Xvec = X(:);
Yvec = Y(:);
maskVec = logical(mask(:));
XvecNo0 = Xvec(maskVec);
YvecNo0 = Yvec(maskVec);
NpixelsNo0 = sum(maskVec);

j = 1;
for i = stitch.startFrame:stitch.nFrameJump:stitch.endFrame
    % prepare container
%     stitch.imageLocXYZval{j} = zeros([Npixels,4]); % initialize N-by-4
%     stitch.imageLocXYZval{j}(:,1) = X(:)*stitch.pixSize; % scale to mm
%     stitch.imageLocXYZval{j}(:,2) = Y(:)*stitch.pixSize; % scale to mm
    stitch.imageLocXYZval{j} = zeros([NpixelsNo0,4]); % initialize N-by-4
    stitch.imageLocXYZval{j}(:,1) = XvecNo0*stitch.pixSize; % scale to mm
    stitch.imageLocXYZval{j}(:,2) = YvecNo0*stitch.pixSize; % scale to mm
    % Z is zero
    tmp = stitch.imageOriginal{j};
    % insert image
%     stitch.imageLocXYZval{j}(:,4) = tmp(:);
    tmp = tmp(:);
    stitch.imageLocXYZval{j}(:,4) = tmp(maskVec);
    
    % register images based on EM
    tmp = stitch.imageLocXYZval{j}';
    tmp(4,:) = 1; % put it in [x;y;z;1] format
    tmp = stitch.imageLocEM4x4{i}*stitch.T_CT_IMG*tmp;
    tmp = tmp';
    stitch.imageLocXYZval{j}(:,1:3) = tmp(:,1:3);
%     tmp = [stitch.imageLocXYZval{j}(:,1:3),zeros(Npixels,1)]*(stitch.imageLocEM4x4{i}');
%     stitch.imageLocXYZval{j}(:,1:3) = tmp(:,1:3);
    j = j + 1;
end

fprintf('Transformed images\n')

%% Combine all points

% xyz = zeros([Npixels*stitch.nFrames,3]);
% c = zeros([Npixels*stitch.nFrames,1]);
% 
% for j = 1:stitch.nFrames
%     xyz((j-1)*Npixels+1:j*Npixels,:) = stitch.imageLocXYZval{j}(:,1:3);
%     c((j-1)*Npixels+1:j*Npixels,:) = stitch.imageLocXYZval{j}(:,4);
% end
xyz = zeros([NpixelsNo0*stitch.nFrames,3]);
c = zeros([NpixelsNo0*stitch.nFrames,1]);

for j = 1:stitch.nFrames
    xyz((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,1:3);
    c((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,4);
end

% transform back
% rotmat = stitch.imageLocEM4x4{stitch.startFrame}(1:3,1:3);
% xyz = xyz*rotmat;
% tmp = xyz';
% tmp = [tmp;ones(1,length(tmp))];
% tmp = stitch.T_CT_IMG*tmp;
% tmp = tmp';
% xyz = tmp(:,1:3);

%% Do thresholding beforehand

if(doPreThreshold)
    c(c < preThresh) = 0;
end

%% Discretize workspace
xyzmin = floor(min(xyz)); % in mm
xyzmax = ceil(max(xyz)); % in mm

step = reconstrScale*stitch.pixSize;
% x_disc = xyzmin(1):step:(xyzmax(1)+step);
% y_disc = xyzmin(2):step:(xyzmax(2)+step);
% z_disc = xyzmin(3):step:(xyzmax(3)+step);
x_disc = (xyzmin(1)):step:(xyzmax(1));
y_disc = (xyzmin(2)):step:(xyzmax(2));
z_disc = (xyzmin(3)):step:(xyzmax(3));

nElems_x = length(x_disc);
nElems_y = length(y_disc);
nElems_z = length(z_disc);

[Xd,Yd,Zd] = meshgrid(x_disc,y_disc,z_disc);
%Cd = zeros(size(Xd));
% 
% %% Find convex hull and pad with zeros
% XYZd = [Xd(:),Yd(:),Zd(:)]; % test points
% 
% if(doConvHull)
%     convHullIdx = convhull(xyz(:,1),xyz(:,2),xyz(:,3),'simplify', true);
%     
%     % find blob
%     in = inhull(XYZd,xyz,convHullIdx);
%     in3 = reshape(in,size(Xd,1),size(Xd,2),size(Xd,3));
%     
%     % might need to dilate by one
%     % W = 2;
%     % dilatedEdges = imdilate(in3,strel('square', W) );
%     
%     % find shell
%     BW2 = bwperim(in3); % try 18?
%     xyzShell = XYZd(BW2(:),:);
%     
%     % add these points to observations
%     xyz = cat(1,xyz,xyzShell);
%     c = cat(1, c, zeros(size(xyzShell,1),1));
% end

%% Compute interpolation
% Find max pixel value in each bin
observations = [xyz,c];
volume = interpolateAvg_v2(observations,step);

fprintf('Computing griddata...')
tic
Cd = inpaintn(volume,20);
% Interpolate scattered data
%Cd = griddata(xyz(:,1),xyz(:,2),xyz(:,3),c,Xd(:),Yd(:),Zd(:));
% Cd = griddata(xyz(:,1),xyz(:,2),xyz(:,3),c,Xd,Yd,Zd);
% Cd = griddata(volume(:,1),volume(:,2),volume(:,3),volume(:,4),Xd,Yd,Zd);
%try natural/nearest instead of linear
toc
%% Threshold
% thresh = 50;
% idx = find(c > thresh);
% c_thresh = c(idx);
% xyz_thresh = xyz(idx,:);
% showPointCloud(xyz_thresh,c_thresh)
% colormap('gray')
% set(gcf,'color','black')
% box off
% grid off
% axis off
% fprintf('Plotting point cloud\n')

CdZeroed = Cd;
CdZeroed(isnan(CdZeroed)) = 0;

thresh = isovalue(CdZeroed);
if thresh < 0 
    thresh = 0;
end
Cd_ = CdZeroed(:);
%idx = find( ((Cd_ > thresh) & ~isnan(Cd_)) & (Cd_ < (256-thresh)) );
idx = find( ((Cd_ >= thresh) & ~isnan(Cd_)) & (Cd_ <= 255) );
Cd_thresh = Cd_(idx);
XYZd = [Xd(:),Yd(:),Zd(:)];
XYZd_thresh = XYZd(idx,:);

% showPointCloud(XYZd_thresh,Cd_thresh)
% colormap('gray')
% set(gcf,'color','black')
% % set(gcf,'Renderer','OpenGL')
% box off
% grid off
% axis off
%%

%CdZeroed = rot90(Cd,2);
% CdZeroed = Cd;
%CdZeroed(CdZeroed < thresh) = 0;
% CdZeroed(isnan(CdZeroed)) = 0;

CdZeroedThreshed = CdZeroed;
CdZeroedThreshed(CdZeroedThreshed < thresh) = 0;
CdZeroedThreshed(CdZeroedThreshed > 255) = 255;

% showcs3(CdZeroedThreshed/max(max(max(CdZeroedThreshed))))

%% Marching cubes

if(plotVolume)
    fprintf('Plotting mesh\n')
    % iso = isovalue(CdZeroed)-30;
    % if (iso < 0)
    %     iso = 0;
    % end
    [f,v] = MarchingCubes(Xd(:,:,:),Yd(:,:,:),Zd(:,:,:),CdZeroed(:,:,:),thresh);
    view(94,-58)
    axis on
    xlabel('x');ylabel('y'),zlabel('z')
    hold on
    %plot3(29.8,5.6,81.7,'b*') % 002
    %plot3(39,24.3,70.2,'b*') % 003
    %plot3(28.8,26.2,62.3,'b*') % 004
    %plot3(44.5,24.8,52.4,'b*') % 005
    %plot3(31.3,26.2,82.1,'b*') % 006
    %plot3(26.5317,26.7383,83.6301,'b*') % 007
    
    set(gcf,'Renderer','OpenGL')
    
    % define axes
    q0 = [0;0;0;1];
    qx = [15;0;0;1];
    qy = [0;15;0;1];
    qz = [0;0;15;1];
    
    % plot EM
    for i = 1:stitch.nFrames
        xcoord = stitch.imageLocEM4x4{stitch.frameIdx(i)}*[q0,qx];
        ycoord = stitch.imageLocEM4x4{stitch.frameIdx(i)}*[q0,qy];
        zcoord = stitch.imageLocEM4x4{stitch.frameIdx(i)}*[q0,qz];
        h1 = plot3(xcoord(1,1:2),xcoord(2,1:2),xcoord(3,1:2),'b','LineWidth',3);
        h2 = plot3(ycoord(1,1:2),ycoord(2,1:2),ycoord(3,1:2),'g','LineWidth',3);
        h3 = plot3(zcoord(1,1:2),zcoord(2,1:2),zcoord(3,1:2),'k','LineWidth',3);
    end
    %legend([h1,h2,h3],'X','Y','Z')
    view(-123,20)
    
    axis equal
end

%% Save to RAW file for Volume Rendering

if(nStudy < 10)
    outLoc = ['../00',num2str(nStudy),'_output/'];
elseif(nStudy < 100)
    outLoc = ['../0',num2str(nStudy),'_output/'];
else
    error('nStudy too big')
end

if ~isdir(outLoc)
    mkdir(outLoc);
end

dt = datestr(datetime('now'),'yymmdd_HHMMss');
outfilePre = [outLoc,'volume_00',num2str(nStudy),dt];

% save as .RAW
volumeFileName = [outfilePre,'.raw'];
saveStitched2RawFile(CdZeroed, volumeFileName)
if(save_threshed)
    saveStitched2RawFile(CdZeroedThreshed, volumeFileName);
end

% save as .mat
save([outfilePre,'.mat'],'CdZeroed')
if(save_threshed)
    save([outfilePre,'.mat'],'CdZeroedThreshed')
end

% save volume size to text file
[xn,yn,zn] = size(CdZeroed);
if(save_threshed)
    [xn,yn,zn] = size(CdZeroedThreshed);
end
fileID = fopen([outfilePre,'.txt'],'w');
fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
fclose(fileID);

% save reconstruction info to txt file
fileID = fopen([outfilePre,'_notes.txt'],'w');
fprintf(fileID,'PreThresholding = %d\n',doPreThreshold);
fprintf(fileID,'PreThreshold Value = %d\n',preThresh);
fprintf(fileID,'PostThresholding = %d\n',save_threshed);
fprintf(fileID,'PostThreshold Value = %.3f\n',thresh);
fprintf(fileID,'Convex Hull Padding = %d\n',doConvHull);
fprintf(fileID,'Reconstruction Scale = %d\n',reconstrScale);
fclose(fileID);

%% Done

beep

%%

%cursorLoc = cursor_info.Position;
%cursor_info.Position
%center_
%cursorLoc = [cursorLoc(2);cursorLoc(1);cursorLoc(3)];

%%
% figure('color','white')
% patch('vertices',v,'faces',f,'edgecolor','none',...
%     'facecolor',[1 0 0],'facelighting','phong','facealpha',0.5)
% light
% axis equal off
%%
% figure%('color',[1 1 1])
% scatter3(XYZd_thresh(:,1),XYZd_thresh(:,2),XYZd_thresh(:,3),0.5,Cd_thresh/max(max(max(Cd_thresh))),'o','LineWidth',10)
% colormap('gray')
% % set(gcf,'color','black')
% view(-84,-74)