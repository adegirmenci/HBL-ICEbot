% Written by Alperen Degirmenci
% Harvard Biorobotics Lab
% Last updated : 6/25/2016
% Prep video post-trial #3
% See also: ICEBOT_IMGPROCGUI_V1

close all; clear all; clc

% where are the images
imgFolder = '/Volumes/Macintosh HD/Research/BIDMC Exp 3/2016-06-24 BIDMC exp3/RECORDED_FRMGRAB/20160624_150039';
croppedImgFolder = '/Volumes/Macintosh HD/Research/BIDMC Exp 3/2016-06-24 BIDMC exp3/RECORDED_FRMGRAB/20160624_150039cropped';
imgTimestampFile = [imgFolder,filesep,'20160624_150039_EMStart.txt'];

% where are the EM files
EMfolder = '/Volumes/Macintosh HD/Research/BIDMC Exp 3/2016-06-24 BIDMC exp3/RECORDED_DATA_ROBOT';
EMf1 = [EMfolder,filesep,'20160624_150039_4DOF_control.txt'];
EMf2 = [EMfolder,filesep,'20160624_150039_fourDOF_inputs.txt'];

% where is the ECG file
ECGfolder = '/Volumes/Macintosh HD/Research/BIDMC Exp 3/2016-06-24 BIDMC exp3/RECORDED_LABJACK/20160624_150039';
ECGfile = [ECGfolder,filesep,'20160624_150146_ECG.txt'];

% which mask are you using
load(['cropMasks',filesep,'Sequoia_StarTech_Size 1 (largest).mat'])

%%%%%

% load EM
[control_, finputs_, results_] = importEMfiles(EMf1, EMf2);

% load ECG
[ECG_time, ECGvoltage] = importECG(ECGfile);
%plot(handles.ECG_time, handles.ECGvoltage);

% load image timestamps
[imageFileNames,imageTimestamps] = importImageTimestamps(imgTimestampFile);

%% find EM that lines up with images

closestEMidx = zeros(length(imageTimestamps)-2,1);
for i = 3:length(imageTimestamps)
    t = imageTimestamps(i);
    
%     [~,idx] = min(abs(finputs_.time - t));
%     closestEMidx(i-2) = idx;
    [~,idx] = min(abs(finputs_.EM_sample_time - t));
    closestEMidx(i-2) = idx;
end

%%
warning off

T_emit = eul2tform([0,0,0]);

tr_ = finputs_.T_BBfixed_CT(:,:,1);
upvec = -tr_(1:3,1);
targetPos = tr_/[0.03 0 -0.01 1];
camPos = tr_/[0.03 -1 -0.01 1];

writeMovie = 1;
%movie 
if(writeMovie)
% vidWriter = VideoWriter('newfile.mp4','MPEG-4');
vidWriter = VideoWriter('newfile.avi','Uncompressed AVI');
open(vidWriter);
end

scrsz = get(groot,'ScreenSize');
figure('Position',[1 1 scrsz(3)/1.2 scrsz(4)/1.2])
% set(gcf,'renderer','painters');
set(gcf,'renderer','OpenGL');
for i = 1000:3000%length(closestEMidx)
    idx = closestEMidx(i);

    trplot(T_emit,'frame','Emitter','length',0.02,'rgb','thick',2, 'text_opts', {'FontSize', 12, 'FontWeight', 'bold'})
    hold on
    %set(gca,'CameraViewAngleMode', 'manual')
    set(gca,'CameraTargetMode', 'manual')
    set(gca,'CameraPositionMode', 'manual')
    set(gca,'CameraUpVector',upvec)
    set(gca,'CameraTarget',targetPos(1:3))
    set(gca, 'CameraPosition', camPos(1:3))
    set(gca, 'CameraViewAngle', 10)
    box off
    grid off
    axis off
    set(gcf,'color','k')
    
    T_CT = finputs_.T_BBfixed_CT(:,:,idx);
    trplot(T_CT,'frame','ICE','length',0.02,'rgb','thick',2, 'text_opts', {'FontSize', 14, 'FontWeight', 'bold'})
    T_Inst = finputs_.T_BBfixed_Instr(:,:,idx);
    trplot(T_Inst,'frame','Instr','length',0.025,'rgb','thick',2, 'text_opts', {'FontSize', 14, 'FontWeight', 'bold'})
    axis equal
    axis([-0.1 0.1 -0.05 0.1 -0.05 0.2])
    
    idxs = closestEMidx(1:i);
    xCT = squeeze(finputs_.T_BBfixed_CT(1,4,idxs));
    yCT = squeeze(finputs_.T_BBfixed_CT(2,4,idxs));
    zCT = squeeze(finputs_.T_BBfixed_CT(3,4,idxs));
    xIns = squeeze(finputs_.T_BBfixed_Instr(1,4,idxs));
    yIns = squeeze(finputs_.T_BBfixed_Instr(2,4,idxs));
    zIns = squeeze(finputs_.T_BBfixed_Instr(3,4,idxs));
    plot3(xCT,yCT,zCT,'m');
    plot3(xIns,yIns,zIns,'c');
    
    img = imread([croppedImgFolder,filesep,imageFileNames{i+2}(1:end-3),'jp2']);
    img = fliplr(img);
    xImage = [0 0; 1 1]*110/size(img,2)/2; % The x data for the image corners
    yImage = [0 0; 0 0]; % The y data for the image corners
    zImage = [-0.5 0.5;-0.5 0.5]*110/size(img,1)/2; % The z data for the image corners
    xyzIm = T_CT*[xImage(:)';yImage(:)';zImage(:)'; 1 1 1 1];
    xyzIm = xyzIm(1:3,:) + repmat(t2r(T_CT)*[0.003;0;0],[1,4]);
    xImage = reshape(xyzIm(1,:),[2,2]);
    yImage = reshape(xyzIm(2,:),[2,2]);
    zImage = reshape(xyzIm(3,:),[2,2]);
    hsurf = surf(xImage,yImage,zImage);
    hsurf.FaceColor = 'texturemap';
    hsurf.CData = img;
    hsurf.EdgeColor = 'none';
    hsurf.FaceAlpha = 'texturemap';
    hsurf.AlphaData = cropSettings.mask;
    colormap gray
    hold off
    drawnow
    if(writeMovie)
        F = getframe(gcf);
        writeVideo(vidWriter,F);
    end
    
end

warning on

if(writeMovie)
    close(vidWriter)
end