function varargout = ICEbot_imgProcGUI_v1(varargin)
% ICEBOT_IMGPROCGUI_V1 MATLAB code for ICEbot_imgProcGUI_v1.fig
%      ICEBOT_IMGPROCGUI_V1, by itself, creates a new ICEBOT_IMGPROCGUI_V1 or raises the existing
%      singleton*.
%
%      H = ICEBOT_IMGPROCGUI_V1 returns the handle to a new ICEBOT_IMGPROCGUI_V1 or the handle to
%      the existing singleton*.
%
%      ICEBOT_IMGPROCGUI_V1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ICEBOT_IMGPROCGUI_V1.M with the given input arguments.
%
%      ICEBOT_IMGPROCGUI_V1('Property','Value',...) creates a new ICEBOT_IMGPROCGUI_V1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ICEbot_imgProcGUI_v1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ICEbot_imgProcGUI_v1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last Update: 6/14/2017
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ICEbot_imgProcGUI_v1

% Last Modified by GUIDE v2.5 14-Jul-2016 23:39:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ICEbot_imgProcGUI_v1_OpeningFcn, ...
                   'gui_OutputFcn',  @ICEbot_imgProcGUI_v1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ICEbot_imgProcGUI_v1 is made visible.
function ICEbot_imgProcGUI_v1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ICEbot_imgProcGUI_v1 (see VARARGIN)

% Choose default command line output for ICEbot_imgProcGUI_v1
handles.output = hObject;

% add some defaults
handles.greenColor = [50,255,50]./255;
handles.redColor = [0.95, 0.2, 0.1];

% default folder locations
if(ispc)
    handles.defaultFolder = 'D:\Dropbox\Harvard\ICEbot share\Current working directory\2016-06-24 Exp3 in vivo';
else
    handles.defaultFolder = '/Volumes/Macintosh HD/Dropbox/Harvard/ICEbot share/Current working directory/2016-06-24 Exp3 in vivo';
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ICEbot_imgProcGUI_v1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ICEbot_imgProcGUI_v1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in selectStudyDirButton.
function selectStudyDirButton_Callback(hObject, eventdata, handles)
% hObject    handle to selectStudyDirButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delim = filesep; % get system-specific file separator (\ or /)

handles.studyDirPath = uigetdir(handles.defaultFolder); % ask user to select folder
if(handles.studyDirPath) % folder selected
    % get study name, it should be the name of the folder the user selected
    splitStr = regexp(handles.studyDirPath,delim,'split');
    temp = strcat(splitStr(1:end-1), {delim});
    handles.studyRootPath = [temp{:}];
    handles.studyName = splitStr{end};
    
    set(handles.studyDirTextbox, 'String', handles.studyDirPath);
    set(handles.studyDirTextbox, 'BackgroundColor', handles.greenColor);
    % activate other tools
    set(handles.maskSelectPopupMenu, 'Enable','on');
    set(handles.selectEMfileButton, 'Enable','on');
    set(handles.selectECGfileButton, 'Enable','on');
else % folder not selected
    set(handles.studyDirTextbox, 'String', 'Please select folder...');
    set(handles.studyDirTextbox, 'BackgroundColor', handles.redColor);
    % deactivate other tools
    set(handles.maskSelectPopupMenu, 'Enable','off');
    set(handles.selectEMfileButton, 'Enable','off');
    set(handles.selectECGfileButton, 'Enable','off');
    set(handles.loadEMfilesButton, 'Enable','off');
end

% Update handles structure
guidata(hObject, handles);
    

% --- Executes on selection change in maskSelectPopupMenu.
function maskSelectPopupMenu_Callback(hObject, eventdata, handles)
% hObject    handle to maskSelectPopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

contents = cellstr(get(hObject,'String'));
handles.cropMaskIdx = get(hObject,'Value');
handles.cropMask = contents{handles.cropMaskIdx};

if(exist(['cropMasks',filesep,handles.cropMask,'.mat'],'file'))
    tmp = load(['cropMasks',filesep,handles.cropMask,'.mat']);
    handles.cropSettings = tmp.cropSettings;
    handles.cropSettings.mask_uint8 = uint8(handles.cropSettings.mask);

    set(hObject,'BackgroundColor',handles.greenColor);
else
    disp(['[' 8 'Mask file does not exist!]' 8])
    set(hObject,'BackgroundColor',handles.redColor);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function maskSelectPopupMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maskSelectPopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in selectEMfileButton.
function selectEMfileButton_Callback(hObject, eventdata, handles)
% hObject    handle to selectEMfileButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ask user to select file
[handles.EMfileName,handles.EMpathName] = ...
    uigetfile({'*_4DOF_control.txt','Text';'*.*','All Files'},'Select 4DOF_control file', handles.studyRootPath);

if(handles.EMfileName) % folder selected
    % also find the fourDOF_inputs file
    lenToSub = length('4DOF_control.txt');
    handles.InputsFileName = [handles.EMfileName(1:end-lenToSub),'fourDOF_inputs.txt'];
    % if it exists
    if(exist([handles.EMpathName,handles.InputsFileName],'file'))
        fullname_ = [handles.EMpathName,handles.EMfileName];
        if(length(fullname_) > 60)
            fullname_ =  ['. . . ',fullname_(end-58:end)];
        end
        set(handles.emFileTextbox, 'String', fullname_);
        set(handles.emFileTextbox, 'BackgroundColor', handles.greenColor);
        set(handles.loadEMfilesButton, 'Enable','on');
    else % if it doesn't exist
        set(handles.emFileTextbox, 'String', 'fourDOF_inputs.txt missing...');
        set(handles.emFileTextbox, 'BackgroundColor', handles.redColor);
        set(handles.loadEMfilesButton, 'Enable','off');
    end
else % file not selected
    set(handles.emFileTextbox, 'String', 'Please select a 4DOF_control file...');
    set(handles.emFileTextbox, 'BackgroundColor', handles.redColor);
    set(handles.loadEMfilesButton, 'Enable','off');
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in loadEMfilesButton.
function loadEMfilesButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadEMfilesButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

emfile = [handles.EMpathName,handles.EMfileName];
inputsfile = [handles.EMpathName,handles.InputsFileName];
[handles.control_, handles.finputs_, handles.results_] = ...
    importEMfiles(emfile, inputsfile);

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in selectECGfileButton.
function selectECGfileButton_Callback(hObject, eventdata, handles)
% hObject    handle to selectECGfileButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ask user to select file

[handles.ECGfileName,handles.ECGpathName] = ...
    uigetfile({'*_ECG.txt','Text';'*.*','All Files'},'Select ECG file', handles.studyRootPath);

if(handles.ECGfileName) % folder selected
    fullname_ = [handles.ECGpathName,handles.ECGfileName];
    if(length(fullname_) > 60)
        fullname_ =  ['. . . ',fullname_(end-58:end)];
    end
    set(handles.ecgFileTextbox, 'String', fullname_);
    set(handles.ecgFileTextbox, 'BackgroundColor', handles.greenColor);
    set(handles.loadECGfileButton, 'Enable','on');
else % file not selected
    set(handles.ecgFileTextbox, 'String', 'Please select an ECG file...');
    set(handles.ecgFileTextbox, 'BackgroundColor', handles.redColor);
    set(handles.loadECGfileButton, 'Enable','off');
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in loadECGfileButton.
function loadECGfileButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadECGfileButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[handles.ECG_time, handles.ECGvoltage] = ...
    importECG([handles.ECGpathName,handles.ECGfileName]);

if(~isempty(handles.ECG_time))
    set(handles.ECGgatingButton, 'Enable','on');
    axes(handles.axes1);
    plot(handles.ECG_time, handles.ECGvoltage);
else
    set(handles.ECGgatingButton, 'Enable','off');
end

msg = sprintf('Loaded %d ECG readings.\n', length(handles.ECG_time));
set(handles.numECGtextbox, 'String', msg);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in ECGgatingButton.
function ECGgatingButton_Callback(hObject, eventdata, handles)
% hObject    handle to ECGgatingButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

minPeakDist = 60/100/2;
minPeakHei = 0.2;

[handles.ECGpeakLocs, handles.ECGpeakTime, avgHR, stdHR] = ...
    ECGgating(handles.ECGvoltage, handles.ECG_time, handles.axes1, minPeakDist, minPeakHei);

msg = sprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);
set(handles.bpmTextbox, 'String', msg);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in loadImgTimeButton.
function loadImgTimeButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadImgTimeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%find txt file with timestamps
handles.imageTimestampFile = dir([handles.studyDirPath,filesep, '*.txt']);
if(isempty(handles.imageTimestampFile))
    error('timeStampfiles does not exist here')
elseif(length(handles.imageTimestampFile) ~= 1)
    error('Too many timeStampfiles')
end

[imageFileNames,imageTimestamps] = ...
importImageTimestamps([handles.studyDirPath,filesep,handles.imageTimestampFile.name]);

handles.allImages = struct('FileName',imageFileNames,... % file name of image
                   'TimeStamp',num2cell(imageTimestamps),... % time stamp of image
                   'poseEM',[],... % interpolated pose
                   'EMidx',[],... % index of closest EM reading
                   'sweepIdx',[],... % sweep that the image belongs to
                   'errorPsi',[],... % angular error from desired pose
                   'ECGcycleIdx',[],... % ECG cycle that the image belongs to
                   'percentECG',[]); % location in the ECG cycle, value b/w 0.0 and 1.0

% [handles.imageFileNames,handles.imageTimestamps] = ...
% importImageTimestamps([handles.studyDirPath,filesep,handles.imageTimestampFile.name]);

set(handles.EM_ECG_button, 'Enable','on');

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in EM_ECG_button.
function EM_ECG_button_Callback(hObject, eventdata, handles)
% hObject    handle to EM_ECG_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[allImgs, reorderedImgs, accurateImgs, nSweeps] = EM_ECG_matching(handles);

handles.allImages = allImgs;
handles.reorderedImgs = reorderedImgs;
handles.accurateImgs = accurateImgs;
handles.nSweeps = nSweeps;

set(handles.stitchButton, 'Enable','on');
set(handles.reconstr4Dbutton, 'Enable','on');

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in stitchButton.
function stitchButton_Callback(hObject, eventdata, handles)
% hObject    handle to stitchButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

addpath('../updatedCodeFromLastExperimentButNotInGUI');
% addpath('../inpaintn');
% run('../RoboticsToolboxPCorke/rvctools/startup_rvc.m')

% reorderedImgs = handles.reorderedImgs;
accurateImgs = handles.accurateImgs;

nBins = 25;
binCounts = zeros(handles.nSweeps,nBins); % make 30 bins
temp = linspace(0,1,nBins+1);
binMins = temp(1:end-1);
binMaxs = temp(2:end);
% find the best aligned ECG spot
for i = 1:handles.nSweeps
    inSweepIdx = [accurateImgs(:).sweepIdx] == i; % in this sweep
    ecgs = [accurateImgs(inSweepIdx).percentECG];
    for j = 1:nBins
        isItTrue = (ecgs < binMaxs(j)) & (ecgs > binMins(j));
        binCounts(i,j) = sum(isItTrue);
    end
end

% find bin with max count
[maxBinCount,maxBinIdx] = max(sum(logical(binCounts),1));
fprintf('%d out of %d sweeps align well at bin number %d\n', maxBinCount,handles.nSweeps,maxBinIdx);

goodSweepIdx = find(binCounts(:,maxBinIdx) > 0);
nSlicesToStitch = length(goodSweepIdx);

% find the most common time by making a vector of ECG pctg 
% then do a most likelihood estimate

listOfECGs = [];
for i = 1:nSlicesToStitch
    currSweepIdx = goodSweepIdx(i);
    inSweepIdx = [accurateImgs(:).sweepIdx] == currSweepIdx; % in this sweep
    ecgs = [accurateImgs(inSweepIdx).percentECG];
    
    j = maxBinIdx;
    idxs = (ecgs < binMaxs(j)) & (ecgs >= binMins(j));
    listOfECGs = [listOfECGs, ecgs(idxs)];
end
phat = mle(listOfECGs);
ECGmean = phat(1);
fprintf(['Mean ECG phase is: %.3f ', char(177) ' %.3f\n'], ECGmean, phat(2));

% stitch
for i = 1:nSlicesToStitch
    currSweepIdx = goodSweepIdx(i);
    inSweepIdx = [accurateImgs(:).sweepIdx] == currSweepIdx; % in this sweep
    ecgs = [accurateImgs(inSweepIdx).percentECG];
    
    j = maxBinIdx;
    isItTrue = (ecgs < binMaxs(j)) & (ecgs > binMins(j));
    ecgvals = ecgs(isItTrue);
    diff_ = abs(ecgvals - ECGmean);
    [~,imIdx] = min(diff_);

    theseImgs = accurateImgs(inSweepIdx);
    theseImgs = theseImgs(isItTrue);
    fileName = theseImgs(imIdx).FileName;
    
    reportECG = theseImgs(imIdx).percentECG;
    fprintf('ECG: %.2f\n', reportECG);

    % Load image to memory
    fullName = [handles.studyDirPath,filesep,fileName];
    %fprintf('%s\n',fullName);
    stitch.imageOriginal{i} = imread(fullName);
  
    imH = size(stitch.imageOriginal{i},1);
    imW = size(stitch.imageOriginal{i},2);
    if(i == 1)
        % get frame size
        stitch.imHeightOrig = imH;
        stitch.imWidthOrig = imW;
    else
        % check image size
        if(~(stitch.imHeightOrig == imH) || ~(stitch.imWidthOrig == imW) )
            error('Images have different sizes!')
        end
    end
    % Check to make sure mask size and image size fit
    if(handles.cropSettings.imHeight ~= stitch.imHeightOrig)
        error('Image height not compatible with mask!')
    end
    if(handles.cropSettings.imWidth ~= stitch.imWidthOrig)
        error('Image width not compatible with mask!')
        % disp(['[' 8 'Image width not compatible with mask!]' 8])
    end
    
    stitch.imageCropped{i} = imcrop(stitch.imageOriginal{i}, handles.cropSettings.cropROI);

    stitch.imageCropped{i} = stitch.imageCropped{i}.*handles.cropSettings.mask_uint8; %apply mask
    
    stitch.originalEM{i} = theseImgs(imIdx).poseEM;
    
    imshow(stitch.imageCropped{i});
    drawnow
end
stitch.imHeightOrig = size(handles.cropSettings.mask,1);
stitch.imWidthOrig = size(handles.cropSettings.mask,2);

%%%%%

% initialize container 
stitch.imageLocXYZval = cell(nSlicesToStitch,1);
stitch.nFrames = nSlicesToStitch;

% interpolate
[CdZeroed] = interpolateSlices(stitch, handles.cropMask, 'true');

dt = datestr(datetime('now'),'_yymmdd_HHMMss');
outfilePre = ['volume_',handles.studyName,dt];

% save as .RAW
volumeFileName = [outfilePre,'.raw'];
saveStitched2RawFile(CdZeroed, volumeFileName)

% save as .mat
save([outfilePre,'.mat'],'CdZeroed')

% save volume size to text file
[xn,yn,zn] = size(CdZeroed);

fileID = fopen([outfilePre,'.txt'],'w');
fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
fclose(fileID);

fprintf('Done!\n')


% --- Executes on button press in reconstr4Dbutton.
function reconstr4Dbutton_Callback(hObject, eventdata, handles)
% hObject    handle to reconstr4Dbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

addpath('../updatedCodeFromLastExperimentButNotInGUI');

% reorderedImgs = handles.reorderedImgs;
accurateImgs = handles.accurateImgs;

% load mask
load(['cropMasks',filesep,handles.cropMask,'.mat']);

n4Dframes = 15;
binCounts = zeros(handles.nSweeps,n4Dframes);
temp = linspace(0,1,n4Dframes+1);
binMins = temp(1:end-1);
binMaxs = temp(2:end);
% find the best aligned ECG spot
for i = 1:handles.nSweeps
    inSweepIdx = [accurateImgs(:).sweepIdx] == i; % in this sweep
    ecgs = [accurateImgs(inSweepIdx).percentECG];
    for j = 1:n4Dframes
        isItTrue = (ecgs < binMaxs(j)) & (ecgs > binMins(j));
        binCounts(i,j) = sum(isItTrue);
    end
end

% find bin with max count
[maxBinCount,maxBinIdx] = max(sum(logical(binCounts),1));
fprintf('%d out of %d sweeps align well at bin number %d\n', maxBinCount,handles.nSweeps,maxBinIdx);

goodSweepIdx = binCounts > 0;
nSlicesToStitch4D = sum(goodSweepIdx,1);

% find the most common time by making a vector of ECG pctg 
% then do a most likelihood estimate

listOfECGs = cell(n4Dframes,1);
phat = zeros(n4Dframes,2);
for i = 1:n4Dframes
    nSlicesToStich = nSlicesToStitch4D(i);
    
    sub_goodSweepIdx = find(goodSweepIdx(:,i));
    
    for j = 1:nSlicesToStich
        currSweepIdx = sub_goodSweepIdx(j);
        inSweepIdx = [accurateImgs(:).sweepIdx] == currSweepIdx; % in this sweep
        ecgs = [accurateImgs(inSweepIdx).percentECG];
        
        idxs = (ecgs < binMaxs(i)) & (ecgs >= binMins(i));
        listOfECGs{i} = [listOfECGs{i}, ecgs(idxs)];
    end
    phat(i,:) = mle(listOfECGs{i});
    fprintf(['Frame %d: Mean ECG phase is: %.3f ', char(177) ' %.3f\n'], i, phat(i,1), phat(i,2));
end

% stitch
interpCube = [];
dt = datestr(datetime('now'),'_yymmdd_HHMMss');
for k = 1:n4Dframes
    nSlicesToStitch = nSlicesToStitch4D(k);
    sub_goodSweepIdx = find(goodSweepIdx(:,k));
    
    for i = 1:nSlicesToStitch
        currSweepIdx = sub_goodSweepIdx(i);
        inSweepIdx = [accurateImgs(:).sweepIdx] == currSweepIdx; % in this sweep
        ecgs = [accurateImgs(inSweepIdx).percentECG];
        
        isItTrue = (ecgs < binMaxs(k)) & (ecgs > binMins(k));
        ecgvals = ecgs(isItTrue);
        diff_ = abs(ecgvals - phat(k,1));
        [~,imIdx] = min(diff_);
        
        theseImgs = accurateImgs(inSweepIdx);
        theseImgs = theseImgs(isItTrue);
        fileName = theseImgs(imIdx).FileName;
        
        reportECG = theseImgs(imIdx).percentECG;
        fprintf('ECG: %.2f\n', reportECG);
        
        % Load image to memory
        fullName = [handles.studyDirPath,filesep,fileName];
        %fprintf('%s\n',fullName);
        stitch.imageOriginal{i} = imread(fullName);
        
        imH = size(stitch.imageOriginal{i},1);
        imW = size(stitch.imageOriginal{i},2);
        if(i == 1)
            % get frame size
            stitch.imHeightOrig = imH;
            stitch.imWidthOrig = imW;
        else
            % check image size
            if(~(stitch.imHeightOrig == imH) || ~(stitch.imWidthOrig == imW) )
                error('Images have different sizes!')
            end
        end
        % Check to make sure mask size and image size fit
        if(handles.cropSettings.imHeight ~= stitch.imHeightOrig)
            error('Image height not compatible with mask!')
        end
        if(handles.cropSettings.imWidth ~= stitch.imWidthOrig)
            error('Image width not compatible with mask!')
            % disp(['[' 8 'Image width not compatible with mask!]' 8])
        end
        
        stitch.imageCropped{i} = imcrop(stitch.imageOriginal{i}, handles.cropSettings.cropROI);
        
        stitch.imageCropped{i} = stitch.imageCropped{i}.*handles.cropSettings.mask_uint8; %apply mask
        
        stitch.originalEM{i} = theseImgs(imIdx).poseEM;
        
        imshow(stitch.imageCropped{i});
        drawnow
    end

stitch.imHeightOrig = size(handles.cropSettings.mask,1);
stitch.imWidthOrig = size(handles.cropSettings.mask,2);

%%%%% need respiration compensation / tissue based tracking

% initialize container 
stitch.imageLocXYZval = cell(nSlicesToStitch,1);
stitch.nFrames = nSlicesToStitch;

% interpolate
[CdZeroed,interpCube] = interpolateSlices4D(stitch, handles.cropMask, 'true', interpCube);

outfilePre = ['volume_',handles.studyName,dt,'_',num2str(k)];

% save as .RAW
volumeFileName = [outfilePre,'.raw'];
saveStitched2RawFile(CdZeroed, volumeFileName)

% save as .mat
save([outfilePre,'.mat'],'CdZeroed')

% save volume size to text file
[xn,yn,zn] = size(CdZeroed);

fileID = fopen([outfilePre,'.txt'],'w');
fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
fclose(fileID);

end

fprintf('Done!\n')
