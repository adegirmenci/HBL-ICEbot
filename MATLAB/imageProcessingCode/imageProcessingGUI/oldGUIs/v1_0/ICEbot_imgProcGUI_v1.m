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
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ICEbot_imgProcGUI_v1

% Last Modified by GUIDE v2.5 29-Jun-2016 11:07:34

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
    handles.defaultFolder = 'D:\BIDMC_Exp3';
else
    handles.defaultFolder = '/Volumes/Macintosh HD/Research/BIDMC Exp 3';
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
    handles.croppedFolderName = [handles.studyName,'cropped'];
    handles.croppedFolderPath = [handles.studyRootPath,handles.croppedFolderName];
    
    set(handles.studyDirTextbox, 'String', handles.studyDirPath);
    set(handles.studyDirTextbox, 'BackgroundColor', handles.greenColor);
    % activate other tools
    set(handles.maskSelectPopupMenu, 'Enable','on');
    set(handles.alreadyCroppedToggleButton, 'Enable','on');
    set(handles.selectEMfileButton, 'Enable','on');
    set(handles.selectECGfileButton, 'Enable','on');
else % folder not selected
    set(handles.studyDirTextbox, 'String', 'Please select folder...');
    set(handles.studyDirTextbox, 'BackgroundColor', handles.redColor);
    % deactivate other tools
    set(handles.maskSelectPopupMenu, 'Enable','off');
    set(handles.cropButton, 'Enable','off');
    set(handles.alreadyCroppedToggleButton, 'Enable','off');
    set(handles.selectEMfileButton, 'Enable','off');
    set(handles.selectECGfileButton, 'Enable','off');
    set(handles.loadEMfilesButton, 'Enable','off');
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in cropButton.
function cropButton_Callback(hObject, eventdata, handles)
% hObject    handle to cropButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cropSuccess = cropImagesAndSave(handles.studyDirPath, handles.croppedFolderPath, handles.cropMask);
if(cropSuccess)
    set(handles.cropButton, 'Enable','off');
    set(handles.alreadyCroppedToggleButton, 'Enable','off');
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

set(handles.cropButton, 'Enable','on');

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


% --- Executes on button press in alreadyCroppedToggleButton.
function alreadyCroppedToggleButton_Callback(hObject, eventdata, handles)
% hObject    handle to alreadyCroppedToggleButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

toggleState = get(hObject,'Value');
if(toggleState) % already cropped
    % check if '*cropped' folder exists
    
    if exist(handles.croppedFolderPath, 'dir')
        set(handles.maskSelectPopupMenu, 'Enable','off');
        set(handles.cropButton, 'Enable','off');
    else
        warningMsg = [handles.croppedFolderName,' does not exist!'];
        warndlg(warningMsg,'Warning!') % display warning message
        set(handles.alreadyCroppedToggleButton,'Value',0); % untoggle
    end
else
    set(handles.maskSelectPopupMenu, 'Enable','on');
    set(handles.cropButton, 'Enable','on');
end

% Update handles structure
guidata(hObject, handles);


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

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in ECGgatingButton.
function ECGgatingButton_Callback(hObject, eventdata, handles)
% hObject    handle to ECGgatingButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

minPeakDist = 60/100/2;
minPeakHei = 0.2;
timeOffset = 0.445;

[handles.ECGpeakLocs, handles.ECGpeakTime] = ...
    ECGgating(handles.ECGvoltage, handles.ECG_time, handles.axes1, minPeakDist, minPeakHei);

handles.ECGpeakTime = handles.ECGpeakTime + timeOffset;

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

[handles.imageFileNames,handles.imageTimestamps] = ...
importImageTimestamps([handles.studyDirPath,filesep,handles.imageTimestampFile.name]);

set(handles.EM_ECG_button, 'Enable','on');

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in EM_ECG_button.
function EM_ECG_button_Callback(hObject, eventdata, handles)
% hObject    handle to EM_ECG_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% get EM readings
EMidx = logical(handles.finputs_.flag_updateorig_first_point);
EMreadingTime = handles.finputs_.EM_sample_time(EMidx);
EMreading = handles.finputs_.T_BBfixed_CT(:,:,EMidx);

% find images that match ECG peaks
handles.alignedImagesIdx = zeros(length(handles.ECGpeakTime),1);
handles.alginedEMidx = zeros(length(handles.ECGpeakTime),1);
handles.alginedEMreading = zeros(4,4,length(handles.ECGpeakTime));
for i = 1:length(handles.ECGpeakTime)
    ecgt_ = handles.ECGpeakTime(i);
    difft_ = abs(handles.imageTimestamps - ecgt_);
    [~,minidx] = min(difft_);
    handles.alignedImagesIdx(i) = minidx;
    
    difft_ = abs(EMreadingTime - handles.imageTimestamps(minidx));
    [~,minidx] = min(difft_);
    handles.alginedEMidx(i) = minidx;
    handles.alginedEMreading(:,:,i) = EMreading(minidx);
end

fprintf('Found %d EM readings that line up with ECG and images.', sum(handles.alginedEMidx ~= 0))
axes(handles.axes1)
hold on
plot(EMreadingTime(handles.alginedEMidx),zeros(length(handles.alginedEMidx),1)','o')
hold off

% interpolate EM accordingly
%%%

set(handles.stitchButton, 'Enable','on');

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in stitchButton.
function stitchButton_Callback(hObject, eventdata, handles)
% hObject    handle to stitchButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

addpath('../updatedCodeFromLastExperimentButNotInGUI');
addpath('../inpaintn');
addpath('../RoboticsToolboxPCorke/');

% initialize container
stitch.imageOriginal = cell(length(handles.alignedImagesIdx),1);

% load mask
load(['cropMasks',filesep,handles.cropMask,'.mat']);

j = 1;
for i = 1:length(handles.alignedImagesIdx)
    fileName = handles.imageFileNames{handles.alignedImagesIdx(i)};

    % Load image to memory
    fullName = [handles.studyDirPath,'cropped',filesep,fileName(1:end-3),'jp2'];
    disp(fullName)
    stitch.imageOriginal{j} = imread(fullName);
    
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
%%%%%

% calculate pixel size
stitch.usPlaneLength = 76.6-3.9; %76.0; % mm
stitch.pixSize = stitch.usPlaneLength/stitch.imHeightOrig; % mm/pix
% generate image to EM transformation matrix
stitch.T_CT_IMG = [0 1 0 0;...
            0 0 -1 0;...
           -1 0 0 stitch.imWidthOrig/2.0*stitch.pixSize;...
            0 0 0 1];

% initialize container 
stitch.imageLocXYZval = cell(length(handles.alignedImagesIdx),1);
Npixels = stitch.imHeightOrig * stitch.imWidthOrig; % N pixels
%[X,Y] = meshgrid(0:stitch.imHeightOrig-1,0:stitch.imWidthOrig-1); % mesh of img idx
[X,Y] = meshgrid(0:stitch.imWidthOrig-1,0:stitch.imHeightOrig-1); % mesh of img idx

stitch.nFrames = length(handles.alignedImagesIdx);
Xvec = X(:);
Yvec = Y(:);
maskVec = logical(cropSettings.mask(:));
XvecNo0 = Xvec(maskVec);
YvecNo0 = Yvec(maskVec);
NpixelsNo0 = sum(maskVec);

j = 1;
for i = 1:stitch.nFrames
    % prepare container
    stitch.imageLocXYZval{j} = zeros([NpixelsNo0,4]); % initialize N-by-4
    stitch.imageLocXYZval{j}(:,1) = XvecNo0*stitch.pixSize; % scale to mm
    stitch.imageLocXYZval{j}(:,2) = YvecNo0*stitch.pixSize; % scale to mm
    % Z is zero
    tmp = stitch.imageOriginal{j};
    % insert image
    tmp = tmp(:);
    stitch.imageLocXYZval{j}(:,4) = tmp(maskVec);
    
    % register images based on EM
    tmp = stitch.imageLocXYZval{j}';
    tmp(4,:) = 1; % put it in [x;y;z;1] format
    tmp = handles.alginedEMreading(:,:,i)*stitch.T_CT_IMG*tmp;
    tmp = tmp';
    stitch.imageLocXYZval{j}(:,1:3) = tmp(:,1:3);
    j = j + 1;
end

xyz = zeros([NpixelsNo0*stitch.nFrames,3]);
c = zeros([NpixelsNo0*stitch.nFrames,1]);

for j = 1:stitch.nFrames
    xyz((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,1:3);
    c((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,4);
end

observations = [xyz,c];
volume = interpolateAvg_v2(observations,2.0);

fprintf('Computing griddata...')
tic
Cd = inpaintn(volume,20);
toc

CdZeroed = Cd;
CdZeroed(isnan(CdZeroed)) = 0;

dt = datestr(datetime('now'),'yymmdd_HHMMss');
outfilePre = ['volume_',num2str(nStudy),dt];

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
