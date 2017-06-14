function varargout = cropMaskGenerationTool(varargin)
% CROPMASKGENERATIONTOOL MATLAB code for cropMaskGenerationTool.fig
%      CROPMASKGENERATIONTOOL, by itself, creates a new CROPMASKGENERATIONTOOL or raises the existing
%      singleton*.
%
%      H = CROPMASKGENERATIONTOOL returns the handle to a new CROPMASKGENERATIONTOOL or the handle to
%      the existing singleton*.
%
%      CROPMASKGENERATIONTOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CROPMASKGENERATIONTOOL.M with the given input arguments.
%
%      CROPMASKGENERATIONTOOL('Property','Value',...) creates a new CROPMASKGENERATIONTOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before cropMaskGenerationTool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to cropMaskGenerationTool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help cropMaskGenerationTool

% Last Modified by GUIDE v2.5 20-Jun-2016 23:04:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cropMaskGenerationTool_OpeningFcn, ...
                   'gui_OutputFcn',  @cropMaskGenerationTool_OutputFcn, ...
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


% --- Executes just before cropMaskGenerationTool is made visible.
function cropMaskGenerationTool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to cropMaskGenerationTool (see VARARGIN)

% Choose default command line output for cropMaskGenerationTool
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes cropMaskGenerationTool wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = cropMaskGenerationTool_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in selectImageButton.
function selectImageButton_Callback(hObject, eventdata, handles)
% hObject    handle to selectImageButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(ispc)
    defaultFolder = 'D:\BIDMC_Exp3';
else
    defaultFolder = '/Volumes/Macintosh HD/Research/BIDMC Exp 3';
end
delim = filesep; % get system-specific file separator (\ or /)

% ask user to select file
[handles.fileName, handles.pathName] = uigetfile({'*.jpg;*.tif;*.png;*.gif','All Image Files';...
          '*.*','All Files' },'Select Ultrasound Image for Crop Mask Generation',...
          defaultFolder);
      
if(handles.fileName) % file selected
    set(handles.defineRoiButton, 'Enable','on');
    % show image
    handles.image = imread([handles.pathName,handles.fileName]);
    if(size(handles.image,3) == 3)
        handles.image = rgb2gray(handles.image);
    end
    imshow(handles.image, 'parent', handles.axes1);
    axes(handles.axes1)
    axis equal
    
    % get height and width of image
    handles.imHeight = size(handles.image, 1);
    handles.imWidth = size(handles.image, 2);
    
else % folder not selected
    set(handles.defineRoiButton, 'Enable','off');
end
handles.cropLimitsSet = false(1,4);

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in defineRoiButton.
function defineRoiButton_Callback(hObject, eventdata, handles)
% hObject    handle to defineRoiButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.defineRoiButton,'Enable','off');
set(handles.cropDoneButton,'Enable','off');

height = handles.imHeight; width = handles.imWidth;
h = imrect(gca,[10, 10, width-20, height-20]);

api = iptgetapi(h);
% fcn = makeConstrainToRectFcn('imrect',get(gca,'XLim'),...
%    get(gca,'YLim'));
fcn = @(pos) roundConstraintFcn(pos);
api.setPositionConstraintFcn(fcn);

position = wait(h);
delete(h);
if(~isempty(position))
    handles.cropROI = round(position);
    set(handles.cropDoneButton,'Enable','on');
else
    set(handles.cropDoneButton,'Enable','off');
end
set(handles.defineRoiButton,'Enable','on');

guidata(hObject, handles);

% --- Executes on button press in cropDoneButton.
function cropDoneButton_Callback(hObject, eventdata, handles)
% hObject    handle to cropDoneButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.ellipseROIbutton,'Enable','on');

handles.croppedImage = imcrop(handles.image, handles.cropROI);

imshow(handles.croppedImage, 'parent', handles.axes1);
handles.crImHeight = size(handles.croppedImage,1);
handles.crImWidth = size(handles.croppedImage,2);

set(handles.polyROIbutton,'Enable','on');
set(handles.defineRoiButton,'Enable','off');

guidata(hObject, handles);

% --- Snaps ROI to integer values. Used in defineROI.
function pos = roundConstraintFcn(pos)
pos = round(pos);

% --- Keeps top and bottom polygon lines horizontal. 
%     Also snaps ROI to integer values. Used in polyROI.
function pos = polyConstraintFcn(pos)
% xlims = get(gca,'XLim');
% ylims = get(gca,'YLim');
% 
% tblx = [pos(1:4,1) < xlims(1), pos(1:4,1) > xlims(2)];
% tbly = [pos(1:4,2) < ylims(1), pos(1:4,2) > ylims(2)];
% 
% tblx = sum( tblx.*[ones(4,1)*xlims(1),ones(4,1)*xlims(2)], 2 );
% tbly = sum( tbly.*[ones(4,1)*ylims(1),ones(4,1)*ylims(2)], 2 );
% 
% pos(logical(tblx),1) = tblx(logical(tblx));
% pos(logical(tbly),2) = tbly(logical(tbly));

pos(1:2,2) = mean(pos(1:2,2));
pos(3:4,2) = mean(pos(3:4,2));
pos = round(pos);

function pos = ellipseConstraintFcn(pos, hObject, handles)
pos(1,:) = handles.polyROI(4,:);
pos(4,:) = handles.polyROI(3,:);
pos(2:3,2) = mean(pos(2:3,2));
midx = mean(pos([1,4],1));
distx = abs(diff(pos(2:3,1)))/2;
pos(2:3,1) = [midx - distx; midx + distx];
pos = round(pos);

% spline
splineHandle = getappdata(hObject,'splineHandle');
if(splineHandle ~= 0)
    delete(splineHandle)
    drawnow
end

hold on
xs = pos(:,1);
ys = pos(:,2);
xx = linspace(xs(1),xs(4),(abs(xs(1)-xs(4)))+1);
yy = round(spline(xs,ys,xx));
splineHandle = plot(handles.axes1, xx, yy, '-r');
setappdata(hObject,'splineHandle',splineHandle);
setappdata(hObject,'splineXY',[xx',yy']);
hold off

guidata(hObject, handles);

% --- Executes on button press in polyROIbutton.
function polyROIbutton_Callback(hObject, eventdata, handles)
% hObject    handle to polyROIbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.polyROIbutton,'Enable','off');
set(handles.ellipseROIbutton,'Enable','off');

height = handles.crImHeight; width = handles.crImWidth;
h = impoly(gca,[width/3,10; 2*width/3, 10; width-10 height-20; 10, height-20]); % CCW

api = iptgetapi(h);
% fcn = makeConstrainToRectFcn('impoly',get(gca,'XLim'),get(gca,'YLim'));
fcn = @(pos) polyConstraintFcn(pos);
api.setPositionConstraintFcn(fcn);

position = wait(h);
if(~isempty(position))
    handles.polyROI = round(position);
    handles.polyROImask = createMask(h);
    set(handles.ellipseROIbutton,'Enable','on');
else
    set(handles.ellipseROIbutton,'Enable','off');
end
delete(h);

set(handles.polyROIbutton,'Enable','on');

guidata(hObject, handles);

% --- Executes on button press in ellipseROIbutton.
function ellipseROIbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ellipseROIbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.ellipseROIbutton,'Enable','off');
set(handles.addEllipseButton,'Enable','off');

height = handles.crImHeight; width = handles.crImWidth;
h = impoly(gca,[handles.polyROI(4,:); width/3 height-10; 2*width/3 height-10; handles.polyROI(3,:)]); % CCW

setappdata(handles.ellipseROIbutton,'splineHandle',0);
setappdata(handles.ellipseROIbutton,'splineXY',[0,0]);

api = iptgetapi(h);
% fcn = makeConstrainToRectFcn('impoly',get(gca,'XLim'),get(gca,'YLim'));
fcn = @(pos) ellipseConstraintFcn(pos, hObject, handles);
api.setPositionConstraintFcn(fcn);

wait(h);
delete(h);
splineHandle = getappdata(hObject,'splineHandle');
if(splineHandle ~= 0)
    delete(splineHandle)
    drawnow
end
splineXY = getappdata(hObject,'splineXY');
if(~isempty(splineXY))
    handles.ellipseROI = round(splineXY);
    handles.ellipseROImask = poly2mask(splineXY(:,1), splineXY(:,2), height, width);
end
set(handles.ellipseROIbutton,'Enable','off');

set(handles.addEllipseButton,'Enable','on');
set(handles.doneMaskButton,'Enable','on');

handles.polyEllipseMask = handles.polyROImask | handles.ellipseROImask;

imshow(handles.croppedImage.*uint8(handles.polyEllipseMask), 'parent', handles.axes1);

guidata(hObject, handles);

% --- Executes on button press in addEllipseButton.
function addEllipseButton_Callback(hObject, eventdata, handles)
% hObject    handle to addEllipseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.addEllipseButton,'Enable','off');
set(handles.doneMaskButton,'Enable','off');

height = handles.crImHeight; width = handles.crImWidth;
h = imellipse(gca,[width/2 height/2 5 5]); % CCW
setFixedAspectRatioMode(h, true);

api = iptgetapi(h);
% fcn = makeConstrainToRectFcn('impoly',get(gca,'XLim'),get(gca,'YLim'));
fcn = @(pos) roundConstraintFcn(pos);
api.setPositionConstraintFcn(fcn);

position = wait(h);
if(~isempty(position))
    if(~exist('handles.circleROIs', 'var'))
        handles.circleROIs = {};
        handles.circleROImasks = {};
        handles.nCircleROIs = 0;
    end
    handles.nCircleROIs = handles.nCircleROIs + 1;
    handles.circleROIs{handles.nCircleROIs} = round(position);
    handles.circleROImasks{handles.nCircleROIs} = createMask(h);
    
    % update mask and image
    intersect_ = handles.polyEllipseMask & handles.circleROImasks{handles.nCircleROIs};
    handles.polyEllipseMask = handles.polyEllipseMask & ~intersect_;
    imshow(handles.croppedImage.*uint8(handles.polyEllipseMask), 'parent', handles.axes1);
end
delete(h);

set(handles.addEllipseButton,'Enable','on');
set(handles.doneMaskButton,'Enable','on');

guidata(hObject, handles);

% --- Executes on button press in doneMaskButton.
function doneMaskButton_Callback(hObject, eventdata, handles)
% hObject    handle to doneMaskButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.doneMaskButton,'Enable','off');
set(handles.addEllipseButton,'Enable','off');

handles.finalMask = handles.polyEllipseMask;
handles.finalImage = handles.croppedImage.*uint8(handles.polyEllipseMask);

set(handles.outFileNamePopup,'Enable','on');

guidata(hObject, handles);


% --- Executes on button press in saveMaskButton.
function saveMaskButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveMaskButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cropSettings.cropROI = handles.cropROI;
cropSettings.mask = handles.finalMask;
cropSettings.imHeight = handles.imHeight;
cropSettings.imWidth = handles.imWidth;

% get filename for saving
handles.outFileName = [handles.sourceType,'.mat'];
% save mask
save(handles.outFileName,'cropSettings');

guidata(hObject, handles);


% --- Executes on selection change in outFileNamePopup.
function outFileNamePopup_Callback(hObject, eventdata, handles)
% hObject    handle to outFileNamePopup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

contents = cellstr(get(hObject,'String'));
handles.sourceType = contents{get(hObject,'Value')};

set(handles.saveMaskButton,'Enable','on');

guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function outFileNamePopup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to outFileNamePopup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
