% Written by Alperen Degirmenci
% Last updated : 9/12/15

%--------------------------------------------------------------------------

close all; clc;

%--------------------------------------------------------------------------
%ask the user for folder containing data
current_loc = mfilename('fullpath');
current_dir = fileparts(current_loc);
study_dir = [uigetdir(pwd,'Select Image Data Root Directory...'),'/'];
if isequal(study_dir,0)
    return;
end

%--------------------------------------------------------------------------
%find directories in the provided directory
folders = dir(study_dir);
folders(1:2) = [];
folders = folders([folders.isdir]);
n_folders = length(folders);

folderSinkName = [study_dir, folders(1).name];

for i = 2:n_folders
    folderSourceName = [study_dir, folders(i).name];
    
    filesList = dir(folderSourceName);
    filesList = filesList(~[filesList.isdir]);
    
    n_files = length(filesList);
    
    for j = 1:n_files
        src = [folderSourceName,'/',filesList(j).name];
        dst = [folderSinkName,'/',filesList(j).name];
        movefile(src,dst);
    end
    fprintf('.')
end

fprintf('Done\n')