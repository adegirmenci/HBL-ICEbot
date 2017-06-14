% Written by Alperen Degirmenci
% Last updated : 9/12/15

%--------------------------------------------------------------------------

close all; clc;

%--------------------------------------------------------------------------
%ask the user for folder containing data
current_loc = mfilename('fullpath');
current_dir = fileparts(current_loc);
study_dir = [uigetdir(pwd,'Select Timestamp Root Directory...'),'/'];
if isequal(study_dir,0)
    return;
end

%--------------------------------------------------------------------------
%find '*.txt' files in the provided directory
time_files = dir([study_dir,'*EMStart.txt']);
%time_files = dir([study_dir,'*AUTOSWEEP.txt']);
%time_files = dir([study_dir,'*BUTTON.txt']);

n_files = length(time_files);

if(isempty(time_files))
    error('Time_files does not exist here')
elseif(n_files == 1)
    fprintf('Only one file found, no need for merge. Terminating.\n')
    return
else
    fprintf('Found %d files\n',n_files)
end

filename = [study_dir, time_files(1).name];
fileSinkName = [study_dir, 'combinedTime.txt'];
% copy for safe operations
copyfile(filename, fileSinkName);

fileSink = fopen(fileSinkName,'a'); % copy to this
fseek(fileSink, 0, 'eof');

startRow = 3;

for i = 2:n_files
    filename = [study_dir, time_files(i).name];
    fileSource = fopen(filename,'r'); % copy from this
    
    for j = 1:startRow
        tline = fgets(fileSource); % skip these lines
    end
    
    while ischar(tline)
        fprintf(fileSink,'%s',tline);
        tline = fgets(fileSource);
    end
    
    fclose(fileSource);
    
    fprintf('.')
end

fclose(fileSink);

fprintf('Done\n')