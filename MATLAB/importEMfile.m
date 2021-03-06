function [sensorID,time,x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33,quality] = importEMfile(filename, startRow, endRow)
%IMPORTFILE Import numeric data from a text file as column vectors.
%   [SENSORID,TIME,X,Y,Z,R11,R12,R13,R21,R22,R23,R31,R32,R33,QUALITY] =
%   IMPORTFILE(FILENAME) Reads data from text file FILENAME for the default
%   selection.
%
%   [SENSORID,TIME,X,Y,Z,R11,R12,R13,R21,R22,R23,R31,R32,R33,QUALITY] =
%   IMPORTFILE(FILENAME, STARTROW, ENDROW) Reads data from rows STARTROW
%   through ENDROW of text file FILENAME.
%
% Example:
%   [sensorID,time,x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33,quality] = importfile('20161215_152131413_EM.txt',2, 213501);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2017/01/04 17:21:55

%% Initialize variables.
delimiter = '\t';
if nargin<=2
    startRow = 2;
    endRow = inf;
end

%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
%   column15: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow(1)-startRow(1)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(1)-1, 'ReturnOnError', false);
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines', startRow(block)-1, 'ReturnOnError', false);
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
sensorID = dataArray{:, 1};
time = dataArray{:, 2};
x = dataArray{:, 3};
y = dataArray{:, 4};
z = dataArray{:, 5};
r11 = dataArray{:, 6};
r12 = dataArray{:, 7};
r13 = dataArray{:, 8};
r21 = dataArray{:, 9};
r22 = dataArray{:, 10};
r23 = dataArray{:, 11};
r31 = dataArray{:, 12};
r32 = dataArray{:, 13};
r33 = dataArray{:, 14};
quality = dataArray{:, 15};


