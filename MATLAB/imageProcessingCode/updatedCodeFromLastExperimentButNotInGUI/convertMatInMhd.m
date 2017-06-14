%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                               %
%      Function to convert a matlab             %
%      matrix in medical data mhd               %
%           Author: PF Villard                  %
%                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pig='dset023';
% load '../data/pig19/zp2/m3.mat';
m3 = CdZeroedThreshed;
%m3 = stitched;

res = size(m3)*stitch.pixSize;

mhdFile=sprintf('%s%s.mhd',dataLoc,pig);
rawFile=sprintf('%s%s.raw',dataLoc,pig);

%% Write the raw file
fout=fopen(rawFile, 'w');
%fwrite(fout,m2,'int16');
fwrite(fout,m3,'int16');
fclose(fout);

%% Write the header file
fout=fopen(mhdFile, 'wb');
fprintf(fout,' ObjectType = Image\n');
fprintf(fout,' NDims = 3\n');
fprintf(fout,' BinaryData = True\n');
fprintf(fout,' BinaryDataByteOrderMSB = False\n');
fprintf(fout,' CompressedData = False\n');
fprintf(fout,' TransformMatrix = 1 0 0 0 1 0 0 0 1\n');
fprintf(fout,' Offset = 0 0 0\n');
fprintf(fout,' CenterOfRotation = 0 0 0\n');
fprintf(fout,' AnatomicalOrientation = RPI\n');
fprintf(fout,' ElementSpacing = %f %f %f\n',res(1), res(2),res(3) );
%fprintf(fout,' DimSize = %i %i %i\n', size(m2,1), size(m2,2),size(m2,3) );
fprintf(fout,' DimSize = %i %i %i\n', size(m3,1), size(m3,2),size(m3,3) );
fprintf(fout,' ElementType = MET_SHORT\n');
fprintf(fout,' ElementDataFile = %s.raw\n',pig);
fclose(fout);

%% Write the threshold file
mhdFile2=sprintf('%s%s_2.mhd',dataLoc,pig);
rawFile2=sprintf('%s%s_2.raw',dataLoc,pig);

%% Write the raw file
fout=fopen(rawFile2, 'w');
%val = isovalue(double(m2));
%seg = m2 > val;
val = isovalue(double(m3))-10;
seg = m3 > val;
fwrite(fout,seg,'int16');
fclose(fout);


%% Write the header file
fout=fopen(mhdFile2, 'wb');
fprintf(fout,' ObjectType = Image\n');
fprintf(fout,' NDims = 3\n');
fprintf(fout,' BinaryData = True\n');
fprintf(fout,' BinaryDataByteOrderMSB = False\n');
fprintf(fout,' CompressedData = False\n');
fprintf(fout,' TransformMatrix = 1 0 0 0 1 0 0 0 1\n');
fprintf(fout,' Offset = 0 0 0\n');
fprintf(fout,' CenterOfRotation = 0 0 0\n');
fprintf(fout,' AnatomicalOrientation = RPI\n');
fprintf(fout,' ElementSpacing = %f %f %f\n',res(1), res(2),res(3) );
%fprintf(fout,' DimSize = %i %i %i\n', size(m2,1), size(m2,2),size(m2,3) );
fprintf(fout,' DimSize = %i %i %i\n', size(m3,1), size(m3,2),size(m3,3) );
fprintf(fout,' ElementType = MET_SHORT\n');
fprintf(fout,' ElementDataFile = %s.raw\n',pig);
fclose(fout);

