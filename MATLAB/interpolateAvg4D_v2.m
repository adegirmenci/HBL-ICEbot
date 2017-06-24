function [volume, interpCube, in3] = interpolateAvg4D_v2(observations,step,interpCube,method)
% INTERPOLATEAVG_V3
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last updated : 7/14/2016
%      Given a set of points in 3D, snaps them to a grid.
%
%      observations : pixel locations and intensities
%      step : spacing between interpolated pixels (assume isometric)
%      method: avg (0) or max (1) intensity value
%      volume : interpolated volume
% See also: ICEBOT_IMGPROCGUI_V1

xyz = observations(:,1:3);
c = observations(:,4);

if(~isempty(interpCube))
    Xd = interpCube.Xd;
    Yd = interpCube.Yd;
    Zd = interpCube.Zd;

    x_disc = interpCube.x_disc;
    y_disc = interpCube.y_disc;
    z_disc = interpCube.z_disc;
    
    xyzmin = interpCube.xyzmin;
    xyzmax = interpCube.xyzmax;
    nBinsXYZ = interpCube.nBinsXYZ;
else
    % bounds
    xyzmin = floor(min(xyz)); % in mm
    xyzmax = ceil(max(xyz)); % in mm
    
    xyzdiff = xyzmax - xyzmin;
    xyzdiffpctg = xyzdiff*0.2;
    xyzmin = floor(xyzmin - xyzdiffpctg);
    xyzmax = ceil(xyzmax + xyzdiffpctg);

    % determine grid size
    nBinsXYZ = ceil((xyzmax - xyzmin)/step);
    
    % bins
    x_disc = linspace(xyzmin(1),xyzmax(1),nBinsXYZ(1));
    y_disc = linspace(xyzmin(2),xyzmax(2),nBinsXYZ(2));
    z_disc = linspace(xyzmin(3),xyzmax(3),nBinsXYZ(3));
    
    %[Xd,Yd,Zd] = meshgrid(x_disc,y_disc,z_disc);
    [Xd,Yd,Zd] = ndgrid(x_disc,y_disc,z_disc);
    
    interpCube.Xd = Xd;
    interpCube.Yd = Yd;
    interpCube.Zd = Zd;

    interpCube.x_disc = x_disc;
    interpCube.y_disc = y_disc;
    interpCube.z_disc = z_disc;
    
    interpCube.xyzmin = xyzmin;
    interpCube.xyzmax = xyzmax;
    interpCube.nBinsXYZ = nBinsXYZ;
end

nPoints = size(c,1); % number of observations

% discretize using linear interp
% this gives us the index of the point in the discretized grid
x_d = round(interp1q([xyzmin(1),xyzmax(1)]',[1,nBinsXYZ(1)]',xyz(:,1)));
y_d = round(interp1q([xyzmin(2),xyzmax(2)]',[1,nBinsXYZ(2)]',xyz(:,2)));
z_d = round(interp1q([xyzmin(3),xyzmax(3)]',[1,nBinsXYZ(3)]',xyz(:,3)));

% create containers
intensitySumCube = zeros(nBinsXYZ);
pointCountsCube = zeros(nBinsXYZ);

%
if(method == 0)
    fprintf('Using average intensity method\n');
    for i = 1:nPoints
        intensitySumCube(x_d(i),y_d(i),z_d(i)) = intensitySumCube(x_d(i),y_d(i),z_d(i)) + c(i);
        pointCountsCube(x_d(i),y_d(i),z_d(i)) = pointCountsCube(x_d(i),y_d(i),z_d(i)) + 1;
    end
elseif(method == 1)
    fprintf('Using maximum intensity method\n');
    for i = 1:nPoints
        cnt = pointCountsCube(x_d(i),y_d(i),z_d(i));
        if( cnt )  % greater than zero
            intensitySumCube(x_d(i),y_d(i),z_d(i)) = max( [intensitySumCube(x_d(i),y_d(i),z_d(i)), c(i)] );
        else % is zero
            intensitySumCube(x_d(i),y_d(i),z_d(i)) = c(i);
            pointCountsCube(x_d(i),y_d(i),z_d(i)) = 1;
        end
    end
else
    error('Method not recognized.\n');
end

% compute average
avgIntensityCube = intensitySumCube./pointCountsCube;

% take the non-inf non-NaN indices and do convex hull
goodIdx = isfinite(avgIntensityCube);
ptsX = Xd(goodIdx);
ptsY = Yd(goodIdx);
ptsZ = Zd(goodIdx);
ptsxyz = [ptsX(:),ptsY(:),ptsZ(:)];

%% Find convex hull and pad with zeros
XYZd = [Xd(:),Yd(:),Zd(:)]; % test points

%convHullIdx = convhulln(ptsxyz,'simplify', true);

% find blob
%in = inhull(XYZd,ptsxyz,convHullIdx);
fprintf('Computing inhull...');
in = inhull(XYZd,ptsxyz);
% in3 = reshape(in,size(Xd,2),size(Xd,1),size(Xd,3));
in3 = reshape(in,size(Xd,1),size(Xd,2),size(Xd,3));
fprintf('done.\n');
% might need to dilate by one
% W = 2;
% dilatedEdges = imdilate(in3,strel('square', W) );
fprintf('Eroding...');
W = 2;
in3 = imerode(in3,strel('cube',W));
fprintf('done.\n');

volume = avgIntensityCube;
volume(~in3) = 0;

if(ismac) % not engouh GPU RAM
    fprintf('THIS IS A MAC: NOT ENOUGH GPU RAM, or NO CUDA: USE SLOW INTERP\n');
    CdZeroed = griddata(xyz(:,1),xyz(:,2),xyz(:,3),c,Xd(:),Yd(:),Zd(:));
    CdZeroed = reshape(CdZeroed,size(Xd,1),size(Xd,2),size(Xd,3));
    
    volume = CdZeroed;
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% test1
% fprintf('Doing regular interp\n');
% CdZeroed = griddata(xyz(:,1),xyz(:,2),xyz(:,3),c,Xd(:),Yd(:),Zd(:));
% CdZeroed = reshape(CdZeroed,size(Xd,1),size(Xd,2),size(Xd,3));
% saveStitched2RawFile(CdZeroed, 'test1.raw')
% [xn,yn,zn] = size(CdZeroed);
% 
% fileID = fopen('test1.txt','w');
% fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
% fclose(fileID);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end