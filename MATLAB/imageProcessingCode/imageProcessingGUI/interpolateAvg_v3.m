function volume = interpolateAvg_v3(observations,step,method)
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

% bounds
xyzmin = floor(min(xyz)); % in mm
xyzmax = ceil(max(xyz)); % in mm

% determine grid size
nBinsXYZ = ceil((xyzmax - xyzmin)/step);

% bins
x_disc = linspace(xyzmin(1),xyzmax(1),nBinsXYZ(1));
y_disc = linspace(xyzmin(2),xyzmax(2),nBinsXYZ(2));
z_disc = linspace(xyzmin(3),xyzmax(3),nBinsXYZ(3));

[Xd,Yd,Zd] = meshgrid(x_disc,y_disc,z_disc);

nPoints = size(c,1); % number of observations
interpSize = size(Xd); % interpolated volume dimensions

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
in = inhull(XYZd,ptsxyz);
in3 = reshape(in,size(Xd,1),size(Xd,2),size(Xd,3));

% might need to dilate by one
% W = 2;
% dilatedEdges = imdilate(in3,strel('square', W) );

volume = avgIntensityCube;
volume(~in3) = 0;

end