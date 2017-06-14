function volume = interpolateAvgGPU(observations,step)
% INTERPOLATEAVGGPU


xyz = observations(:,1:3);
c = observations(:,4);

% bounds
xyzmin = floor(min(xyz)); % in mm
xyzmax = ceil(max(xyz)); % in mm

% bins
x_disc = (xyzmin(1)):step:(xyzmax(1));
y_disc = (xyzmin(2)):step:(xyzmax(2));
z_disc = (xyzmin(3)):step:(xyzmax(3));

[Xd,Yd,Zd] = meshgrid(x_disc,y_disc,z_disc);

nPoints = size(c,1); % number of observations
interpSize = size(Xd); % interpolated volume dimensions

% create containers
intensitySumCube = zeros(interpSize);
pointCountsCube = zeros(interpSize);

% Go through all observations and assign each to a location in the cube
for i = 1:nPoints
    obsX = xyz(i,1);
    obsY = xyz(i,2);
    obsZ = xyz(i,3);
    obsC = c(i,1);
    
    % figure out membership
    idxX = ((obsX-step/2.0) <= x_disc) & ((obsX+step/2.0) >= x_disc);
    idxY = ((obsY-step/2.0) <= y_disc) & ((obsY+step/2.0) >= y_disc);
    idxZ = ((obsZ-step/2.0) <= z_disc) & ((obsZ+step/2.0) >= z_disc);
    
    % add intensity to sum
    intensitySumCube(idxY,idxX,idxZ) = intensitySumCube(idxY,idxX,idxZ) + obsC;
    % increment corresponding counter
    pointCountsCube(idxY,idxX,idxZ) = pointCountsCube(idxY,idxX,idxZ) + 1;
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