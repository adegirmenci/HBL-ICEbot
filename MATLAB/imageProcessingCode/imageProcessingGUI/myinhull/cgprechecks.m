function cgprechecks(x, num_cgargs)
%CGPRECHECKS  Sanity checks for the Computational Geometry commands.
% The checks are applied to DELAUNAY, VORONOI, CONVHULL


%   Copyright 1984-2007 The MathWorks, Inc.

if num_cgargs < 1
    error('MATLAB:cgprechecks:NotEnoughInputs');
end

if ~isnumeric(x)
    error('MATLAB:cgprechecks:NonNumericInput');
end    

if issparse(x)
    error('MATLAB:cgprechecks:Sparse');
end  

if ~isreal(x)
    error('MATLAB:cgprechecks:Complex');
end  
    
if any(isinf(x(:)) | isnan(x(:)))
  error('MATLAB:cgprechecks:CannotAcceptInfOrNaN');
end

if ndims(x) > 2
    error('MATLAB:cgprechecks:NonTwoDInput');
end      
    
[m,n] = size(x);

if m < n+1,
  error('MATLAB:cgprechecks:NotEnoughPts');
end

