function data = readRawFile(filename, dims)
fid = fopen(filename,'r');
if(numel(dims) ~= 3)
    error('dims should have three elements');
end
data = fread(fid, prod(dims), 'uchar');
data = reshape(data,dims);
fclose(fid);
end
