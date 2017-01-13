%data: one 3DUS frame 
%072711 to be compatible with volumen render in c++
%no need to flip row/col
function writeRawFile(data, fout)
fid = fopen(fout,'w');
for i = 1:size(data, 3)
     fwrite(fid, squeeze(data(:,:,i)),'uchar');  
%     fwrite(fid, squeeze(data(:,:,i))','uchar');    
end
fclose(fid);
end
