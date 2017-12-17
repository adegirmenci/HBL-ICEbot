files = dir('D:\BIDMC_Exp5\20170616_124815718\images') ;
files = files(3:end);

vw = VideoWriter('20170616_124815718.avi','Grayscale AVI');

vw.FrameRate = 30;

open(vw)

msg = '0';
fprintf(msg);
for i = 1:numel(files)
    writeVideo(vw,imread([files(i).folder,filesep,files(i).name]));
    
    if(mod(i,10) == 1)
        fprintf(repmat('\b', 1, numel(msg)));
        msg = sprintf('%d',i);
        fprintf(msg);
    end
end
fprintf(repmat('\b', 1, numel(msg)));
fprintf('%d frames written.\n', i);

close(vw)