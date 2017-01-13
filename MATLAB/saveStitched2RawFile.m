function saveStitched2RawFile(vol, fname)
%save a vol to a raw file for rendering
%stitched vol from the affine_transfom is normalized to [0,1],
%so need to rescale it back to [0 255] for viewing in the vol renderer

% OLD : vol1 = vol ./ max(vol(:)) * 255;

% NEW:
if(max(vol(:)) > 255)
    fprintf('Volume has value larger than 255 (%g). Clamping.\n', max(vol(:)));
    vol(vol > 255) = 255;
end
if(min(vol(:)) < 0)
    fprintf('Volume has value smaller than 0 (%g). Clamping.\n', min(vol(:)));
    vol(vol < 0) = 0;
end

if(max(vol(:)) <= 1)
    fprintf('Volume has max value 1. Should be uint8 instead of double.\n');
end

writeRawFile(vol,fname);

end
