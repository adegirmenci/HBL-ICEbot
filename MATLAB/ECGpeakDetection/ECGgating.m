function [ECGpeakVals,ECGpeakTimes, avgHR, stdHR, phase] = ...
        ECGgating(ECGvoltage, ECG_time, minPeakDist, minPeakHei)
% ECGGATING: A part of ICEbot_imgProcGUI_v1
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last updated : 6/23/2016
%      Detects peaks in ECG signal
% See also: ICEBOT_IMGPROCGUI_V1

[ECGpeakVals,ECGpeakTimes] = findpeaks(ECGvoltage,ECG_time, ...
                                       'MinPeakDistance',minPeakDist, ...
                                         'MinPeakHeight',minPeakHei);
% fprintf('Found %d peaks in the ECG signal.\n',length(ECGpeakTime))

% Average heartbeat
diffECGpeaks = diff(ECGpeakTimes);
validDiffECGpeaks = diffECGpeaks(diffECGpeaks < 1.2);
meanECGpeaks = mean(validDiffECGpeaks); % anything larger means a gap in the data
stdECGpeaks = std(validDiffECGpeaks);
avgHR = 0; stdHR = 0; phase = 0;
if(numel(validDiffECGpeaks))
    avgHR = 60./meanECGpeaks;
    stdHR = 60.*stdECGpeaks/(meanECGpeaks+stdECGpeaks);
    
    % find the distance from the last peak
    lastPeakTime = ECGpeakTimes(end);
    currTime = ECG_time(end);
    
    % convert to phase
    phase = (currTime - lastPeakTime)/ meanECGpeaks;
end
% fprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);

end