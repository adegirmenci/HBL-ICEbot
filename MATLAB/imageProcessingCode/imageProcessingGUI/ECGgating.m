function [ECGpeakLocs,ECGpeakTime, avgHR, stdHR] = ...
    ECGgating(ECGvoltage, ECG_time, axesHandle, minPeakDist, minPeakHei)
% ECGGATING: A part of ICEbot_imgProcGUI_v1
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last updated : 6/23/2016
%      Detects peaks in ECG signal
% See also: ICEBOT_IMGPROCGUI_V1

% find peaks in ECG
%ECGvoltage = -ECGvoltage;
%ECGvoltage(ECGvoltage < 0) = 0;
% findpeaks(ECGvoltage,'MinPeakDistance',2, 'MinPeakHeight',0.25);
% [~,ECGpeakLocs] = findpeaks(ECGvoltage,'MinPeakDistance',2, 'MinPeakHeight',0.25);
axes(axesHandle);
%findpeaks(ECGvoltage,'MinPeakDistance',250, 'MinPeakHeight',0.25);
%findpeaks(ECGvoltage,ECG_time, 'MinPeakDistance',minPeakDist, 'MinPeakHeight',minPeakHei);
%[~,ECGpeakLocs] = findpeaks(ECGvoltage,'MinPeakDistance',250, 'MinPeakHeight',0.25);
[pks,ECGpeakLocs] = findpeaks(ECGvoltage,ECG_time, 'MinPeakDistance',minPeakDist, 'MinPeakHeight',minPeakHei);
plot(ECG_time,ECGvoltage,ECGpeakLocs,pks,'o')
fprintf('Found %d peaks in the ECG signal.\n',length(ECGpeakLocs))
% ECGpeakTime = ECG_time(ECGpeakLocs);% + 0.245;
ECGpeakTime = ECGpeakLocs;

% we should do more advanced processing here
% set a percentage distance from peak, so that things are scaled according
% to the cariations in heartbeat

% Average heartbeat
diffECGpeaks = diff(ECGpeakTime);
meanECGpeaks = mean(diffECGpeaks(diffECGpeaks < 1)); % anything larger means a gap in the data
stdECGpeaks = std(diffECGpeaks(diffECGpeaks < 1));
avgHR = 60./meanECGpeaks;
stdHR = 60.*stdECGpeaks/(meanECGpeaks+stdECGpeaks);
fprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);

end