close all; clear all; clc

% Load data
load('exampleInput.mat')

% Set parameters
minPeakDist = 60/120/2;
minPeakHei = 0.2;
samplingRate = 1000; % 1kHz
dataSize = samplingRate*3; % 3 seconds of data

% Run ECGgating
for i = 1:10
    time_i = time(i:i+dataSize-1);
    voltage_i = voltage(i:i+dataSize-1);
    
    [ECGpeakVals,ECGpeakTimes, avgHR, stdHR, phase] = ...
        ECGgating(voltage_i, time_i, minPeakDist, minPeakHei);
    
    fprintf('Found %d peaks in the ECG signal.\n',length(ECGpeakTimes))
    
    fprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);
end