close all; clear all; clc

% Written by Alperen Degirmenci
% 1/12/2017 - Harvard Biorobotics Lab

%% Select Study Directory

root = 'C:\Users\Alperen\Documents\QT Projects\ICEbot_QT_v1\LoggedData\';
folder = uigetdir(root);

study = folder(length(root):end);

%% Load Controller Data

[CycleNum,Time,Type,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16] = ...
  importControllerFile([folder,filesep,study,'_Controller.txt']);

numRec = length(Time);

%% Extract DXYZPSI

mask = strcmp(Type,'DXYZPSI');

relevantIdx = find(mask);
relevantTime = (Time(relevantIdx) - Time(1))./1000.0;

numRelevant = length(relevantIdx);

dxyzpsi = [x1(relevantIdx), x2(relevantIdx), x3(relevantIdx), x4(relevantIdx)];

%% Extract T_BB_CT

mask = strcmp(Type,'T_BB_CT');
if(~isempty(mask))
    resetbbIdx = find(strcmp(Type,'RESETBB'));
    
    relevantIdx = find(mask);
    relevantIdx = relevantIdx(relevantIdx>resetbbIdx);
    Time_T_BB_CT = (Time(relevantIdx) - Time(1))./1000.0;
    
    numRelevant = length(relevantIdx);
    
    T_BB_CT = zeros(4,4,length(Time_T_BB_CT));
    T_BB_CT(1,1,:) = x1(relevantIdx);
    T_BB_CT(2,1,:) = x2(relevantIdx);
    T_BB_CT(3,1,:) = x3(relevantIdx);
    T_BB_CT(4,1,:) = x4(relevantIdx);
    T_BB_CT(1,2,:) = x5(relevantIdx);
    T_BB_CT(2,2,:) = x6(relevantIdx);
    T_BB_CT(3,2,:) = x7(relevantIdx);
    T_BB_CT(4,2,:) = x8(relevantIdx);
    T_BB_CT(1,3,:) = x9(relevantIdx);
    T_BB_CT(2,3,:) = x10(relevantIdx);
    T_BB_CT(3,3,:) = x11(relevantIdx);
    T_BB_CT(4,3,:) = x12(relevantIdx);
    T_BB_CT(1,4,:) = x13(relevantIdx);
    T_BB_CT(2,4,:) = x14(relevantIdx);
    T_BB_CT(3,4,:) = x15(relevantIdx);
    T_BB_CT(4,4,:) = x16(relevantIdx);
else
    disp('No T_BB_CT in file. Must be older format.');
end

%% Import Images



%% if acquiring images once at a time, at each target

% import tip data

% import images

% align image timestamp with EM and select the pertinent readings

% reconstruct


%% else, if acquiring images continuously

% import tip data

% import images

% align image timestamps with EM, put readings into memory

% figure out when the angle between images is greater than a threshold

% reconstruct