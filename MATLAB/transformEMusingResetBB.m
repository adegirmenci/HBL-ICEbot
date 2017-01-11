close all; clear all; clc

% Written by Alperen Degirmenci
% 1/6/2017 - Harvard Biorobotics Lab

% Load Controller and EM data
% Transform EM data based on ResetBB from Controller

root = 'C:\Users\Alperen\Documents\QT Projects\ICEbot_QT_v1\LoggedData\';
folder = uigetdir(root);

study = folder(length(root):end);

[CycleNum,Time,Type,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16] = ...
  importControllerFile([folder,filesep,study,'_Controller.txt']);

numRec = length(Time);

% extract DXYZPSI
mask = strcmp(Type,'DXYZPSI');

relevantIdx = find(mask);
relevantTime = (Time(relevantIdx) - Time(1))./1000.0;

numRelevant = length(relevantIdx);

dxyzpsi = [x1(relevantIdx), x2(relevantIdx), x3(relevantIdx), x4(relevantIdx)];

% extract T_BB_CT
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

%% Plot

figure
plot(relevantTime, dxyzpsi(:,1))
hold on
plot(relevantTime, dxyzpsi(:,2))
plot(relevantTime, dxyzpsi(:,3))
plot(relevantTime, dxyzpsi(:,4))

legend('x','y','z','\psi')

figure
plot3(squeeze(T_BB_CT(1,4,:)),squeeze(T_BB_CT(2,4,:)),squeeze(T_BB_CT(3,4,:)))
hold on
plot3(5,5,70,'rx')
plot3(0,0,70,'rx')
xlabel('x')
ylabel('y')
zlabel('z')
legend('Tip Trajectory','Target 1','Target 2')

return
%% Import EM

[sensorID,time,x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33,quality] = ...
    importEMfile([folder,filesep,study,'_EM.txt']);

time = (time - time(1))/1000.0; % seconds

Tforms = zeros(4,4,length(time));
Tforms(1,1,:) = r11;
Tforms(1,2,:) = r12;
Tforms(1,3,:) = r13;
Tforms(1,4,:) = x;
Tforms(2,1,:) = r21;
Tforms(2,2,:) = r22;
Tforms(2,3,:) = r23;
Tforms(2,4,:) = y;
Tforms(3,1,:) = r31;
Tforms(3,2,:) = r32;
Tforms(3,3,:) = r33;
Tforms(3,4,:) = z;
Tforms(4,1,:) = 0;
Tforms(4,2,:) = 0;
Tforms(4,3,:) = 0;
Tforms(4,4,:) = 1;

T_BB = Tforms(:,:,1:4:end);
T_BT = Tforms(:,:,2:4:end);
T_Inst = Tforms(:,:,3:4:end);
T_Chest = Tforms(:,:,4:4:end);

figure
plot(time(1:4:end), squeeze(T_BT(1,4,:)))
hold on
plot(time(1:4:end), squeeze(T_BT(2,4,:)))
plot(time(1:4:end), squeeze(T_BT(3,4,:)))
legend('x','y','z')

figure
plot3(squeeze(T_BT(1,4,:)),squeeze(T_BT(2,4,:)),squeeze(T_BT(3,4,:)))