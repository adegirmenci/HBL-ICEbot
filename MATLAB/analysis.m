close all; clear all; clc

folder = 'C:\Users\Alperen\Documents\QT Projects\ICEbot_QT_v1\LoggedData\20170616_103508752\20170616_103508752';

[CycleNum,Time,Type,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16] = ...
  importControllerFile([folder,'_Controller.txt']);
% importControllerFile('../20170104_144907123/20170104_144907123_Controller.txt');


numRec = length(Time);

mask = strcmp(Type,'DXYZPSI');

relevantIdx = find(mask);
relevantTime = (Time(relevantIdx) - Time(1))./1000.0;

numRelevant = length(relevantIdx);

dxyzpsi = [x1(relevantIdx), x2(relevantIdx), x3(relevantIdx), x4(relevantIdx)];

figure
plot(relevantTime, dxyzpsi(:,1))
hold on
plot(relevantTime, dxyzpsi(:,2))
plot(relevantTime, dxyzpsi(:,3))
plot(relevantTime, dxyzpsi(:,4))

legend('x','y','z','$\psi$')

%% Import EM

[sensorID,time,x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33,quality] = ...
    importEMfile([folder,'_EM.txt']);

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
axis equal

figure
plot3(squeeze(T_BT(1,4,:)),squeeze(T_BT(2,4,:)),squeeze(T_BT(3,4,:)))

%%

figure
plot(time(1:4:end), squeeze(T_BT(3,4,:)))
hold on
plot(time(1:4:end), squeeze(T_Chest(3,4,:))+90)
legend('x','y','z')
axis equal