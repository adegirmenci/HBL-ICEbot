% Calibrate EM tracker at tip of bending section
% EM was mounted while the catheter was pointing straight ahead at 1.000,
% with other angles rotated as close to 1 as possible.

% Desired BB_BT transform is ideal
T_BB_BT=[1 0 0 0;
         0 1 0 0; 
         0 0 1 0.05; 
         0 0 0 1];


% The catheter was pointed straight ahead with both sensors mounted on.
% The torsion of the catheter was relaxed
%%%%% Copy the rotation 3x3 of the TIP sensor here
T_Box_STm=[-0.9958   -0.0397   -0.0814   0;
           -0.0906   0.4122   0.9065   0;
           -0.0023   0.9102   -0.4142   0;
    0 0 0 1];

% ideal ST coordinate axis
T_Box_STi=[-1   0   0  0;
            0   0   1  0;
            0   1   0  0; 
            0   0   0  1];
% transform between the ideal axis and the measured axis
T_STm_STi=inv(T_Box_STm)*T_Box_STi;
% T_STm_STi=T_Box_STm\T_Box_STi;

% Sensor at tip in terms of bending tip coords
%0.00265 = cath radius + em sensor radius
%0.0055 = em sensor tip to BT
T_BT_STi=[0   0   1   0.000;
          0  -1   0   0.0023; 
          1   0   0   0.0110;
          0 0 0 1];

T_STm_BT = T_STm_STi*inv(T_BT_STi);
% T_STm_BT = T_STm_STi/T_BT_STi;

% The catheter was pointed straight ahead with both sensors mounted on.
% The torsion of the catheter was relaxed
%%%%% Copy the rotation 3x3 of the BASE sensor here
T_Box_SBm=[-0.9978   0.0652   -0.0062 0;
           -0.0632   -0.9823   -0.1768 0;
           -0.0177   -0.1759   0.9841  0;
           0 0 0 1];

% ideal SB coordinate axis
T_Box_SBi=[-1   0   0  0;
            0  -1   0  0;
            0   0   1  0; 0 0 0 1];
% transform between the ideal axis and the measured axis
T_SBm_SBi=inv(T_Box_SBm)*T_Box_SBi;
% T_SBm_SBi=T_Box_SBm\T_Box_SBi;

T_BB_SBi=[0  -1   0   0.000;
          0   0  -1   0.0023; 
          1   0   0  -0.011; 0 0 0 1];

T_BB_SBm=T_BB_SBi*inv(T_SBm_SBi);
% T_BB_SBm=T_BB_SBi/T_SBm_SBi;

% Calibrate the angle at which the ultrasound plane broadcasts with respect
% to the BT and BB x-axis
% T_BT_BB should be 0 0 0.05 at resting
% T_CT_BB should be 0 0 0.0717 at resting
USangle=deg2rad(0);
T_BT_CT=[cos(USangle) -sin(USangle) 0 0;
      sin(USangle) cos(USangle) 0 0;
      0 0 1 0.025; % !!! IF YOU CHANGE THIS, ALSO UPDATE THE QT GUI !!!
      0 0 0 1]; %to the center of the imager
% T_BT_CT=[cos(USangle) -sin(USangle) 0 0; sin(USangle) cos(USangle) 0 0; 0 0 1 0.03; 0 0 0 1]; %to the tip of the imager


%%
% Write out constants.  Put these in the main directory for the robot code.

dlmwrite('C_T_STm_BT.txt', T_STm_BT, 'delimiter', '\n', 'precision', 4);
dlmwrite('C_T_BB_SBm.txt', T_BB_SBm, 'delimiter', '\n', 'precision', 4);
dlmwrite('C_T_BT_CT.txt', T_BT_CT, 'delimiter', '\n', 'precision', 4);

return
%% Instrument sensor on ablation catheter


% % 5Fr catheter with EM sensor
% T_Box_ISm=[-0.999 +0.044 -0.015 0;
%            +0.039 +0.971 +0.235 0;
%            +0.025 +0.235 -0.972 0;
%     0 0 0 1];% 
% 
% % 5Fr catheter with EM sensor
% % ideal SB coordinate axis
% T_Box_ISi=[-1  0   0  0;
%             0  1   0  0;
%             0  0  -1  0; 0 0 0 1];
% 
% % 5Fr catheter with EM sensor
% T_INSTR_ISi=[0  1  0   0.000;
%              0  0  1   0.0015833;
%              1  0  0  -0.017; 0 0 0 1];
% % /5FR
         
         
% % 6Fr catheter with EM sensor
% T_Box_ISm=[-0.999 -0.033 -0.037 0;
%            +0.019 +0.425 -0.905 0;
%            +0.045 -0.905 -0.424 0;
%     0 0 0 1];% 
% 
% % 6Fr catheter with EM sensor
% % ideal SB coordinate axis
% T_Box_ISi=[-1   0   0  0;
%             0   0  -1  0;
%             0  -1   0  0; 0 0 0 1];
% 
% % 6Fr catheter with EM sensor
% T_INSTR_ISi=[0  0 -1   0.000;
%              0  1  0   0.00175;
%              1  0  0  -0.018; 0 0 0 1];
% % /6FR
         
         
% % ABLATION catheter with EM sensor
% T_Box_ISm=[-0.9971   0.0479   0.0603 0;
%            -0.0532  -0.9945   -0.0900 0;
%            0.0557   -0.0929   0.9940 0;
%     0 0 0 1];% 
% 
% % ABLATION catheter with EM sensor
% % ideal SB coordinate axis
% T_Box_ISi=[-1   0   0  0;
%             0  -1   0  0;
%             0   0   1  0; 0 0 0 1];
% 
% % Ablation catheter with EM sensor
% T_INSTR_ISi=[0  -1   0   0.000;
%              0   0  -1   0.00185;
%              1   0   0  -0.040; 0 0 0 1];
% % /ABLATION

% OLD ABLATION catheter with EM sensor
T_Box_ISm=[-0.9982   -0.0492   -0.0363 0;
           -0.0349   0.9457   -0.3229 0;
            0.0500   -0.3210   -0.9458 0;
    0 0 0 1];% 

% OLD ABLATION catheter with EM sensor
% ideal SB coordinate axis
T_Box_ISi=[-1   0   0  0;
            0   1   0  0;
            0   0  -1  0; 0 0 0 1];

% Ablation catheter with EM sensor
T_INSTR_ISi=[0   1   0   0.000;
             0   0   1   0.00185;
             1   0   0  -0.013; 0 0 0 1];
% /ABLATION



% transform between the ideal axis and the measured axis
T_ISm_ISi=inv(T_Box_ISm)*T_Box_ISi;
% T_SBm_SBi=T_Box_SBm\T_Box_SBi;
         
T_INSTR_ISm=T_INSTR_ISi*inv(T_ISm_ISi);

dlmwrite('C_T_ISm_INSTR_OldAblation.txt', inv(T_INSTR_ISm), 'delimiter', '\n', 'precision', 4);

% % This is the calculation that happens in C++
% % where T_Box_ISm is the tracker reading
% T_BB_INSTR =   T_BB_Box * T_Box_ISm * T_ISm_INSTR
% %

return

%%
% Testing the box motion around the catheter
       
% Reset BB
M_BB_Box=T_BB_SBi*inv(T_SBm_SBi)*inv(M_Box_SBm)

% Calculate BB_CT
M_BB_CT=M_BB_Box*M_Box_STm*T_STm_STi*inv(T_BT_STi)*T_BT_CT



