% Calibrate EM tracker at tip of bending section
% EM was mounted while the catheter was pointing straight ahead at 1.000,
% with other angles rotated as close to 1 as possible.

% Desired BB_BT transform is ideal
T_BB_BT=[1 0 0 0; 0 1 0 0; 0 0 1 0.05; 0 0 0 1];


% The catheter was pointed straight ahead with both sensors mounted on.
% The torsion of the catheter was relaxed
%%%%% Copy the rotation 3x3 of the TIP sensor here
T_Box_STm=[-0.9929  +0.0549   -0.1051  0;
           -0.1088  -0.0671   +0.9917  0;
           +0.0474  +0.9961   +0.0725  0;
    0 0 0 1];% 2015-06-12 benchtop, no bucket

% ideal ST coordinate axis
T_Box_STi=[-1  0  0  0;
            0  0  1  0;
            0  1  0  0; 0 0 0 1];
% transform between the ideal axis and the measured axis
T_STm_STi=inv(T_Box_STm)*T_Box_STi;
% T_STm_STi=T_Box_STm\T_Box_STi;

% Sensor at tip in terms of bending tip coords
%0.00265 = cath radius + em sensor radius
%0.0055 = em sensor tip to BT(the tip of the red tape)
% T_BT_STi=[0 1 0 0; 0 0 1 0.00265; 1 0 0 0.010; 0 0 0 1];
T_BT_STi=[0  0  1   0.000;
          0 -1  0   0.0023; 
          1  0  0   0.014; 0 0 0 1];
% T_BT_ST=[0 0 1 0; 0 -1 0 0.00265; 1 0 0 0.010; 0 0 0 1];

T_STm_BT = T_STm_STi*inv(T_BT_STi);
% T_STm_BT = T_STm_STi/T_BT_STi;


% The catheter was pointed straight ahead with both sensors mounted on.
% The torsion of the catheter was relaxed
%%%%% Copy the rotation 3x3 of the BASE sensor here
T_Box_SBm=[-0.9855   -0.0992   -0.1375  0;
           -0.0587   0.9603   -0.2722  0;
           0.1591   -0.2603   -0.9524  0;
           0 0 0 1];% 2015-06-12 benchtop, no bucket

% ideal SB coordinate axis
T_Box_SBi=[-1  0   0  0;
            0  1   0  0;
            0  0  -1  0; 0 0 0 1];
% transform between the ideal axis and the measured axis
T_SBm_SBi=inv(T_Box_SBm)*T_Box_SBi;
% T_SBm_SBi=T_Box_SBm\T_Box_SBi;

T_BB_SBi=[0  1  0   0.000
          0  0  1   0.0023;
          1  0  0  -0.010; 0 0 0 1];

T_BB_SBm=T_BB_SBi*inv(T_SBm_SBi);
% T_BB_SBm=T_BB_SBi/T_SBm_SBi;

% Calibrate the angle at which the ultrasound plane broadcasts with respect
% to the BT and BB x-axis
% T_BT_BB should be 0 0 0.05 at resting
% T_CT_BB should be 0 0 0.0717 at resting
USangle=deg2rad(0);
T_BT_CT=[cos(USangle) -sin(USangle) 0 0;
      sin(USangle) cos(USangle) 0 0;
      0 0 1 0.0217;
      0 0 0 1]; %to the center of the imager
% T_BT_CT=[cos(USangle) -sin(USangle) 0 0; sin(USangle) cos(USangle) 0 0; 0 0 1 0.03; 0 0 0 1]; %to the tip of the imager


%%
% Write out constants.  Put these in the main directory for the robot code.

dlmwrite('C_T_STm_BT.txt', T_STm_BT, 'delimiter', '\n', 'precision', 4);
dlmwrite('C_T_BB_SBm.txt', T_BB_SBm, 'delimiter', '\n', 'precision', 4);
dlmwrite('C_T_BT_CT.txt', T_BT_CT, 'delimiter', '\n', 'precision', 4);

%% Instrument sensor on ablation catheter

% % ablation catheter with EM sensor
% T_Box_ISm=[-0.998 -0.066 -0.015  0;
%            -0.057 +0.944 -0.325  0;
%            +0.036 -0.323 -0.946  0;
%     0 0 0 1];% 
% 
% % ablation catheter with EM sensor
% % ideal SB coordinate axis
% T_Box_ISi=[-1  0  0  0;
%             0  1  0  0;
%             0  0  -1  0; 0 0 0 1];
% 
% % ablation catheter with EM sensor
% T_INSTR_ISi=[0  1  0   0.000
%              0  0  1   0.00308;
%              1  0  0  -0.014; 0 0 0 1];

         
% 6Fr catheter with EM sensor
T_Box_ISm=[-0.997 +0.030 +0.071  0;
           -0.027 -0.999 +0.043  0;
           +0.072 +0.041 +0.996  0;
    0 0 0 1];% 

% 6Fr catheter with EM sensor
% ideal SB coordinate axis
T_Box_ISi=[-1  0   0  0;
            0  -1  0  0;
            0  0   1  0; 0 0 0 1];

% 6Fr catheter with EM sensor
T_INSTR_ISi=[0  -1  0   0.000
             0  0  -1   0.00178;
             1  0  0  -0.013; 0 0 0 1];


% transform between the ideal axis and the measured axis
T_ISm_ISi=inv(T_Box_ISm)*T_Box_ISi;
% T_SBm_SBi=T_Box_SBm\T_Box_SBi;
         
T_INSTR_ISm=T_INSTR_ISi*inv(T_ISm_ISi);

dlmwrite('C_T_ISm_INSTR.txt', inv(T_INSTR_ISm), 'delimiter', '\n', 'precision', 4);

% % This is the calculation that happens in C++
% % where T_Box_ISm is the tracker reading
% T_BB_INSTR =   T_BB_Box * T_Box_ISm * T_ISm_INSTR
% %



%%
% Testing the box motion around the catheter
       
% Reset BB
M_BB_Box=T_BB_SBi*inv(T_SBm_SBi)*inv(M_Box_SBm)

% Calculate BB_CT
M_BB_CT=M_BB_Box*M_Box_STm*T_STm_STi*inv(T_BT_STi)*T_BT_CT



