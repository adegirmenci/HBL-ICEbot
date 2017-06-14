function [control, finputs, results] = importEMfiles(controlFileName,inputsFileName)
% IMPORTEMFILES: A part of ICEbot_imgProcGUI_v1
%      Written by Alperen Degirmenci and Paul Loschak
%      Harvard Biorobotics Lab
%      Last updated : 6/23/2016
%      Load and process the _control and _inputs files
%      Returns the parsed control, finputs, results structures
% See also: ICEBOT_IMGPROCGUI_V1

hWait = waitbar(0, 'Processing EM files');

f1 = load(controlFileName);
f2 = load(inputsFileName);

waitbar(0.1, hWait, 'Loaded files');

num_total_f1 = 41; % control
num_total_f2 = 179; % inputs function

num_f1 = size(f1,1)/num_total_f1;
num_f2 = size(f2,1)/num_total_f2;
ff1 = reshape(f1,num_total_f1,num_f1);
ff2 = reshape(f2,num_total_f2,num_f2);

% Control results
idx1 = 1;
control.fDOF_time = ff1(idx1,:); idx1 = idx1 + 1;
control.finputs_time = ff1(idx1,:); idx1 = idx1 + 1;
control.finputs_counter = ff1(idx1,:); idx1 = idx1 + 1;
control.mat1 = ff1(idx1:idx1+16-1,:); idx1 = idx1 + 16;
control.dx = ff1(idx1,:); idx1 = idx1 + 1;
control.dy = ff1(idx1,:); idx1 = idx1 + 1;
control.dz = ff1(idx1,:); idx1 = idx1 + 1;
control.dpsi = ff1(idx1,:); idx1 = idx1 + 1;
control.gamma = ff1(idx1,:); idx1 = idx1 + 1;
control.jointAngles_curr = ff1(idx1:idx1+4-1,:); idx1 = idx1 + 4; 
control.jointAngles_tgt = ff1(idx1:idx1+4-1,:); idx1 = idx1 + 4;
% control.curr_C = ff1(idx1:idx1+4-1,:); idx1 = idx1 + 4;
% control.tgt_C = ff1(idx1:idx1+4-1,:); idx1 = idx1 + 4;
% control.T_curr_calc = ff1(idx1:idx1+16-1,:); idx1 = idx1 + 16;
% control.T_tgt = ff1(idx1:idx1+16-1,:); idx1 = idx1 + 16;
control.motorcounts = ff1(idx1:idx1+4-1,:); idx1 = idx1 + 4;
control.gain_trans = ff1(idx1,:); idx1 = idx1 + 1;
control.gain_pitch = ff1(idx1,:); idx1 = idx1 + 1;
control.gain_yaw = ff1(idx1,:); idx1 = idx1 + 1;
control.gain_roll = ff1(idx1,:); idx1 = idx1 + 1;
control.flag_reached_target = ff1(idx1,:); idx1 = idx1 + 1;

waitbar(0.3, hWait, 'Calculated control');

% Inputs function
idx1 = 1;
finputs.time = ff2(idx1,:); idx1 = idx1 + 1;
finputs.fourDOF_counter = ff2(idx1,:); idx1 = idx1 + 1;

finputs.mat1 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;
finputs.mat2 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;
finputs.mat3 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;
finputs.mat4 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;

% Target pose from input boxes
finputs.inputbox_teth_x = ff2(idx1,:); idx1 = idx1 + 1;
finputs.inputbox_teth_y = ff2(idx1,:); idx1 = idx1 + 1;
finputs.inputbox_teth_z = ff2(idx1,:); idx1 = idx1 + 1;
finputs.inputbox_delt_x = ff2(idx1,:); idx1 = idx1 + 1;
finputs.inputbox_delt_y = ff2(idx1,:); idx1 = idx1 + 1;
finputs.inputbox_delt_z = ff2(idx1,:); idx1 = idx1 + 1;
finputs.inputbox_delt_psi = ff2(idx1,:); idx1 = idx1 + 1;

% Get parameter models
finputs.BBfixed_CTtraj_model = ff2(idx1:(idx1+7-1),:); idx1 = idx1 + 7;
finputs.BBfixed_CTtraj_future_model = ff2(idx1:(idx1+7-1),:); idx1 = idx1 + 7;
finputs.BBfixed_BBtraj_model = ff2(idx1:(idx1+7-1),:); idx1 = idx1 + 7;

% EKF estimates
finputs.T_BBfixed_Instr_EKF_x = ff2(idx1,:); idx1 = idx1 + 1;
finputs.T_BBfixed_Instr_EKF_y = ff2(idx1,:); idx1 = idx1 + 1;
finputs.T_BBfixed_Instr_EKF_z = ff2(idx1,:); idx1 = idx1 + 1;

% Flags
finputs.flag_coordframe = ff2(idx1,:); % World = 0, Mobile = 1
 idx1 = idx1 + 1;
finputs.flag_IT_on = ff2(idx1,:); % IT off = 0, IT on = 1
 idx1 = idx1 + 1;
finputs.flag_tethered = ff2(idx1,:); % Tethered = 0, Relative = 1
 idx1 = idx1 + 1;
finputs.flag_IT_imager = ff2(idx1,:); % Position mode = 0, Imager mode = 1
 idx1 = idx1 + 1;
finputs.flag_EKF_stop = ff2(idx1,:); % EKF on = 0, EKF off = 1
 idx1 = idx1 + 1;
 
finputs.USangle = ff2(idx1,:); idx1 = idx1 + 1;

finputs.the_future_cycle = ff2(idx1,:); idx1 = idx1 + 1;
finputs.t_begin = ff2(idx1,:); idx1 = idx1 + 1;

finputs.time2 = ff2(idx1,:); idx1 = idx1 + 1;
finputs.fourDOF_counter2 = ff2(idx1,:); idx1 = idx1 + 1;
finputs.mat5 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;
finputs.dx = ff2(idx1,:); idx1 = idx1 + 1;
finputs.dy = ff2(idx1,:); idx1 = idx1 + 1;
finputs.dz = ff2(idx1,:); idx1 = idx1 + 1;
finputs.dpsi = ff2(idx1,:); idx1 = idx1 + 1;
finputs.gamma = ff2(idx1,:); idx1 = idx1 + 1;
finputs.mat6 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;
finputs.mat7 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;
finputs.mat8 = ff2(idx1:(idx1+16-1),:); idx1 = idx1 + 16;

finputs.flag_get_inputs_from_traj3D = ff2(idx1,:); idx1 = idx1 + 1;
finputs.EM_sample_time = ff2(idx1,:); idx1 = idx1 + 1;
finputs.flag_updateorig_first_point = ff2(idx1,:); idx1 = idx1 + 1;

%

for i=1:num_f1
    control.T_BBmob_CT(:,:,i) = reshape(control.mat1(:,i),4,4)';    
end
for i=1:num_f2
    finputs.T_BBfixed_CT(:,:,i) = reshape(finputs.mat1(:,i),4,4)';    
    finputs.T_BBfixed_CTorig(:,:,i) = reshape(finputs.mat2(:,i),4,4)';    
    finputs.T_BBfixed_BBmob(:,:,i) = reshape(finputs.mat3(:,i),4,4)';    
    finputs.T_BBfixed_Instr(:,:,i) = reshape(finputs.mat4(:,i),4,4)';    
    finputs.T_BBmob_CT2(:,:,i) = reshape(finputs.mat5(:,i),4,4)';    
    finputs.T_BBfixed_CTtraj(:,:,i) = reshape(finputs.mat6(:,i),4,4)';    
    finputs.T_BBfixed_BBtraj(:,:,i) = reshape(finputs.mat7(:,i),4,4)';    
    finputs.T_BBfixed_CT_future(:,:,i) = reshape(finputs.mat8(:,i),4,4)';    

end

waitbar(0.6, hWait, 'Calculated finputs');

% Shift the target in time
edge_effect = 35;
the_future = 60;
delta_t = 0.0234;
% tgt_time_shifted = data.time + (the_future-edge_effect)*delta_t;


for i=2:num_f1
    timediff_f1(i) = control.fDOF_time(i)-control.fDOF_time(i-1);
end
for i=2:num_f2
    timediff_f2(i) = finputs.time(i)-finputs.time(i-1);
end

% from process4DOF_calc_results.m
% This depends on which flags are active
% I'm not sure if I'm doing this right... but for the simple cases it's
% easy.  For the harder cases it's confusing to know which time point is
% correct and whether I should be taking these results from the robot
% control file or the finputs file.
% What I should be doing here is looking at the results and then using the
% counter number to go into the finputs and grab the right info.  The
% counter number is simply the index of the finputs.
j = 1;
for i=1:num_f1
    idx = control.finputs_counter(i);
    results.time(i) = finputs.time(idx);
    % World coords
    if finputs.flag_coordframe(idx) == 0
        % World, Single pose
        if finputs.flag_IT_on(idx) == 0
            results.errorx(i) = finputs.dx(idx);
            results.errory(i) = finputs.dy(idx);
            results.errorz(i) = finputs.dz(idx);
            results.errorxyz(i) = sqrt((finputs.dx(idx)).^2 + (finputs.dy(idx)).^2 + (finputs.dz(idx)).^2);
            results.errorpsi(i) = finputs.dpsi(idx);
            results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
            results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
            results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
            % World, Tethered, Single pose
            if finputs.flag_tethered(idx) == 1
                results.x_tgt(i) = finputs.inputbox_teth_x(idx);
                results.y_tgt(i) = finputs.inputbox_teth_y(idx);
                results.z_tgt(i) = finputs.inputbox_teth_z(idx);
            % World, Relative adjustments, Single pose
            else
                results.x_tgt(i) = finputs.T_BBfixed_CTorig(1,4,idx) + finputs.inputbox_delt_x(idx);
                results.y_tgt(i) = finputs.T_BBfixed_CTorig(2,4,idx) + finputs.inputbox_delt_y(idx);
                results.z_tgt(i) = finputs.T_BBfixed_CTorig(3,4,idx) + finputs.inputbox_delt_z(idx);
            end
        % World, IT
        else
            % World, IT, Position
            if finputs.flag_IT_imager(idx) == 0
                % World, IT, Position, EKF off
                if finputs.flag_EKF_stop(idx) == 1
                    results.errorx(i) = finputs.dx(idx);
                    results.errory(i) = finputs.dy(idx);
                    results.errorz(i) = finputs.dz(idx);
                    results.errorxyz(i) = sqrt((finputs.dx(idx)).^2 + (finputs.dy(idx)).^2 + (finputs.dz(idx)).^2);
                    results.errorpsi(i) = finputs.dpsi(idx);
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.T_BBfixed_Instr(1,4,idx);
                    results.y_tgt(i) = finputs.T_BBfixed_Instr(2,4,idx);
                    results.z_tgt(i) = finputs.T_BBfixed_Instr(3,4,idx);
                % World, IT, Position, EKF on
                else
                    results.errorx(i) = finputs.T_BBfixed_Instr(1,4,idx) - finputs.T_BBfixed_CT(1,4,idx);
                    results.errory(i) = finputs.T_BBfixed_Instr(2,4,idx) - finputs.T_BBfixed_CT(2,4,idx);
                    results.errorz(i) = finputs.T_BBfixed_Instr(3,4,idx) - finputs.T_BBfixed_CT(3,4,idx);
                    results.errorxyz(i) = sqrt((results.errorx(i)).^2 + (results.errory(i)).^2 + (results.errorz(i)).^2);
                    results.errorpsi(i) = finputs.dpsi(idx);
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.T_BBfixed_Instr(1,4,idx);
                    results.y_tgt(i) = finputs.T_BBfixed_Instr(2,4,idx);
                    results.z_tgt(i) = finputs.T_BBfixed_Instr(3,4,idx);                    
                    
                end
            % World, IT, Imager
            else
                % World, IT, Imager, EKF off
                if finputs.flag_EKF_stop(idx) == 1
                    results.errorx(i) = finputs.dx(idx);
                    results.errory(i) = finputs.dy(idx);
                    results.errorz(i) = finputs.dz(idx);
                    results.errorxyz(i) = sqrt((finputs.dx(idx)).^2 + (finputs.dy(idx)).^2 + (finputs.dz(idx)).^2);
                    results.errorpsi(i) = finputs.dpsi(idx);
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.inputbox_teth_x(idx);
                    results.y_tgt(i) = finputs.inputbox_teth_y(idx);
                    results.z_tgt(i) = finputs.inputbox_teth_z(idx);                    
                % World, IT, Imager, EKF on
                else
                    results.errorx(i) = finputs.inputbox_teth_x(idx) - finputs.T_BBfixed_CT(1,4,idx);
                    results.errory(i) = finputs.inputbox_teth_y(idx) - finputs.T_BBfixed_CT(2,4,idx);
                    results.errorz(i) = finputs.inputbox_teth_z(idx) - finputs.T_BBfixed_CT(3,4,idx);
                    results.errorxyz(i) = sqrt((results.errorx(i)).^2 + (results.errory(i)).^2 + (results.errorz(i)).^2);
                    % This is for when the instrument is fully hooked up in
                    % all 3 axes
                    results.errorpsi(i) = CathSweep3_obj2(finputs.T_BBfixed_CT(:,:,idx),reshape(finputs.T_BBfixed_Instr(1:3,4,idx),3,1));
                    % in the meantime, use this:
%                     results.errorpsi(i) = CathSweep3_obj2(finputs.T_BBfixed_CT(:,:,idx),reshape([finputs.T_BBfixed_Instr(1,4,idx); -0.009; 0.084],3,1));
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.inputbox_teth_x(idx);
                    results.y_tgt(i) = finputs.inputbox_teth_y(idx);
                    results.z_tgt(i) = finputs.inputbox_teth_z(idx); 
                end
            end
        end
        
    % Mobile coords
    else
        % Mobile, Single pose
        if finputs.flag_IT_on(idx) == 0

            % Mobile, Tethered, Single pose
            if finputs.flag_tethered(idx) == 1
                % This case is de-activated right now.  Same as Relative.
                
            % Mobile, Relative adjustments, Single pose
            else
                results.errorx(i) = finputs.dx(idx);
                results.errory(i) = finputs.dy(idx);
                results.errorz(i) = finputs.dz(idx);
                results.errorxyz(i) = sqrt((finputs.dx(idx)).^2 + (finputs.dy(idx)).^2 + (finputs.dz(idx)).^2);
                results.errorpsi(i) = finputs.dpsi(idx);
                results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                results.x_tgt(i) = finputs.T_BBfixed_CTtraj(1,4,idx) + finputs.inputbox_delt_x(idx);
                results.y_tgt(i) = finputs.T_BBfixed_CTtraj(2,4,idx) + finputs.inputbox_delt_y(idx);
                results.z_tgt(i) = finputs.T_BBfixed_CTtraj(3,4,idx) + finputs.inputbox_delt_z(idx);

            end
        % Mobile, IT
        else
            % Mobile, IT, Position
            if finputs.flag_IT_imager(idx) == 0
                
                % Mobile, IT, Position, EKF off
                if finputs.flag_EKF_stop(idx) == 1
%                     results.time(i) = control.fDOF_time(i);
                    results.errorx(i) = finputs.dx(idx);
                    results.errory(i) = finputs.dy(idx);
                    results.errorz(i) = finputs.dz(idx);
                    results.errorxyz(i) = sqrt((finputs.dx(idx)).^2 + (finputs.dy(idx)).^2 + (finputs.dz(idx)).^2);
                    results.errorpsi(i) = finputs.dpsi(idx);
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.T_BBfixed_Instr(1,4,idx);
                    results.y_tgt(i) = finputs.T_BBfixed_Instr(2,4,idx);
                    results.z_tgt(i) = finputs.T_BBfixed_Instr(3,4,idx);                    

                % Mobile, IT, Position, EKF on
                else
                  
                    results.errorpsi(i) = finputs.dpsi(idx);
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.T_BBfixed_Instr(1,4,idx);
%                     results.y_tgt(i) = -0.009;
%                     results.z_tgt(i) = 0.084;
%                     results.y_tgt(i) = finputs.T_BBfixed_CTtraj(2,4,idx);
%                     results.z_tgt(i) = finputs.T_BBfixed_CTtraj(3,4,idx);
                    results.y_tgt(i) = finputs.T_BBfixed_Instr(2,4,idx);
                    results.z_tgt(i) = finputs.T_BBfixed_Instr(3,4,idx);
                    results.errorx(i) = results.x_tgt(i) - results.x_curr(i);
                    results.errory(i) = results.y_tgt(i) - results.y_curr(i);
                    results.errorz(i) = results.z_tgt(i) - results.z_curr(i);
                    results.errorxyz(i) = sqrt((results.errorx(i)).^2 + (results.errory(i)).^2 + (results.errorz(i)).^2);

                    
                end
            % Mobile, IT, Imager
            else
                % Mobile, IT, Imager, EKF off
                if finputs.flag_EKF_stop(idx) == 1
%                     t_now_model(i) = finputs.time(idx) - finputs.the_future_cycle(idx) - finputs.t_begin(idx);
%                     % Use the model to calculate where it should be
%                     T_BBfixed_CTtraj(1,4,i) = cycle_model_predict(t_now_model(i), tip.x_rect_x, tip.x_polar_x, m);
%                     T_BBfixed_CTtraj(2,4,i) = cycle_model_predict(t_now_model(i), tip.x_rect_y, tip.x_polar_y, m);
%                     T_BBfixed_CTtraj(3,4,i) = cycle_model_predict(t_now_model(i), tip.x_rect_z, tip.x_polar_z, m);
%                     r(1,i) = cycle_model_predict(t_now_model(i), tip.x_rect_xaxis, tip.x_polar_xaxis, m);
%                     r(2,i) = cycle_model_predict(t_now_model(i), tip.x_rect_yaxis, tip.x_polar_yaxis, m);
%                     r(3,i) = cycle_model_predict(t_now_model(i), tip.x_rect_zaxis, tip.x_polar_zaxis, m);
%                     r(4,i) = cycle_model_predict(t_now_model(i), tip.x_rect_angle, tip.x_polar_angle, m);
%                     T_BBfixed_CTtraj(1:3,1:3,i) = convert_eaa_3x3(r(:,i));
%                     T_BBfixed_CTtraj(4,1:4,i) = [0 0 0 1];
%                     
%                     results.errorx(i) = finputs.T_BBfixed_CT(1,4,idx) - T_BBfixed_CTtraj(1,4,i);
%                     results.errory(i) = finputs.T_BBfixed_CT(2,4,idx) - T_BBfixed_CTtraj(2,4,i);
%                     results.errorz(i) = finputs.T_BBfixed_CT(3,4,idx) - T_BBfixed_CTtraj(3,4,i);
%                     results.errorxyz(i) = sqrt((results.errorx(i)).^2 + (results.errory(i)).^2 + (results.errorz(i)).^2);
%                     results.errorpsi(i) = CathSweep3_obj2(finputs.T_BBfixed_CT(:,:,idx),reshape(finputs.T_BBfixed_Instr(1:3,4,idx),3,1));

                    
                    results.errorx(i) = finputs.dx(idx);
                    results.errory(i) = finputs.dy(idx);
                    results.errorz(i) = finputs.dz(idx);
                    results.errorxyz(i) = sqrt((finputs.dx(idx)).^2 + (finputs.dy(idx)).^2 + (finputs.dz(idx)).^2);
                    results.errorpsi(i) = finputs.dpsi(idx);
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.T_BBfixed_CTtraj(1,4,idx);
                    results.y_tgt(i) = finputs.T_BBfixed_CTtraj(2,4,idx);
                    results.z_tgt(i) = finputs.T_BBfixed_CTtraj(3,4,idx);                    
                
                % Mobile, IT, Imager, EKF on
                else
                    results.errorpsi(i) = CathSweep3_obj2(finputs.T_BBfixed_CT(:,:,idx),reshape(finputs.T_BBfixed_Instr(1:3,4,idx),3,1));
%                     results.errorpsi(i) = CathSweep3_obj2(finputs.T_BBfixed_CT(:,:,idx),reshape([finputs.T_BBfixed_Instr(1,4,idx),-0.009,0.084],3,1));
                    results.x_curr(i) = finputs.T_BBfixed_CT(1,4,idx);
                    results.y_curr(i) = finputs.T_BBfixed_CT(2,4,idx);
                    results.z_curr(i) = finputs.T_BBfixed_CT(3,4,idx);
                    results.x_tgt(i) = finputs.T_BBfixed_CTtraj(1,4,idx);
                    results.y_tgt(i) = finputs.T_BBfixed_CTtraj(2,4,idx);
                    results.z_tgt(i) = finputs.T_BBfixed_CTtraj(3,4,idx);                    
                    results.errorx(i) = results.x_tgt(i)-results.x_curr(i);
                    results.errory(i) = results.y_tgt(i)-results.y_curr(i);
                    results.errorz(i) = results.z_tgt(i)-results.z_curr(i);
                    results.errorxyz(i) = sqrt((results.errorx(i)).^2 + (results.errory(i)).^2 + (results.errorz(i)).^2);

                end
            end
        end        
    end
    
    % Save cropped results if running a special trajectory or sweep
    if (finputs.flag_get_inputs_from_traj3D(idx) == 1)
        results_cropped.time(j)     = results.time(i);
        results_cropped.errorx(j)   = results.errorx(i);
        results_cropped.errory(j)   = results.errory(i);
        results_cropped.errorz(j)   = results.errorz(i);
        results_cropped.errorxyz(j) = results.errorxyz(i);
        results_cropped.errorpsi(j) = results.errorpsi(i);
        results_cropped.x_curr(j)   = results.x_curr(i);
        results_cropped.y_curr(j)   = results.y_curr(i);
        results_cropped.z_curr(j)   = results.z_curr(i);
        results_cropped.x_tgt(j) = results.x_tgt(i);
        results_cropped.y_tgt(j) = results.y_tgt(i);
        results_cropped.z_tgt(j) = results.z_tgt(i);
        j=j+1;
    end
    
end

waitbar(0.9, hWait, 'Calculated results');

fprintf('\nMean error xyz: %.3f (mm), st.dev. %.3f (mm)\n',mean(results.errorxyz)*1e3,std(results.errorxyz)*1e3);

fprintf('Mean error angle: %.3f (deg), st.dev. %.3f (deg)\n', mean(abs(rad2deg(results.errorpsi))), std(abs(rad2deg(results.errorpsi))) );

fprintf('Mean delta_t, Control: %.3f ms, Finputs: %.3f ms\n\n', mean(timediff_f1)*1e3, mean(timediff_f2)*1e3);

waitbar(1, hWait, 'Done!');

close(hWait);

end