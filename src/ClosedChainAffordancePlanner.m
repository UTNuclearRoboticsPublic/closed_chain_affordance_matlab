%--------------------------------------------------------------------------
% Author: Crasun Jans (Janak Panthi)
% Date: 07/2023
% Description:
%   This MATLAB script simulates execution of different affordance types by
%   a UR5 robot using the Closed-chain Affordance framework.
%
% References:
% Janak Panthi, A Closed-Chain Approach to Generating Affordance Trajectories
%
%--------------------------------------------------------------------------

% Add MR functions to path
addpath(fullfile(pwd, 'ModernRobotics'));

%% Clear variables and figures
% close all
clear all
clf
clc

% Robot and Affordance Type
robotType = 'UR5';
affType = 'screw'; % values: 'pure_rot', 'pure_trans', or 'screw'.

% Algorithm control parameters
affStep = 0.1;
accuracy = 1*(1/100); % accuracy for error threshold
taskErrThreshold = accuracy*affStep;
closureErrThreshold = 1e-4;
maxItr = 50; % for IK solver
stepperMaxItr = 50; % for total steps , enter 0 to plot start config only
dt = 1e-2; % time step to compute joint velocities
delta_theta = -0.1;
pathComputerFlag = true;
taskOffset = 1;

% Build the robot screw list and frames, and set plotting parameters
[mlist, slist, thetalist0, Tsd, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation] = RobotBuilder();

M = mlist(:,:,end); % End-effector frame
start_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'; % Currently assuming robot is at home position for these simulations. TODO: start at any config. Cpp version supports this already.

% Define affordance screw
if strcmpi(affType,'pure_rot')
    % Set the components to define the affordance and affordance frame
    w_aff = -[1 0 0]';

    aff_offset = [0 -0.1 0]'; 
    q_aff = M(1:3,4) + aff_offset; % Defining wrt to the EE position for convenience but can set as however needed.

    aff_screw = [w_aff; cross(q_aff,w_aff)]
elseif strcmpi(affType,'pure_trans')
    % Set the components to define the affordance frame
    aff_offset = [0 -0.1 0]'; 
    q_aff = M(1:3,4) + aff_offset; % Defining wrt to the EE position for convenience but can set as however needed.

    % Set the affordance screw
    aff_screw = [0 0 0 -1 0 0]';
elseif strcmpi(affType, 'screw')
    % Set the components to define the affordance and affordance frame
    pitch = 0.05; %m/rad
    w_aff = [1 0 0]';

    aff_offset = [0 0.1 0]'; 
    q_aff = M(1:3,4) + aff_offset; % Defining wrt to the EE position for convenience but can set as however needed.

    % Compute the screw
    aff_screw = [w_aff; cross(q_aff,w_aff)+pitch*w_aff];
end

% cc_slist = compose_cc_slist(slist, start_config, M, aff_screw);
slist = ComposeCcSlist(slist, start_config, M, aff_screw, affType);

% Append frames for plotting purposes
mlist(:,:,end+1) = M; % Virtual screw axis 2. M for Virtual screw axis 1 is already set as EE HTM from RobotBuilder.
mlist(:,:,end+1) = M; % Virtual screw axis 3
% Define affordance frame
aff_frame = eye(4);
aff_frame(1:3,4) = q_aff;
mlist(:,:,end+1) = aff_frame;
% Closed-chain end-effector frame
mlist(:,:,end+1) = Tsd; %Tsd is desired closed-chain end-effector frame


% Plot starting configuration
figure(1)
robotType = 'UR5';
if strcmpi(robotType,'UR5')
    robot = loadrobot("universalUR5","DataFormat","column");
else
    robot = [];
end


screwPathMatrix = zeros(3,3,3);
[plotrepf, plotrepl, plotrepj, plotrept, plotrepn] = FKPlotter(mlist,slist,thetalist0, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix);

% Loop iterators
stepperItr = 1; % increment iterator for the Stepper loop
stepperItrSuc = 1; % increment iterator for the Stepper loop for successful steps
ikIterHolder = []; % Holds IK iterations for successful steps

% Guesses
qp_guess = thetalist0(1:end-taskOffset);
qsb_guess = thetalist0(end-taskOffset+1:end);

% Stepping loop
while stepperItr<=stepperMaxItr

% Set desired secondary task (affordance and maybe gripper orientation) as
% just a few radians away from the current position
qsd = thetalist0(end-taskOffset+1:end);
qsd(taskOffset) = qsd(taskOffset)+stepperItr*affStep;

% Call IK solver
[success, thetalist, errPlotMatrix, ikIter] = CallCcIkSolver(slist, qp_guess, qsb_guess, qsd, taskOffset, taskErrThreshold, maxItr, dt, closureErrThreshold, Tsd);


if success
    % disp("Working m:")
    % disp(stepperItr);
    ikIterHolder = [ikIterHolder ikIter-1];
    if pathComputerFlag
    % Compute the screw path to plot as well
    screwPathStart = FKinSpace(mlist(:,:,end-2), slist(:,1:end-2), thetalist(1:end-2));% Starting guess for all relevant frames/tasks
    if strcmpi(affType,'pure_trans')
        iterations = 5;
        start_offset = [0.1; 0; 0];
        screwPathStart(1:3, 4) = screwPathStart(1:3, 4) + start_offset; % offset the path such that the beginning of the path coincides with the start config of the robot
        screwPath  = ScrewPathCreator(slist(:,end), screwPathStart, -delta_theta, iterations);

    else
        iterations = 64;
        screwPath  = ScrewPathCreator(slist(:,end), screwPathStart, delta_theta, iterations);
    end
    screwPathMatrix = reshape(screwPath(1:3, 4, :), 3, [])'; % get xyz coordinates and put them in an N x 3 form
    pathComputerFlag = false; % to compute only once

    end
    animPlotMatrix(:,stepperItrSuc) = thetalist;

    %Update the guess for next iteration
    qp_guess = thetalist(1:end-taskOffset);
    qsb_guess = thetalist(end-taskOffset+1:end);
    stepperItrSuc = stepperItrSuc+1;
end

% Increment stepper loop iterator
stepperItr = stepperItr +1;
end

% Plot animation
figure(1)

% Delete the manipulator initial config plot references
if strcmpi(robotType, 'UR5')
    delete(plotrepn);
end

% Capture animation video
% v = VideoWriter("demo", 'MPEG-4');
% open(v);
% Animate stored configurations
for ikIter = 1:1:stepperItrSuc-1
    [plotrepf, plotrepl, plotrepj, plotrept, plotrepn] = FKPlotter(mlist,slist,animPlotMatrix(:,ikIter), x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix);
    drawnow;
    % frame = getframe(gcf);
    % writeVideo(v,frame)
    pause(0.001);
    if ikIter~=stepperItrSuc-1
        if strcmpi(robotType, 'UR5')
            delete(plotrepn);
        end
    end
end
% close(v)
