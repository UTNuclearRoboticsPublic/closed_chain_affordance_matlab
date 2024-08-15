%--------------------------------------------------------------------------
% Function to return the screw list for the UR-5 robot, along with plotting parameters
% Author: Crasun Jans (Janak Panthi)
%--------------------------------------------------------------------------

function [mlist, slist, thetalist0, Tsd, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler, azimuth, elevation] = RobotBuilder()


        %Link lengths
        mconv = 1000;
        W1 = 109/mconv; W2 = 82/mconv; L1 = 425/mconv; L2 = 392/mconv; H1 = 89/mconv; H2 = 95/mconv; W3 = 135.85/mconv; W4 = 119.7/mconv; W6 = 93/mconv; aff = 100/mconv;

        % Start angles
        thetalist0 = zeros([10 1]);

        % Screw axis locations from base frame in home position
        q(:,1) = [0 0 H1]';% first joint, base frame
        q(:,2) = [0 W3 H1]';
        q(:,3) = [L1 W3-W4 H1]';
        q(:,4) = [L1+L2 W3-W4 H1]';
        q(:,5) = [L1+L2 W3-W4+W6 H1]';
        q(:,6) = [L1+L2 W3-W4+W6 H1-H2]';
        q(:,7) = [L1+L2 W3-W4+W6+W2 H1-H2]';% Imaginary joint
        q(:,8) = [L1+L2 W3-W4+W6+W2 H1-H2]';% Imaginary joint
        q(:,9) = [L1+L2 W3-W4+W6+W2 H1-H2]';% Imaginary joint
        q(:,10) = [L1+L2 W3-W4+W6+W2+aff H1-H2]'; % location of affordance frame
        q(:,11) = [0 0 0]';% Closed-loop end-effector


        % Type and alignment of screw axes
        w(:,1) = [0 0 1]';
        w(:,2) = [0 1 0]';
        w(:,3) = [0 1 0]';
        w(:,4) = [0 1 0]';
        w(:,5) = [0 0 -1]';
        w(:,6) = [0 1 0]';

       % Set the first frame as identity
        % mlist(:,:,1) = eye(4);
        mlist(:,:,1) = CreateTransformationMatrix('z', 0, q(:,1));

        % Construct screw axes and frames
        for i = 1:length(w)
        v(:,i) = -cross(w(:,i), q(:,i));
        slist(:,i) = [w(:,i); v(:,i)];
        mlist(:,:, i+1) = CreateTransformationMatrix('z', 0, q(:,i+1)); 
        end


        % Desired closure error frame
        Tsd = mlist(:,:,1); % same as the first joint frame

        % Task indices for plotting purposes
        x1Tindex = 10;
        x2Tindex = 9;

         % Plotting parameters
        xlimits = [-.2 1.0];
        ylimits = [-.2 1.0];
        zlimits = [-.7 .5];
        tick_quantum = 0.1;
        quiverScaler = 0.1;
        azimuth = 45;
        elevation = 45;

end
