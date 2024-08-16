%--------------------------------------------------------------------------
% Author: Crasun Jans (Janak Panthi)
%--------------------------------------------------------------------------

function [success, thetalist, errPlotMatrix, ikIter] = CallCcIkSolver(slist, qp_guess, qsb_guess, qsd, taskOffset, accuracy, maxItr, dt, eps_rw, eps_rv, Tsd)

    % Set starting guess for the primary joint angles and compute forward
    % kinematics
    oldqp= zeros(length(qp_guess),1); % for joint velocity calculation
    qp = qp_guess;
    qsb = qsb_guess;
    
    % Set closure error to zero to enter the loop
    errTwist = zeros(6,1);
    
    % Check Newton-Raphson error
    qs_tol = accuracy .* abs(qsd); % tolerance
    qs_err = abs(qsd - qsb);
    err = any(qs_err> qs_tol) || norm(errTwist(1:3)) > eps_rw || norm(errTwist(end-2:end)) > eps_rv;
    
    % Implement Algorithm
    ikIter = 1; % IK loop iterator
    warning('off'); % turn MATLAB warnings off
    
    errPlotMatrix = []; % Plotting matrix
    
    while err && ikIter<maxItr
    
        %Update Jacobians
        thetalist =[qp; qsb];
        rJ = JacobianSpace(slist,thetalist);
        Np = rJ(:,1:end-taskOffset); % primary - actuated
        Ns = rJ(:,end-taskOffset+1:end); % secondary - unactuated

        % Compute primary joint angles
        qp_dot = (qp - oldqp)/dt;
        cJ = -pinv(Ns)*(Np+errTwist*pinv(qp_dot)); % Constraint Jacobian

        oldqp = qp;

        % Pseudoinverse method
        qp = qp + pinv(cJ)*(qsd-qsb);

        % Transpose method
        % qp = qp + transpose(cJ)*(qsd-qsb);
    
        % Damped Least Squares method
        % lambda = 0.5;
         % qp = qp +cJ'*pinv(cJ*cJ'+lambda^2*eye(size(cJ*cJ',1)))*(qsd-qsb);
    
        % Optimize closure error
        [qp, qsb, errTwist] = ClosureErrorOptimizer(slist, qp, qsb, Np, Ns, Tsd, taskOffset);

    
    
        % Store errors for plotting
        errPlotMatrix(ikIter,1) = ikIter;
        errPlotMatrix(ikIter,2) = norm(qsd-qsb);
        errPlotMatrix(ikIter,3) = norm(errTwist);
    
        % Check error as loop condition
        qs_err = abs(qsd - qsb);
        err = any(qs_err> qs_tol) || norm(errTwist(1:3)) > eps_rw || norm(errTwist(end-2:end)) > eps_rv;
    
        % Increment loop iterator
        ikIter = ikIter+1;
    
    end
    % warning('on');
    
    % Determine success
    success = ~err; % If no error and thetalist does not have NaNs
    thetalist =[qp; qsb]; % store the very last values as thetalist
end
