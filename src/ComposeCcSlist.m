%--------------------------------------------------------------------------
% Author: Crasun Jans (Janak Panthi)
%--------------------------------------------------------------------------
function [cc_slist] = ComposeCcSlist(slist, thetalist, M, aff_screw, affType)

        % Compute robot Jacobian
        jac = JacobianSpace(slist(:, 1:6), thetalist(1:6));

        % Compute forward kinematics to EE
        fk = FKinSpace(M, slist(:, 1:6), thetalist(1:6));

        % Construct virtual EE screws as aligned with the space-frame axes
        sph_x = [[1, 0, 0] cross(fk(1:3, 4)', [1, 0, 0])]';
        sph_y = [[0, 1, 0] cross(fk(1:3, 4)', [0, 1, 0])]';
        sph_z = [[0, 0, 1] cross(fk(1:3, 4)', [0, 0, 1])]';

        % Fill out the cc_list
        if strcmpi(affType, 'pure_rot')
            aff_screw = -aff_screw;
        end
        cc_slist = [jac sph_x sph_y sph_z aff_screw];

end