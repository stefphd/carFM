function output = interpTraj(traj, backend, method, dsmin)
%INTERPTRAJ Interpolate the trajectory using CASADI interpolant.
%
% INPUT:
% traj: trajectory data struct
% backend: backend to use ('matlab', 'casadi', 'internal')
% method: 'linear' or 'bspline'
% dsmin: minimum distance for decimation
%
% OUTPUT:
% output: struct containining CASADI interpolant functions

% Fields
traj_fields = fieldnames(traj);
optional_fields = {'z', 'sigma', 'beta', 'Gammax', 'Gammay'};
required_fields = {'zeta', 'x', 'y', 'psi', 'Gammaz'};

% Reduce mesh to avoid mem overhead
% dsmin = 1; % 1m minimum resampling - now input argument
ds = mean(diff(traj.zeta)); % averaged sampling
if ds < dsmin
    n_ds = round(dsmin/ds);
    if n_ds <= 1 % should this never happends, as ds<dsmin?
        n_ds = 1;
    end
    % decimate required_fields and optional_fields fields
    keep_last = rem(numel(traj.zeta)-1, n_ds)~=0;
    for k = 1 : numel(traj_fields)
        field = traj_fields{k};
        if ~any(strcmp(field, [required_fields optional_fields, {'Gammazp'}]))
            continue;
        end
        % decimate
        last_val = traj.(field)(end);
        traj.(field) = traj.(field)(1:n_ds:end);
        % ensure last point
        if keep_last
            traj.(field)(end+1) = last_val;
        end
    end
end

% Interp required fields
for k = 1 : numel(required_fields)
    field = required_fields{k};
    output.(field) = carfm.common.interpField(traj, backend, 'zeta', field, method, 'traj');
end

% Interp optional fields
output.isTrack3D = true; % assume 3D track at the beggining
for k = 1 : numel(optional_fields)
    if ~any(strcmp(optional_fields{k}, traj_fields)) % check if optinal field exists
        % All optional field must exist for 3D track
        output.isTrack3D = false; % track is 2D b/c one or more optional field not existing
        break;
    end
    field = optional_fields{k};
    output.(field) = carfm.common.interpField(traj, backend, 'zeta', field, method, 'traj');
end

% Interp Gammazp field
field = 'Gammazp';
output.hasGammazp = false;
if any(strcmp(field, traj_fields))
    output.hasGammazp = true;
    output.(field) = carfm.common.interpField(traj, backend, 'zeta', field, 'linear', 'traj');
end

end