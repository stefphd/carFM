function output = interpTraj(traj, backend, method)
%INTERPTRAJ Interpolate the trajectory using CASADI interpolant.
%
% INPUT:
% traj: trajectory data struct
% backend: backend to use ('matlab', 'casadi', 'internal')
% method: 'linear' or 'bspline'
%
% OUTPUT:
% output: struct containining CASADI interpolant functions

% Fields
traj_fields = fieldnames(traj);
optional_fields = {'z', 'sigma', 'beta', 'Gammax', 'Gammay'};
required_fields = {'zeta', 'x', 'y', 'psi', 'Gammaz'};

% Reduce mesh to avoid mem overhead
dsmin = 1; % 1m minimum resampling - HARDCODED
ds = mean(diff(traj.zeta)); % averaged sampling
if ds < dsmin
    n_ds = round(dsmin/ds);
    if n_ds <= 1
        n_ds = 2;
    end
    % resample required_fields and optional_fields fields
    keep_last = rem(numel(traj.zeta)-1, n_ds)~=0;
    for k = 1 : numel(traj_fields)
        field = traj_fields{k};
        % resample
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
    output.(field) = interpField(traj, backend, 'zeta', field, method);
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
    output.(field) = interpField(traj, backend, 'zeta', field, method);
end

% Interp Gammazp field
field = 'Gammazp';
output.isGammazp = false;
if any(strcmp(field, traj_fields))
    output.isGammazp = true;
    output.(field) = interpField(traj, backend, 'zeta', field, 'linear');
end

end

function output = interpField(data, backend, xfield, yfield, method)   
    % get data
    xdata = data.(xfield);
    ydata = data.(yfield);
    % add two more point at the beggining and end to avoid possible nan
    % evaluations at the near of domain limits
    % data added are just the first and end point repeated
    xdata = [xdata(1)-(xdata(2)-xdata(1));
             xdata(:);
             xdata(end)+(xdata(end)-xdata(end-1))
             ];
    ydata = [ydata(1);
             ydata(:);
             ydata(end)];
    % create interpolant
    switch backend
        case 'casadi' % using casadi.interpolant
            if strcmp(method, 'spline')
                method = 'bspline';
            end
            if all(ydata==0)
                method = 'linear'; % workaround for NaN in the case of all zero if 'bspline'
            end
            output = casadi.interpolant(yfield, method, {xdata}, ydata);
        case 'internal' % using carfm.common.interpolant
            if strcmp(method, 'bspline')
                method = 'spline';
            end
            output = carfm.common.interpolant(yfield, {xdata}, ydata, method);
        case 'matlab' % using matlab griddedInterpolant
            if strcmp(method, 'bspline')
                method = 'spline';
            end
            output = griddedInterpolant({xdata}, ydata, method);
        otherwise
            error('carfm:invalidType','backend must be either ''matlab'', ''casadi'', or ''internal''.');
    end
end