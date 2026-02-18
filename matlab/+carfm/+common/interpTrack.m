function output = interpTrack(track, backend, method, dsmin)
%INTERPTRACK Interpolate the track using CASADI interpolant.
%
% INPUT:
% track: track data struct
% backend: backend to use ('matlab', 'casadi', 'internal')
% method: 'linear' or 'bspline'
% dsmin: minimum distance for decimation
%
% OUTPUT:
% output: struct containining CASADI interpolant functions

% Fields
track_fields = fieldnames(track);
required_fields = {'s', 'rwl', 'rwr', 'x', 'y', 'theta', 'Omegaz', 'xl', 'yl', 'xr', 'yr'};
optional_fields = {'z', 'zl', 'zr', 'mu', 'phi', 'Omegax', 'Omegay'};

% Reduce mesh to avoid mem overhead
% dsmin = 1; % 1m minimum resampling - now input argument
ds = mean(diff(track.s)); % averaged sampling
if ds < dsmin
    n_ds = round(dsmin/ds);
    if n_ds <= 1 % should this never happends, as ds<dsmin?
        n_ds = 1;
    end
    % decimate required_fields and optional_fields fields
    keep_last = rem(numel(track.s)-1, n_ds)~=0;
    for k = 1 : numel(track_fields)
        field = track_fields{k};
        if ~any(strcmp(field, [required_fields optional_fields]))
            continue;
        end
        % decimate
        last_val = track.(field)(end);
        track.(field) = track.(field)(1:n_ds:end);
        % ensure last point
        if keep_last
            track.(field)(end+1) = last_val;
        end
    end
end

% Interp required fields
for k = 1 : numel(required_fields)
    field = required_fields{k};
    output.(field) = carfm.common.interpField(track, backend, 's', field, method, 'track');
end

% Interp optional fields
output.isTrack3D = true; % assume 3D track at the beggining
for k = 1 : numel(optional_fields)
    if ~any(strcmp(optional_fields{k}, track_fields)) % check if optinal field exists
        % All optional field must exist for 3D track
        output.isTrack3D = false; % track is 2D b/c one or more optional field not existing
        break;
    end
    field = optional_fields{k};
    output.(field) = carfm.common.interpField(track, backend, 's', field, method, 'track');
end

end