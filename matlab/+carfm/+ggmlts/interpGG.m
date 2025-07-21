function varargout = interpGG(gg, backend, method, varargin)
%INTERPGG Interpolate the g-g map using CASADI interpolant.
%
% INPUT:
% gg: g-g map struct (possibly vector)
% backend: backend to use ('matlab' or 'casadi') 
% method: interpolation method ('linear' or 'bspline')
% varargin: field(s) to interpolate
%
% OUTPUT:
% varargout: interpolant function (cell)

% Loop over gg vector struct
varargout = cell(size(varargin));
for k0 = 1 : numel(varargin)
    varargout{k0} = cell(size(gg));
end
for l = 1 : numel(gg)

    % Fill missing for each g-g section
    for k1 = 1 : numel(gg(l).V)
        for k2 = 1 : numel(gg(l).g)
            if any(isnan(gg(l).rho(:,k1,k2))) % use rho to check for missing
                fprintf("Warning: filled missing values in g-g at V(%d)=%g, g(%d)=%g\n", ...
                    k1, gg(l).V(k1), k2, gg(l).g(k2));
                % fill each field
                for k0 = 1 : numel(varargin)
                    field = varargin{k0};
                    for k3 = 1 : size(gg(l).(field), 4)
                        gg(l).(field)(:,k1,k2,k3) = fillmissing(gg(l).(field)(:,k1,k2,k3),"linear");
                    end
                end
            end
        end
    end 

    % Sort alpha
    [gg(l).alpha, ia] = sort(gg(l).alpha);
    % Sort V
    [gg(l).V, iV] = sort(gg(l).V);
    % Sort gtilda
    [gg(l).g, ig] = sort(gg(l).g);
    % Sort rho
    for k0 = 1 : numel(varargin)
        field = varargin{k0};
        gg(l).(field) = gg(l).(field)(ia, iV, ig, :);
    end

    % Add more V points at the boundary
    % This is just to avoid that interpolant returns 0 when being evaluated at
    % the boundary (due to machine precision errors)
    dV = gg(l).V(2)-gg(l).V(1);
    gg(l).V = [gg(l).V(1)-dV, gg(l).V, gg(l).V(end)+dV];
    for k0 = 1 : numel(varargin)
        field = varargin{k0};
        gg(l).(field) = cat(2,gg(l).(field)(:,1,:,:), gg(l).(field), gg(l).(field)(:,end,:,:));
    end

    % Add more gz points at the boundary
    % This is just to avoid that interpolant returns 0 when being evaluated at
    % the boundary (due to machine precision errors)
    % This also covers the case when 2D g-g only is provided: in such case
    % g vals are gg(l).g+[-1,-0.5,0.5,1], so that 3D interpolation still works
    % with the minimum overhead
    if numel(gg(l).g)>1 % 3D g-g (more than two g values)
        dg = gg(l).g(2)-gg(l).g(1); % assumed equally-spaced
    else
        dg = 0.5;
        gg(l).g = gg(l).g + [-dg, dg];
        for k0 = 1 : numel(varargin)
            field = varargin{k0};
            gg(l).(field) = repmat(gg(l).(field)(:,:,1,:), [1 1 numel(gg(l).g)]);
        end
    end
    gg(l).g = [gg(l).g(1)-dg, gg(l).g, gg(l).g(end)+dg];
    for k0 = 1 : numel(varargin)
        field = varargin{k0};
        gg(l).(field) = cat(3,gg(l).(field)(:,:,1,:), gg(l).(field), gg(l).(field)(:,:,end,:));
    end

    % Repeat G-G cyclically to add points at the boundaries
    % this also allows symmetrization for single-sided G-G
    % if double-sided G-G provided, this just repeats the G-G and still works fine
    cyclicAngle = 2*gg(l).alpha(end); % angle to add/subtract to enforce cyclicity
    gg(l).alpha = cat(2,-cyclicAngle-flip(gg(l).alpha,2), ...
                         gg(l).alpha(2:end-1), ...
                         cyclicAngle-flip(gg(l).alpha,2));
    for k0 = 1 : numel(varargin)
        field = varargin{k0};
        gg(l).(field) = cat(1,flip(gg(l).(field),1), gg(l).(field)(2:end-1,:,:,:), flip(gg(l).(field),1));
    end

    % Rem some vals at the bounds to reduce overhead
    Naddpts = 5; % hardcoded 5 points before and after +/-pi (enough for both linear and bspline)
    dalpha = gg(l).alpha(2)-gg(l).alpha(1); % delta alpha
    idx2keep = (gg(l).alpha<=(pi+Naddpts*dalpha)) & (gg(l).alpha>=-(pi+Naddpts*dalpha)); % keep alpha within range
    gg(l).alpha = gg(l).alpha(idx2keep);
    for k0 = 1 : numel(varargin)
        field = varargin{k0};
        gg(l).(field) = gg(l).(field)(idx2keep,:,:,:);
    end
    
    % interpolate
    % 3D g-g map: func(alpha,V,g)
    for k0 = 1 : numel(varargin)
        field = varargin{k0};
        switch backend
            case 'casadi'
                if strcmp(method, 'spline')
                    method = 'bspline';
                end
                for ii = 1 : size(gg(l).(field), 4)
                    vv = gg(l).(field)(:,:,:,ii);
                    varargout{k0}{l}{ii} = casadi.interpolant(sprintf('gg%d_%s%d', l, field, ii), method, {gg(l).alpha, gg(l).V, gg(l).g}, vv(:));
                end
            case 'matlab'
                if strcmp(method, 'bspline')
                    method = 'spline';
                end
                for ii = 1 : size(gg(l).(field), 4)
                    vv = gg(l).(field)(:,:,:,ii);
                    varargout{k0}{l}{ii} = griddedInterpolant({gg(l).alpha, gg(l).V, gg(l).g}, vv, method);
                end
            otherwise
                error('carfm:invalidType','backend must be either ''matlab'' or ''casadi''');
        end
    end
end

end