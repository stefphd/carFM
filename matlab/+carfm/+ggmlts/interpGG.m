function varargout = interpGG(gg, backend, method, varargin)
%INTERPGG Interpolate the g-g map using CASADI interpolant.
%
% INPUT:
% gg: g-g map struct (possibly vector)
% backend: backend to use ('matlab', 'casadi', 'internal')
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
    % sizes
    % na = numel(gg(l).alpha);
    nV = numel(gg(l).V);
    ng = numel(gg(l).g);

    % fill missing for each GG section
    % loop over grid using linear index
    for i = 1 : nV*ng
        [iV,ig] = ind2sub([nV,ng],i);
        idx = {iV,ig};
        for k = 1 : numel(varargin) % loop fields to be interps
            field = varargin{k};
            inan = any(isnan(gg(l).(field)(:,idx{:},:)), 8);
            if any(inan(:))
                fprintf("Warning: filled %d nan values in gg(%d).%s at V(%d)=%g, g(%d)=%g\n", ...
                    sum(inan(:)), l, field, iV, gg(l).V(iV), ig, gg(l).g(ig));
                for j = 1 : size(gg(l).(field), 8)
                    gg(l).(field)(:,idx{:},j) = fillmissing(gg(l).(field)(:,idx{:},j),"linear");
                end
            end
        end
    end

    % sort grid vectors
    grid_fields = {'alpha', 'V', 'g'};
    idx = cell(size(grid_fields));
    for j = 1 : numel(grid_fields)
        field = grid_fields{j};
        [gg(l).(field), idx{j}] = sort(gg(l).(field));
    end
    for k = 1 : numel(varargin)
        field = varargin{k};
        gg(l).(field) = gg(l).(field)(idx{:}, :);
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
    
    % interpolation flags for alpha,V,g
    % basically, flag is true when the related variable needs to be
    % included in the interpolation construction and evaluation,and false
    % when the dimension is singleton.
    % note that the flag for alpha is always true, ie. interpVar(1) = true,
    % as in carfm.ggmlts.checkGG it is validated that numel(alpha)>1
    interpVar = true(1,3); % init to true
    grid_sz = size(gg(l).(field), 1:3);
    interpVar(grid_sz==1) = false; 

    % add more points at the boundary if interpVar(k) is true.
    % This is just to avoid that interpolant returns 0 when being evaluated at
    % the boundary (due to machine precision errors). GG are added to
    % maintain the same slope at the boundaries.
    % This is done only for speed V and gravity g (if not singleton), as
    % these are OCP variables (or depend on OCP variables) and out-of-bound 
    % evaluation may occur.
    % The remaining variables (Tf,...,wr) are not OCP variable, but known
    % function which can be saturated with a tolerance to avoid
    % out-of-bound evaluations.
    % speed V
    if interpVar(2) 
        dV = gg(l).V(2)-gg(l).V(1); % equally-spaced
        gg(l).V = [gg(l).V(1)-dV, gg(l).V, gg(l).V(end)+dV];
        for k = 1 : numel(varargin)
            field = varargin{k};
            gg(l).(field) = cat(2,...
                gg(l).(field)(:,1,:,:,:,:,:,:) - ( gg(l).(field)(:,2,:,:,:,:,:,:) - gg(l).(field)(:,1,:,:,:,:,:,:) ), ...
                gg(l).(field), ...
                gg(l).(field)(:,end,:,:,:,:,:,:) + ( gg(l).(field)(:,end,:,:,:,:,:,:) - gg(l).(field)(:,end-1,:,:,:,:,:,:) ) );
        end
    end
    % gravity g
    if interpVar(3)
        dg = gg(l).g(2)-gg(l).g(1); % equally-spaced
        gg(l).g = [gg(l).g(1)-dg, gg(l).g, gg(l).g(end)+dg];
        for k = 1 : numel(varargin)
            field = varargin{k};
            gg(l).(field) = cat(3, ...
                gg(l).(field)(:,:,1,:,:,:,:,:) - ( gg(l).(field)(:,:,2,:,:,:,:,:) - gg(l).(field)(:,:,1,:,:,:,:,:) ), ...
                gg(l).(field), ...
                gg(l).(field)(:,:,end,:,:,:,:,:) + ( gg(l).(field)(:,:,end,:,:,:,:,:) - gg(l).(field)(:,:,end-1,:,:,:,:,:) ) );
        end
    end

    % repeat GG cyclically to add points at the boundaries
    % this also allows symmetrization for single-sided GG
    % if double-sided GG provided, this just repeats the GG and still works fine
    cyclicAngle = 2*gg(l).alpha(end); % angle to add/subtract to enforce cyclicity
    gg(l).alpha = cat(2,-cyclicAngle-flip(gg(l).alpha,2), ...
                         gg(l).alpha(2:end-1), ...
                         cyclicAngle-flip(gg(l).alpha,2));
    for k = 1 : numel(varargin)
        field = varargin{k};
        gg(l).(field) = cat(1,flip(gg(l).(field),1), gg(l).(field)(2:end-1,:,:,:,:,:,:,:), flip(gg(l).(field),1));
    end

    % interpolation variables
    args = cell(size(grid_fields));
    for j = 1 : numel(grid_fields)
        field = grid_fields{j};
        args{j} = gg(l).(field);
    end
    
    % interpolate Y(alpha,V,g)
    % note: 'matlab' and 'internal' backends are able to deal with grids
    % where the length of the grid vector is incompatible with the order of
    % the spline: spline order is reduced for such dimensions accordingly.
    % instead, 'casadi' backend requires same order for all directions, and
    % gives error if more data points needed: needs check in such case
    % check sz of non-singleton dimensions
    if any(grid_sz(interpVar)<4) 
        if strcmp(backend, 'casadi') % error for casadi backend
            error('carfm:invalidValue','Size of non-singleton dimensions in GG must be at least 4 when casadi backend is used\n');
        else % just a warning for matlab and internal backend
            fprintf('Warning: one or more non-singleton dimensions in GG have size less than 4: spline order is adjusted accordingly\n');
        end
    end
    for k = 1 : numel(varargin)
        field = varargin{k};
        for ii = 1 : size(gg(l).(field), 8)
            Y = gg(l).(field)(:,:,:,:,:,:,:,ii);
            Y = squeeze(Y); % remove singleton dimension
            switch backend
            case 'casadi' % using casadi.interpolant
                if strcmp(method, 'spline')
                    method = 'bspline';
                end
                varargout{k}{l}{ii} = casadi.interpolant(sprintf('gg%d_%s%d', l, field, ii), method, args(interpVar), Y(:));
            case 'internal' % using carfm.common.interpolant
                if strcmp(method, 'bspline')
                    method = 'spline';
                end
                varargout{k}{l}{ii} = carfm.common.interpolant(sprintf('gg%d_%s%d', l, field, ii), args(interpVar), Y, method);
            case 'matlab' % using matlab griddedInterpolant
                if strcmp(method, 'bspline')
                    method = 'spline';
                end
                varargout{k}{l}{ii} = griddedInterpolant(args(interpVar), Y, method);
            otherwise
                error('carfm:invalidType','backend must be either ''matlab'', ''casadi'', or ''internal''.');
            end
        end
        varargout{k}{l}{end+1} = interpVar; % save interpVar at the end
    end
end

end