function ssdata = ssa(car,u_long0,u_lat0,opts,guess)
%SSA - See help/ssa.m for the help of this function.

% Checks args
arguments
    car (1,1) struct {carfm.common.checkCarStruct(car)}
    u_long0 (3,:) {mustBeNumeric,mustBeReal}
    u_lat0 (3,:) {mustBeNumeric,mustBeReal,carfm.common.mustBeEqualSize2(u_lat0,u_long0)}
    opts (1,1) struct {carfm.ssa.checkOptStruct(opts)} = struct()
    guess (1,:) struct {carfm.ssa.checkSSAguess(guess,u_long0)} = struct()
end

% Add path and import
filepath = fileparts(mfilename('fullpath')); % path of current file
[flag, errid, errmsg] = carfm.common.setEnvironment(filepath); % check tools and set path
if flag
    % Reset and exit
    clearvars -except filepath errmsg
    carfm.common.setEnvironment(filepath, false);
    % Give error if sth went wrong
    error(errid, '%s', errmsg);
end

% Default options and override with user-defined options
opts = carfm.ssa.getDefaultOptions(opts);

% Generate symbolic expression using CasADi only if ~usePrebuilt or ~mex
% this is to reduce overhead when not necessary, thus reducing function inizialization time
if ~opts.mex || ~opts.usePrebuilt
    % Create vars
    V = casadi.MX.sym('V');
    a__t = casadi.MX.sym('a__t');
    Tau__t = casadi.MX.sym('Tau__t');
    yaw__rate = casadi.MX.sym('yaw__rate');
    delta = casadi.MX.sym('delta');
    lambda = casadi.MX.sym('lambda');

    % Create u_long and u_lat
    u_long = [V; a__t; Tau__t];
    u_lat = [yaw__rate; delta; lambda];

    % Create x = [phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
    %             X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
    %             F__fl, F__fr, F__rl, F__rr, Tau__t, V, yaw__rate, a__t]
    x = casadi.MX.sym('x', 33, 1);

    % Create system function zero = f(x, u_long, u_lat)
    car_casadi = carfm.common.matlab2casadiInterpolant(car);
    [ssres, ssdata] = carfm.ssa.steadyStateEquations(x, u_long, u_lat, car_casadi, opts);
    f = casadi.Function('f', {x, u_long, u_lat}, {ssres, ssdata.residual});
    f2 = casadi.Function('f2', {x, u_long, u_lat}, {ssres, jacobian(ssres, x)});

    % Create guess function xg = guess(u_long, u_lat)
    xg = carfm.ssa.steadyStateGuess(u_long, u_lat, car_casadi, opts);
    guessfun = casadi.Function('guess', {u_long, u_lat}, {xg});

    % Solver to find zero of f
    solver = casadi.rootfinder('solver', opts.ssSolver, f, struct('error_on_fail', false));
end

% Mex
% for mex=true, force build if buildOnly=true, regardless usePrebuilt
if opts.mex && (~opts.usePrebuilt || opts.buildOnly)
    [flag, errid, errmsg] = carfm.ssa.mexFile(opts.ssaMexName, guessfun, solver, f2); % mex functions
    if flag
        % Reset and exit
        clearvars -except filepath errmsg
        carfm.common.setEnvironment(filepath, false);
        % Give error if sth wrong
        error(errid, '%s', errmsg);
    end
end

% Check for build only
if opts.mex && opts.buildOnly
    % Reset and exit
    clearvars -except filepath
    carfm.common.setEnvironment(filepath, false);
    ssdata = struct([]); % empty output argument
    return;
end

% Mex exists?
if opts.mex 
    if exist(opts.ssaMexName, 'file')~=3 % id=3 for mex file (https://it.mathworks.com/help/matlab/ref/exist.html)
        % Reset and exit
        clearvars -except filepath opts
        carfm.common.setEnvironment(filepath, false);
        % Give error
        error('carfm:notFound','Unable to find MEX function %s.%s', opts.ssaMexName, mexext)
    end
end

% Get function
if opts.mex
    % Function handles for easier access to the functions in the mex
    ssasolver_fh = str2func(opts.ssaMexName); % function handle for ssasolver
    eval_guess = @(varargin) ssasolver_fh('guess', varargin{:});
    solver = @(x, varargin) ssasolver_fh('solver', x, varargin{:});
    eval_f2 = @(x, varargin) ssasolver_fh('f2', x, varargin{:});
else
    % Function handles which return data in double (and not casadi.DM type)
    eval_guess = returntypes('full', guessfun);
    solver = returntypes('full', solver);
    eval_f2 = returntypes('full', f2);
end

% Solve SSA
clear ssdata
Xsol = nan(numel(opts.xscale), size(u_long0,2)); % matrix containing solution x
% Loop over V
% tic;
for k = 1 : size(u_long0,2)
    % Create arguments for solver
    args = {u_long0(:,k), u_lat0(:,k)};
    % Get guess x
    if isempty(fieldnames(guess))
        if ~opts.useLastSSA || k==1
            x0 = eval_guess(args{:});
        end
    else % use guess struct
        x0 = guess(min(k,numel(guess))).xss;
    end
    % Call to solver
    [Xsol(:,k), res0] = solver(x0, args{:});
    % Check residual - use fsolve to get guess if failed
    if not(res0 < opts.rtol)
        xtmp = fsolve(@(x) eval_f2(x, args{:}), x0, ...
                optimoptions('fsolve','SpecifyObjectiveGradient',true,'Display','none'));
        Xsol(:,k) = solver(xtmp, args{:});
    end
    % Post processing
    [~, ssdata(k)] = carfm.ssa.steadyStateEquations(Xsol(:,k), u_long0(:,k), u_lat0(:,k), car, opts);
    if ssdata(k).exitcode ~= 0
        % Give warning for failed or invalid solution
        fprintf('%s at index=%d (exitcode %d)\n', ssdata(k).exitmsg, k, ssdata(k).exitcode);
    end
    % Set next guess
    if opts.useLastSSA && ssdata(k).exitcode >= 0 
        x0 = Xsol(:,k);
    end
end
% toc;

% Clear mex memory
if opts.mex
    clear(opts.ssaMexName);
end

% Rem path
clearvars -except ssdata filepath
carfm.common.setEnvironment(filepath, false);

end
