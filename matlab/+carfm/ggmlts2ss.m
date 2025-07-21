function mlts = ggmlts2ss(car, gg, mlts, opts)
% GGMLTS2SS - See help/ggmlts2ss.m for the help of this function.

% Checks args
arguments
    car (1,1) struct {carfm.common.checkCarStruct(car)}
    gg (1,1) struct {carfm.ggmlts.checkGGStruct(gg)}
    mlts (1,1) struct {carfm.ssa.checkMLTSStruct(mlts)}
    opts (1,1) struct {carfm.ssa.checkOptStruct(opts)} = struct()
end

% Default options and override with user-defined options
opts = carfm.ssa.getDefaultOptions(opts);

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

% Interp gg.x 
xinterp = carfm.ggmlts.interpGG(gg, 'matlab', 'bspline', 'x');
% eval interp
X = nan(size(gg.x,4), numel(mlts.s));
for k = 1 : size(gg.x,4)
    X(k,:) = xinterp{1}{k}(mlts.alpha, mlts.V, mlts.geq / car.gravity);
end
% fix sign and left/right for single-sided G-G
isSingleSidedGG = gg.alpha(end)<3/4*pi; % single-sided G-G b/c alpha(end)<135deg
if isSingleSidedGG 
    isym = cos(mlts.alpha)<0; % cos(alpha)<0 is GG left-hand-side
    % change sign
    X(:,isym) = X(:,isym) .* opts.xsign(:);
    % switch left/right
    for l = 1 : size(opts.xswap,1)
        xtmp = X(opts.xswap(l,1),isym); % tmp for swap
        X(opts.xswap(l,1),isym) = X(opts.xswap(l,2),isym); % swap 2->1
        X(opts.xswap(l,2),isym) = xtmp; % swap 1->2
    end
end

% Flag for points that are not on the g-g boundary
% check ratio between rho and rhomax, that is near 1 when on g-g
igg = (mlts.rho ./ mlts.rhomax) > opts.GGRadiusTol; % true if on g-g boundary

% Create symbolic alpha, V, gz, at0
% SS vector x = [phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
%                X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
%                F__fl, F__fr, F__rl, F__rr, Tau__t, V, yaw__rate, a__t]
x = casadi.MX.sym('x', 33, 1);
alpha = casadi.MX.sym('alpha');
V = casadi.MX.sym('V');
g__z = casadi.MX.sym('g__z');
a__t0 = casadi.MX.sym('a__t0'); % a__t shift
rho = casadi.MX.sym('rho');
args = {alpha, V, g__z, a__t0, rho};

% Set SS inputs
a__t = (rho*sin(alpha) + a__t0) * car.gravity;
a__n = rho*cos(alpha) * car.gravity;
yaw__rate = a__n/V;
u_long = [V; a__t; 0]; 
u_lat = [yaw__rate; 0; 0];

% Set options
opts.ssActiveLongInputs = [true, true, false]; % active V, a__t
opts.ssActiveLatInputs = [true, false, false]; % active yaw__rate
opts.gz_g = g__z / car.gravity;

% Create system function zero = f(x, u__long, u__lat)
car_casadi = carfm.common.matlab2casadiInterpolant(car);
[ssres, ssdata] = carfm.ssa.steadyStateEquations(x, u_long, u_lat, car_casadi, opts);
f = casadi.Function('f', [{x}, args], {ssres, ssdata.residual});
f2 = casadi.Function('f2', [{x}, args], {ssres, jacobian(ssres, x), ssdata.residual});

% Solver to find zero of f
solver = casadi.rootfinder('solver', opts.ssSolver, f, struct('error_on_fail', false));

% Optimization problem for possible refine
% L = (rho - sqrt((ssdata.tangentialAcc/bike.gravity - a__t0)^2+(ssdata.lateralAcc/bike.gravity)^2))^2;
L = ((ssdata.tangentialAcc - a__t)/car.gravity)^2 + ...
    ((ssdata.normalAcc - a__n)/car.gravity)^2;
f1 = casadi.Function('f1', [{x}, args], {L, gradient(L,x)});
geq = ssres(1:end-2); % exclude a__t and yaw__rate equation, as they are in the cost L
g1 = casadi.Function('g1', [{x}, args], {[], geq, [], jacobian(geq, x)'});

% Mex
% for mex=true, force build if buildOnly=true, regardless usePrebuilt
if opts.mex && (~opts.usePrebuilt || opts.buildOnly)
    [flag, errid, errmsg] = carfm.ssa.mexFile(opts.ggmlts2ssMexName, solver, f1, g1); % mex functions
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
    mlts = struct(); % empty output argument
    return;
end

% Mex exists?
if opts.mex 
    if exist(opts.ggmlts2ssMexName, 'file')~=3 % id=3 for mex file (https://it.mathworks.com/help/matlab/ref/exist.html)
        % Reset and exit
        clearvars -except filepath opts
        carfm.common.setEnvironment(filepath, false);
        % Give error
        error('carfm:notFound','Unable to find MEX function %s.%s', opts.ggmlts2ssMexName, mexext)
    end
end

% Get function
if opts.mex
    % Function handles for easier access to the functions in the mex
    gg2sssolver_fh = str2func(opts.ggmlts2ssMexName); % function handle for gg2sssolver
    solver = @(x, varargin) gg2sssolver_fh('solver', x, varargin{:});
    eval_cost = @(x, varargin) gg2sssolver_fh('f1', x, varargin{:});
    eval_nlcon = @(x, varargin) gg2sssolver_fh('g1', x, varargin{:});
else
    % Function handles which return data in double (and not casadi.DM type)
    solver = returntypes('full', solver);
    eval_cost = returntypes('full', f1);
    eval_nlcon = returntypes('full', g1);
end
eval_f2 = returntypes('full', f2);

% Solve SSA
clear ssdata
u_long0 = nan(3,1); 
u_lat0 = nan(3,1);
opts.ssActiveLongInputs = false(1,3); % discard active long eqs
opts.ssActiveLatInputs = false(1,3); % discard active lat eqs
x0 = X(:,1); % init
k = 1; % init
% tic;
while k <= numel(mlts.s)
    if igg(k)
        x0 = X(:,k); % take current x
        xsol = x0;
    end
    % check if solving needed
    if ~igg(k) % Point not on GG
        if (k>1) && igg(k-1) % previous point was on g-g: try to relax some components of x0
            % SS vector x = [phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
            %                X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
            %                F__fl, F__fr, F__rl, F__rr, Tau__t, V, yaw__rate, a__t]
            % Relaxed components are:
            % phi, z, delta, lambda,
            % kappa__fl, kappa__fr, kappa__rl, kappa__rr,
            % X__fl, X__fr, X__rl, X__rr, Y__fl, Y__fr, Y__rl, Y__rr,
            % Tau__t, yaw__rate, a__t
            irelax = [1 3 4 9 10 11 12 13 14 15 16 17 18 19 20 21 30 32 33]; % x to relax
            x0(irelax) = mlts.rho(k)/mlts.rhomax(k)*x0(irelax); 
        end
        % set args: alpha, V, g__z, a__t0, rho
        args = {mlts.alpha(k), mlts.V(k), mlts.geq(k), mlts.at0(k), mlts.rho(k)};
        % Call to solver
        [xsol, res0] = solver(x0, args{:});
        % Check residual - use fsolve to get guess if failed
        if not(res0 < opts.rtol)
            xtmp = fsolve(@(x) eval_f2(x, args{:}), x0, ...
                    optimoptions('fsolve','SpecifyObjectiveGradient',true,'Display','none'));
            [xsol, res0] = solver(xtmp, args{:});
        end
        % Check residual - use fsolve 
        if not(res0 < opts.rtol)
            [xsol, fval] = fsolve(@(x) eval_f2(x, args{:}), x0, ...
                    optimoptions('fsolve','SpecifyObjectiveGradient',true,'Display','none'));
            res0 = norm(fval);
        end
        % Set solution
        x0 = xsol;
        if not(res0 < opts.rtol)
            x0(:) = X(:,k);
            xsol(:) = nan;
            fprintf("SS failed at s(%d)=%.3f (rho/rhomax=%.3f)\n", k, mlts.s(k), mlts.rho(k) / mlts.rhomax(k));
        end
    end    
    % post process
    opts.gz_g = mlts.geq(k) / car.gravity;
    [~, ssdata(k)] = carfm.ssa.steadyStateEquations(xsol, u_long0, u_lat0, car, opts);
    % update
    k = k +1;
end
% toc

% Refine each MLTS point by solving an optimization problem that ensures
% SSA is solved exactly
% Note that the above procedure is still performed, so that good guesses
% are obtained
% tic
if opts.refineMLTS
    % fmincon options
    opti_opt = optimoptions('fmincon',...
                           'Algorithm', opts.algorithmGGopt, ...
                           'Display', 'none',...
                           'SpecifyObjectiveGradient', true,...
                           'SpecifyConstraintGradient', true, ...
                           'SubproblemAlgorithm','factorization' ...
                           ); 
    % Upper and lower bounds for x
    % determine bounds from mlts solution
    lbx = min(X, [], 2) - 0.5*rms(X, 2) - 1;
    ubx = max(X, [], 2) + 0.5*rms(X, 2) + 1;
    % Cycle over MLTS points
    for k = 1 : numel(mlts.s)
        % Skip points inside GG (already solved in such case)
        if ~igg(k) 
            continue;
        end
        % Skip points outside opts.refineRange
        if not(any((mlts.s(k) >= opts.refineRange(:,1)) & (mlts.s(k) <= opts.refineRange(:,end))))
            continue;
        end
        % Set args
        x0 = ssdata(k).xss;
        args = {mlts.alpha(k), mlts.V(k), mlts.geq(k), mlts.at0(k), mlts.rho(k)};
        % Call to fmincon
        [xsol,~,exitflag] = fmincon(@(x) eval_cost(x, args{:}), x0,[],[],[],[],...
                lbx,ubx,@(x) eval_nlcon(x,args{:}),opti_opt);
        % Check exitflag
        if exitflag <= 0 % refine failed: notify user
            % Do not update ssdata: keep non-refined
            fprintf("Refining failed at s(%d)=%.3f\n", k, mlts.s(k));
        else
            % Update ssdata with the refined solution
            opts.gz_g = mlts.geq(k) / car.gravity;
            [~, ssdata(k)] = carfm.ssa.steadyStateEquations(xsol, u_long0, u_lat0, car, opts);
        end
    end
end
% toc;

% Add ssdata to mlts
mlts.data = ssdata;

% Clear mex memory
if opts.mex
    clear(opts.ggmlts2ssMexName);
end

% Rem path
clearvars -except mlts filepath
carfm.common.setEnvironment(filepath, false);

end

