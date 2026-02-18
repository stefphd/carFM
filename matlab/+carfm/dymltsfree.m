function mltsout = dymltsfree(car, track, opts, guess)
%DYMLTSFREE - See help/dymltsfree.m for the help of this function.

% Check args
arguments
    car (1,1) struct {carfm.common.checkCarStruct(car)}
    track (1,1) struct {carfm.common.checkTrackStruct(track)}
    opts (1,1) struct {carfm.dymlts.checkOptStruct(opts)} = struct()
    guess (1,1) struct {carfm.common.checkGuessStruct(guess, {'s','n','chi','data'})} = struct()
end

% Imports
filepath = fileparts(mfilename('fullpath')); % path of current file
[flag, errid, errmsg] = carfm.common.setEnvironment(filepath); % check tools and set path
if flag
    % Reset and exit
    clearvars -except filepath errmsg
    carfm.common.setEnvironment(filepath, false);
    % Give error if sth went wrong
    error(errid, '%s', errmsg);
end

% CasADi options
carfm.common.setMaxNumDir(8); % 8 to speed up cc time

% Default options and override with user-defined options
opts = carfm.dymlts.getDefaultOptions(opts);

% Get undocumented options and override with user-defined options
opts = carfm.dymlts.getUndocOptions(opts);

% Check if sensitivity parameters exist in car struct
[flag, errid, errmsg] = carfm.common.checkSensitivityPar(opts.sensitivityPar, car);
if flag
    % Reset and exit
    clearvars -except filepath errmsg errid
    carfm.common.setEnvironment(filepath, false);
    % Give error if sth went wrong
    error(errid, '%s', errmsg);
end

% Extract sensitivity parameters
opts.sensitivityPar = carfm.common.expandSensitivityPar(opts.sensitivityPar, car);
np = numel(opts.sensitivityPar); % num of sensitivies

% Create aux
aux.car = carfm.common.matlab2casadiInterpolant(car);
aux.track = carfm.common.interpTrack(track, 'casadi', opts.trackinterpMethod, opts.minDecLen);

% Check interpolant types
carfm.dymlts.checkInterpolantType(car, {'spline','bspline'}); % suggest to use spline/bspline

% Find si and sf
if opts.sRange(1) <= track.s(1)
    si = track.s(1);
else
    si = opts.sRange(1);
end
if opts.sRange(end) >= track.s(end)
    sf = track.s(end);
else
    sf = opts.sRange(end);
end

% Computation domain
ti = si / opts.sscale;
tf = sf / opts.sscale;

% Create vars
nx = 23+2+2; nu = 2+np; % include sensitivities in control u
x = casadi.MX.sym('x', nx); % state
xp = casadi.MX.sym('xp', nx); % state derivative
u = casadi.MX.sym('u', nu); % control
t = casadi.MX.sym('t'); % independent variab
x0 = casadi.MX.sym('x0', nx); % initial state
u0 = casadi.MX.sym('u0', nu); % initial control
xn = casadi.MX.sym('xn', nx); % final state
un = casadi.MX.sym('un', nu); % final control

% Parameters
p = u(3:end); % 0x1 if no sensitivities
[aux.car, p0] = carfm.common.replaceField(aux.car, opts.sensitivityPar, p);

% Model equations
% r: system dynamics
% c: path constraint
% l: Lagrange term
[r, c, l] = carfm.dymlts.ocpEquations(t,x,xp,u(1:end-np),aux,opts);
nc = numel(c);
% bcs
try
    b = opts.bcsFunc(x0(1:25)   .* opts.xscale(:), ...
                     x0(26:end) .* opts.uscale(:), ...
                     xn(1:25)   .* opts.xscale(:), ...
                     xn(26:end) .* opts.uscale(:));
catch err
    % Reset and exit
    clearvars -except filepath err
    carfm.common.setEnvironment(filepath, false);
    error('carfm:unableEval', 'Unable to ealuate "bcsFunc" function handle: %s', err.message);
end
% Undocumented feature: relax boundary conditions using undocumented 
% option 'opts.bcsRelax' (default false): BCs are removed and added into 
% the OCP cost as a Mayer term. This may be usefull when starting from 
% unfeasible BCs, but has not been extensively tested yet.
if ~opts.bcsRelax % Default
    m = 0; % no mayer
else 
    m = sum(b.^2);
    b = []; % no bcs
end
nb = numel(b);

% Model functions
f = casadi.Function('f', {t, x, xp, u}, {r, l});
g = casadi.Function('g', {t, x, u}, {c});

% Transcribe single mesh interval
x1 = casadi.MX.sym('x1_0', nx); % initial state of interval 
x2 = casadi.MX.sym('x2_0', nx); % final state of interval 
u1 = casadi.MX.sym('u1_0', nu); % initial control of interval 
u2 = casadi.MX.sym('u2_0', nu); % final control of interval 
h = casadi.MX.sym('h'); % time step
w0 = {x0}; % augmented inital state
wn = {xn}; % augmented final state
z0 = {u0}; % augmented inital control
zn = {un}; % augmented final control
w1 = {x1}; % augmented state at the start of interval
w2 = {x2}; % augmented state at the end of interval
z1 = {u1}; % augmented control at the start of interval
z2 = {u2}; % augmented control at the end of interval
if contains(opts.scheme, 'lgr')  
    % Get collocations - LGR (GPOPS2-like)
    d = sscanf(opts.scheme, 'lgr%d');
    % Collocated points are the initial point j=0 of mesh interval and j=1,2,..,d-1 inner points
    % Non-collocated point is the end point j=d of mesh interval
    % see also https://doi.org/10.1145/25589
    tau_root = casadi.collocation_points(d, 'radau'); % this uses end point as collocated point, i.e. in (0, 1]
    % D: LG differentiation matrix (dim d+1-by-d)
    % V: LG interpolation weights (dim d+1-by-d) - NO USED for LGR
    % W: LG quadrature weights (dim d-by-1)
    [D, ~, W] = casadi.collocation_coeff(tau_root); % this automatically adds initial (non-collocated point) at tau=0
    D = full(D); W = full(W);
    % Reverse LGR to use first point instead of end point as collocated point for collocation points in [0, 1)
    D = -rot90(D,2); % equivalent to -flipud(fliplr(D))
    W = flip(W); 
    tau_root = 1 - fliplr(tau_root); % [0, 1) range
    % add end (non-collocated) point
    tau_root = [tau_root, 1]; % [0, 1] range
    W = [W; 0]; % weigth of end point is 0
    % Create augmented state and control including collocation points
    % Augmented state is w = [x_0, x_1, x_2, ..., x_(d-1)], where
    % x_0: state at the start of the interval
    % x_1,x_2,...,x_(d-1): states at the collocation points
    % Augmented control is z = [u_1, u_2, ..., u_(d-1)], where 
    % u_1,u_2,...,u_(d-1): controls at the collocation points
    for j = 1:d-1 % Loop over collocation points
        w0 = [w0; {casadi.MX.sym(['x0_' num2str(j)], nx)}];
        wn = [wn; {casadi.MX.sym(['xn_' num2str(j)], nx)}];
        z0 = [z0; {casadi.MX.sym(['u0_' num2str(j)], nu)}];
        zn = [zn; {casadi.MX.sym(['un_' num2str(j)], nu)}];
        w1 = [w1; {casadi.MX.sym(['x1_' num2str(j)], nx)}];
        w2 = [w2; {casadi.MX.sym(['x2_' num2str(j)], nx)}];
        z1 = [z1; {casadi.MX.sym(['u1_' num2str(j)], nu)}];
        z2 = [z2; {casadi.MX.sym(['u2_' num2str(j)], nu)}];
    end
    % Collocation equations
    w = [w1; w2(1)]; % collocation pts + non-collocated point
    dw = []; % collocation equations
    c = []; % path constraints
    dl = 0; % cost over one mesh interval
    for j = 1:d % loop over collocation points
        tj = t+tau_root(j)*h; % time at collocation point j-th
        xj = w{j}; % state at collocation point j-th
        uj = z1{j}; % control at collocation point j-th
        % state derivative at the collocation point j-th
        xpj = 0; 
        for r=1:d+1
            xpj = xpj + D(r,j)*w{r};
        end
        % collocation equations using LGR differentiation
        [fj, lj] = f(tj, xj, xpj/h, uj);
        dw = [dw; fj];
        % contribution to quadrature functions
        dl = dl + W(j)*lj*h;
        % path constraints
        cj = g(tj, xj, uj);
        c = [c; cj];
    end
else % euler,trapz,midpint
    d = 1; % w=x, z=u
    tau_root = [0 1];
    W = [1 0]'; % unitary weight
    xp12 = (x2-x1)/h; % finite derivative
    switch opts.scheme
        case 'euler' % Euler integration
            [f1, l1] = f(t,x1,xp12,u1); 
            c1 = g(t,x1,u1);
            dw = f1;
            dl = h*l1;
            c = c1;
        case 'midpoint'
            [f1, l1] = f(t+.5*h,0.5*(x1+x2),xp12,0.5*(u1+u2)); 
            c1 = g(t,x1,u1);
            dw = f1;
            dl = h*l1;
            c = c1;
        % case 'trapz' % Trapezoidal rule (fallback)
        otherwise % Fallback to 'trapz'
            [f1, l1] = f(t,x1,xp12,u1);
            [f2, l2] = f(t+h,x2,xp12,u2);
            c1 = g(t,x1,u1);
            dw = 0.5*(f1+f2);
            dl = 0.5*h*(l1+l2);
            c = c1;
    end
end
% Scaling to dw already applied into ocpEquations

% CAT augmented state & control to obtain vectors instead of cells
w1 = vertcat(w1{:});
z1 = vertcat(z1{:});
w2 = vertcat(w2{:});
z2 = vertcat(z2{:});
w0 = vertcat(w0{:});
wn = vertcat(wn{:});
z0 = vertcat(z0{:});
zn = vertcat(zn{:});

% Create OCP functions
null = casadi.MX.sym('p',0);
ocp_runcost = casadi.Function('run_cost', {t, w1, z1, w2, z2, null, h}, {sum(dl)});
ocp_runcost2 = casadi.Function('run_cost2', {t, w1, z1, w2, z2, null, h}, {dl}); % separates laptime and penalty
ocp_bcscost = casadi.Function('bcs_cost', {w0, z0, wn, zn, null}, {m});
ocp_dyn = casadi.Function('dyn_constr', {t, w1, z1, w2, z2, null, h}, {dw});
ocp_path = casadi.Function('path_constr', {t, w1, z1, null, h}, {c});
ocp_bcs = casadi.Function('bcs_constr', {w0, z0, wn, zn, null}, {b});
ocp_int = casadi.Function('int_constr', {t, w1, z1, w2, z2, null, h}, {[]});

% Bounds
% x = [ phi, mu, z, delta,
%       z__fl, z__fr, z__rl, z__rr, 
%       V__P, lambda__P, V__z, 
%       delta__dot, z__fldot, z__frdot, z__rldot, z__rrdot, 
%       Omega__z, Omega__x, Omega__y, 
%       omega__fl, omega__fr, omega__rl, omega__rr
%       n, chi,
%       delta__ddot, Tau__t]
lbx = [[-15/180*pi; -15/180*pi; -0.25; -30/180*pi;
        -0.5; -0.5; -0.5; -0.5;
        5; -0.2; -5;
        -0.5; -0.5; -0.5; -0.5; -0.5;
        -5; -0.1; -0.1;
        15; 15; 15; 15;
        -10; -0.5
       ] ./ opts.xscale(:);
       [-1; -10e3] ./ opts.uscale(:)
    ];
ubx = [[15/180*pi; 15/180*pi; 0.25; 30/180*pi;
        0.5; 0.5; 0.5; 0.5;
       110; 0.2; 5;
       0.5; 0.5; 0.5; 0.5; 0.5;
       5; 0.1; 0.1; 
       350; 350; 350; 350;
       10; 0.5
       ] ./ opts.xscale(:);
       [+1; +10e3] ./ opts.uscale(:)
    ];
% Undocumented feature: constraint the raceline on the track centerline
if opts.constrainRaceline % default false
    % x(24) = n
    lbx(24) = 0;
    ubx(24) = 0;
end
% u = [delta__ddot', Tau__t']
lbu = [[-1; -1e2] ./ opts.upscale(:); p0(:)];
ubu = [[+1; +1e2] ./ opts.upscale(:); p0(:)];
% c = [n/rwr, n/rwl, N__fl/mg, N__fr/mg, N__rl/mg, N__rr/mg, Tau__e-Tau__emax]
lbc = [-1e2; 0; opts.Neps; opts.Neps; opts.Neps; opts.Neps; -1e4]./opts.cscale(:);
ubc = [0; +1e2; +10;       +10;       +10;       +10;       +0]./opts.cscale(:);
% b = bcs_fun(xi,xf)
lbb = zeros(nb,1);
ubb = zeros(nb,1);
% repeat bounds for augmented vars
lbw = repmat(lbx(:), [d,1]); ubw = repmat(ubx(:), [d,1]);
lbz = repmat(lbu(:), [d,1]); ubz = repmat(ubu(:), [d,1]);
lbc = repmat(lbc(:), [d,1]); ubc = repmat(ubc(:), [d,1]);

% Mesh
% N = round(opts.numMeshPts / d); % Value set by user is the total number of pts (including collocated pts)
N = opts.numMeshPts; % Value set by user is number of mesh pts (excluding internal collocated pts)
smesh = carfm.common.createMesh(si, sf, N, track.s, track.Omegaz, opts);
tmesh = smesh / opts.sscale;
hmesh = diff(tmesh); 
% get time at collocation points + end time
tcol = tmesh(1:end-1) + hmesh.*tau_root(1:end-1)';
tcol = tcol(:)'; tcol = [tcol, tmesh(end)];

% Guess
if isempty(fieldnames(guess))
    % generate guess for x, u: SSA at opts.speedGuess in straight
    ssres = carfm.ssa(car,[opts.speedGuess; 0; 0],zeros(3,1));
    % workaround: SSA resets the env, need to set it again ...
    % alternative is to implement SS analysis for dynamics here?
    % or allow user to give guess from SSA explicitly? This may be the best
    % solution b/c allows user to select the proper guess speed, 
    % but the user must ALWAYS provide guess 
    carfm.common.setEnvironment(filepath); 
    % Check SSA exit
    if ssres.exitcode<0
        clearvars -except filepath
        carfm.common.setEnvironment(filepath, false);
        error('carfm:unableSolve','Unable to solve the SSA for MLTS guess.')
    end
    % Convert to dyna vars
    [x10, ~, u10] = carfm.dymlts.getDynVars(ssres);
    u10(2,:) = 0; % better to use zero torque as guess?
    % Create x0,u0
    x0 = [ [x10; 0; 0] ./ opts.xscale(:); u10 ./ opts.uscale(:)]; % n,chi=0
    u0 = repmat(zeros(size(u)), [1 size(x0,2)]);
    % repeat guess for augmented vars
    w0 = repmat(x0(:), [d N]);
    z0 = repmat(u0(:), [d N]);
else
    % guess is mltsout (from ggmlts or from this function)
    [x10, ~, u10] = carfm.dymlts.getDynVars(guess.data);
    x0 = [ [x10; guess.n; guess.chi-[guess.data.driftAngle] ] ./ opts.xscale(:); u10 ./ opts.uscale(:)];
    % estimate control using FD
    u0 = (diff(u10')./diff(guess.s'))' ./ opts.upscale(:); u0(:,end+1) = u0(:,end);
    % u0 = repmat(zeros(size(u)), [1 size(x0,2)]);
    t0 = guess.s ./ opts.sscale;
    % fill missing
    x0 = fillmissing(x0,'nearest',2);
    u0 = fillmissing(u0,'nearest',2);
    % calc rates using FD for guess from ggmlts
    if isfield(guess.data, 'xss') % guess.data from ggmlts has 'xss' field, while from dymlts no
        % when from ggmlts we need better estimations of
        % V__z, Omega__x, Omega__y, Omega__z, delta__dot, ft__dot, sa__dot,
        % because these are computed under QSS assumption in ggmlts and may
        % not consistent with dynamics
        % omega0 = casadi.MX.sym('omega0', 3);
        % vw0 = casadi.MX.sym('vw0', 1);
        % estGuessRates = casadi.Function('estGuessRates', {x, xp, omega0, vw0}, {carfm.dymlts.estGuessRates(x,xp,omega0,vw0,aux.car,opts.xscale)});
        % v0 = (diff(x0')./diff(guess.t'))'; v0(:,end+1) = v0(:,end);
        % filter a bit u0,v0 b/c possibly noisy
        u0 = movmean(u0, 25, 2); % movmean on 25 samples hardcoded
        % v0 = movmean(v0, 25, 2); % movmean on 25 samples hardcoded
        % eval guess rates
        % x0 = full(estGuessRates(x0, v0, [guess.omegax; guess.omegay; guess.omegaz], 0*guess.s));
    end
    % interp onto collocated points
    x0 = interp1(t0', x0', tcol', 'linear', 'extrap')';
    u0 = interp1(t0', u0', tcol', 'linear', 'extrap')';
    % add last d points that are not inside the domain
    x0 = [x0, repmat(x0(:,end), [1, d-1])];
    u0 = [u0, repmat(0*u0(:,end), [1, d-1])];
    % get w0,z0
    w0 = reshape(x0, [nx*d N]);
    z0 = reshape(u0, [nu*d N]);
end

% Get Ipopt tolerance options
[ipopt_options, refine_ipopt_options] = carfm.common.getIpoptOptions(opts);

% OCP-NLP formulation
if opts.mex % use Optra
    % Generate Mex
    if ~opts.usePrebuilt
        optra.build(opts.problemName, ...
            ocp_runcost, ocp_bcscost, ocp_dyn, ocp_path, ocp_bcs, ocp_int, ...
            opts.exactHessian || opts.refineSolution ... % generate hessian only if required
        );
    end
    % Check for build only
    if opts.buildOnly
        % Reset and exit
        clearvars -except filepath
        carfm.common.setEnvironment(filepath, false);
        mltsout = struct(); % empty output argument
        return;
    end
    % create problem
    problem.name = opts.problemName;
    problem.number_mesh = N;
    problem.initial_time = ti;
    problem.final_time = tf;
    problem.mesh_fractions = hmesh/sum(hmesh);
    problem.guess.state = w0;
    problem.guess.control = z0;
    problem.guess.parameter = [];
    problem.bounds.state_lower = lbw; 
    problem.bounds.state_upper = ubw;
    problem.bounds.control_lower = lbz; 
    problem.bounds.control_upper = ubz;
    problem.bounds.parameter_lower = []; 
    problem.bounds.parameter_upper = [];
    problem.bounds.path_lower = lbc; 
    problem.bounds.path_upper = ubc;
    problem.bounds.bcs_lower = lbb; 
    problem.bounds.bcs_upper = ubb;
    problem.bounds.int_lower = []; 
    problem.bounds.int_upper = [];
    if opts.debugSolve 
        problem.iteration_callback  = @carfm.dymlts.iterCallbackFree;
    end
    % create global options
    global_options.iteration_print_frequency = opts.printInt;
    global_options.number_threads = opts.numThreads;
    global_options.sb = true; % suppress banner

    % Call to Optra
    sol = optra.solve(problem, 'ipopt', ipopt_options, global_options);
    % Undocumented feature: refine the solution using the exact Hessian and 
    % the default Ipopt tolerances
    if opts.refineSolution
        problem.guess = sol; % set guess from solution
        sol = optra.solve(problem, 'ipopt', refine_ipopt_options, global_options);
    end
    % Get the solution
    wopt = sol.state;
    zopt = sol.control;
    lamwopt = sol.state_multiplier;
    lamzopt = sol.control_multiplier;
    lamfopt = sol.dyn_multiplier;
    lamcopt = sol.path_multiplier;
    % Check derivatives (debugging only)
    % carfm.common.derivativeChecks;
else % use CASADI solver
    % Utility functions
    lamf1 = casadi.MX.sym('lamf', nx*d);
    lamc1 = casadi.MX.sym('lamc', nc*d);
    eval_y = casadi.Function('eval_y', {w1, z1}, {[w1; z1]});
    eval_wz = casadi.Function('eval_wz', {[w1; z1]}, {w1, z1});
    eval_lamfc = casadi.Function('eval_lamfc', {[lamf1; lamc1]}, {lamf1, lamc1});
    eval_y = returntypes('full', eval_y);
    eval_wz = returntypes('full', eval_wz);
    eval_lamfc = returntypes('full', eval_lamfc);
    % NLP guess
    y0 = eval_y(w0, z0); y0 = y0(:);
    % Ipopt options
    nlp_opts.ipopt = ipopt_options;
    % additional ipopt options
    nlp_opts.ipopt.print_level = 5;
    nlp_opts.ipopt.print_timing_statistics = 'yes';
    nlp_opts.ipopt.print_frequency_iter = opts.printInt;
    nlp_opts.ipopt.file_print_level = 5;
    nlp_opts.ipopt.output_file = [opts.problemName '.log'];
    % Transcribe problem
    [y, g, J, lby, uby, lbg, ubg] = carfm.common.casadiTranscribeOCP(tmesh, ...
        ocp_runcost, ocp_bcscost, ocp_dyn, ocp_path, ocp_bcs, ...
        lbw, ubw, lbz, ubz, lbc, ubc, lbb, ubb, ...
        opts.numThreads);
    % Create NLP struct
    prob = struct('f', J, 'x', y, 'g', g);
    % Set iter_callback
    if opts.debugSolve
        casadiIPOPTCallback = carfm.common.CasadiIPOPTCallback('iter_callback', nx*d, nu*d, nc*d, numel(y), numel(g), tmesh, opts.printInt, ...
            @carfm.dymlts.iterCallbackFree);
        nlp_opts.iteration_callback = casadiIPOPTCallback;
    end
    % Call to Ipopt
    solver = casadi.nlpsol('solver', 'ipopt', prob, nlp_opts);
    if opts.debugSolve
        casadiIPOPTCallback.set_solver(solver);
    end
    sol = solver('x0', y0, 'lbx', lby, 'ubx', uby, 'lbg', lbg, 'ubg', ubg);
    % Get the solution
    yopt = sol.x;
    lamyopt = sol.lam_x;
    lamgopt = sol.lam_g;
    [wopt,zopt] = eval_wz(reshape(yopt, [d*(nx+nu), N]));
    [lamwopt,lamzopt] = eval_wz(reshape(lamyopt, [d*(nx+nu), N]));
    [lamfopt, lamcopt] = eval_lamfc(reshape(lamgopt(1:end-nc*d-nb), [d*(nx+nc), N-1]));
    lamcopt = [lamcopt, full(lamgopt((end-nc*d-nb+1):(end-nb)))];
end

% Post process
% Integrals
dlopt = ocp_runcost2(tmesh(1:end-1), wopt(:,1:end-1), zopt(:,1:end-1), wopt(:,2:end), zopt(:,2:end), [], hmesh);
lopt = full(sum(dlopt,2));

% Get OCP multipliers
% apply quadrature weights
Wlamw = reshape( repmat(W(1:end-1)', [nx 1]), [d*nx 1]);
Wlamz = reshape( repmat(W(1:end-1)', [nu 1]), [d*nu 1]);
Wlamf = reshape( repmat(W(1:end-1)', [nx 1]), [d*nx 1]);
Wlamc = reshape( repmat(W(1:end-1)', [nc 1]), [d*nc 1]);
lamwopt = lamwopt ./ Wlamw;
lamzopt = lamzopt ./ Wlamz;
lamfopt = lamfopt ./ Wlamf;
lamcopt = lamcopt ./ Wlamc;
% apply mesh size
lamwopt = lamwopt ./ [hmesh, hmesh(end)]; % repeat last
lamzopt = lamzopt ./ [hmesh, hmesh(end)]; % repeat last
lamfopt = lamfopt ./ hmesh; 
lamcopt = lamcopt ./ [hmesh, hmesh(end)]; % repeat last
% check for cyclic bcs
xBcsTry = rand(nx,1)+1; % test x
uBcsTry = rand(nu,1)+1; % test u
if ~opts.bcsRelax && all( opts.bcsFunc(xBcsTry,uBcsTry,xBcsTry,uBcsTry) == 0 )
    lamcopt(:,1) = lamcopt(:,1) + lamcopt(:,end); % sum b/c cyclic (i.e. same constraint)
    lamcopt(:,end) = lamcopt(:,1); % cyclic
end
% Get states, controls, and multipliers at collocation points
Xopt = reshape(wopt, [nx N*d]); Xopt = Xopt(:,1:end-(d-1));
Uopt = reshape(zopt, [nu N*d]); Uopt = Uopt(:,1:end-(d-1));
Lxopt = reshape(lamwopt, [nx N*d]); Lxopt = Lxopt(:,1:end-(d-1));
Luopt = reshape(lamzopt, [nu N*d]); Luopt = Luopt(:,1:end-(d-1));
Lcopt = reshape(lamcopt, [nc N*d]); Lcopt = Lcopt(:,1:end-(d-1));
Lfopt = reshape(lamfopt, [nx, (N-1)*d]);
% Find xp
fstep = casadi.Function('fstep', {xp,t,x,u}, {f(t, x, xp, u)});
stepsolver = casadi.rootfinder('stepsolver', 'fast_newton', fstep);
stepsolver = returntypes('full', stepsolver);
if contains(opts.scheme, 'lgr') % For LGR use differentiation matrix to compute xp from x
    Xpopt = zeros(nx, size(Xopt,2)-1);
    for j = 1 : d
        Xpopt(:,j:d:end) = D(1,j)*Xopt(:,1:d:end-1)./hmesh;
        for r = 2 : d+1
            Xpopt(:,j:d:end) = Xpopt(:,j:d:end)  + D(r,j)*Xopt(:,r:d:end)./hmesh;
        end
    end
    Xpopt = [Xpopt, Xpopt(:,end)]; % add non collocated point
    % Solve Xpopt for last point (non collocated)
    Xpopt(:,end) = stepsolver(Xpopt(:,end), tcol(:,end), Xopt(:,end), Uopt(:,end));
else
    Xpopt = (diff(Xopt')./diff(tcol'))';
    Xpopt = [Xpopt, Xpopt(:,end)];
    if ~strcmp(opts.scheme, 'euler')
        for k = 1 : numel(tcol)
            Xpopt(:,k) = stepsolver(Xpopt(:,k), tcol(:,k), Xopt(:,k), Uopt(:,k));
        end
    end
end

% Get dynamics data
aux.car = car; % do not use CASADI data
[~, ~, ~, mltsout] = carfm.dymlts.ocpEquations(tcol,Xopt,Xpopt,Uopt(1:end-np,:),aux,opts); % eval ocp equations at optimal solution
mltsout = carfm.dymlts.calcSensitivities(mltsout, Lxopt, Luopt, Lfopt, Lcopt, opts); % dymlts sensitivities

mltsout.laptime = lopt(1)*opts.lscale; % lap time
mltsout.pnlt = lopt(2)*opts.lscale; % control penalty
mltsout.t = mltsout.t / mltsout.t(end) * mltsout.laptime; % make res.t and res.laptime consistent

% Rem path
clearvars -except mltsout filepath
carfm.common.setEnvironment(filepath, false);

end