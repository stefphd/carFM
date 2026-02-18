function mltsout = ggmltsfixed(gg, traj, opts, guess)
%GGMLTSFIXED - See help/ggmltsfixed.m for the help of this function.

% Check args
arguments
    gg (1,1) struct {carfm.ggmlts.checkGGStruct(gg)}
    traj (1,1) struct {carfm.common.checkTrajStruct(traj)}
    opts (1,1) struct {carfm.ggmlts.checkOptStruct(opts)} = struct()
    guess (1,1) struct {carfm.common.checkGuessStruct(guess, {'zeta','V','at','jt'})} = struct()
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
opts = carfm.ggmlts.getDefaultOptions(opts);

% Get undocumented options and override with user-defined options
opts = carfm.ggmlts.getUndocOptions(opts);

% Find GG grid bounds
% grid = {V,g}
[grid_min,grid_max,flag,errid,errmsg] = carfm.ggmlts.getGGBoundGrid(gg);
if flag
    % Reset and exit
    clearvars -except filepath errmsg errid
    carfm.common.setEnvironment(filepath, false);
    error(errid, '%s', errmsg);
end
ming = grid_min(2); maxg = grid_max(2);
if opts.speedRange(1) <= grid_min(1)
    minV = grid_min(1);
else
    minV = opts.speedRange(1);
end
if opts.speedRange(end) >= grid_max(1)
    maxV = grid_max(1);
else
    maxV = opts.speedRange(end);
end
% check minV<maxV needed
if minV > maxV
    % Reset and exit
    clearvars -except filepath
    carfm.common.setEnvironment(filepath, false);
    % Give error if sth went wrong
    error('carfm:invalidValue', 'Minimum/maximum speed bounds are inconsistent');
end

% Create aux
traj = carfm.common.checkTrajStruct(traj); % eventually use zeta instead of elap_dist
aux.track = carfm.common.interpTraj(traj, 'casadi', opts.trackinterpMethod, opts.minDecLen);
aux.rho = carfm.ggmlts.interpGG(gg, 'internal', opts.gginterpMethod, 'rho');
aux.shift = {gg.shift};
aux.g0 = opts.g;

% Add fake track data to deal with trajectory in carfm.ggmlts.ocpEquations
aux.track.Omegaz = aux.track.Gammaz;
aux.track.theta = aux.track.psi;
aux.track.xl = aux.track.x;
aux.track.yl = aux.track.y;
aux.track.xr = aux.track.x;
aux.track.yr = aux.track.y;
aux.track.rwl = @(zeta) 0;
aux.track.rwr = @(zeta) 0;
if aux.track.isTrack3D
    aux.track.mu = aux.track.sigma;
    aux.track.phi = aux.track.beta;
    aux.track.Omegax = aux.track.Gammax;
    aux.track.Omegay = aux.track.Gammay;
    aux.track.zl = aux.track.z;
    aux.track.zr = aux.track.z;
end

% Find zetai and zetaf
if opts.sRange(1) <= traj.zeta(1)
    zetai = traj.zeta(1);
else
    zetai = opts.sRange(1);
end
if opts.sRange(end) >= traj.zeta(end)
    zetaf = traj.zeta(end);
else
    zetaf = opts.sRange(end);
end

% Computation domain
ti = zetai / opts.sscale;
tf = zetaf / opts.sscale;

% Create vars
nx = 2; nu = 1; % state and control dimensions
x = casadi.MX.sym('x', nx); %states
u = casadi.MX.sym('u', nu); %control
t = casadi.MX.sym('t'); %independent variable
x0 = casadi.MX.sym('x0', nx); % initial state
u0 = casadi.MX.sym('u0', nu); % initial control
xn = casadi.MX.sym('xn', nx); % final state
un = casadi.MX.sym('un', nu); % final control

% Model equations
% xdot: state derivative
% c: path constraint
% l: Lagrange term
s = t*opts.sscale;
V = x(1)*opts.xscale(1);
at = x(2)*opts.xscale(4);
an = aux.track.Gammaz(s)*V^2;
if opts.useLatJerk
    if ~aux.track.hasGammazp 
       % Reset and exit
       clearvars -except filepath err
       carfm.common.setEnvironment(filepath, false);
       error('carfm:notFound', 'Gammazp is required when useLatJerk=true.');
    end
    jn = aux.track.Gammazp(s)*V^3+aux.track.Gammaz(s)*2*V*at;
else
    jn = 0; % not included in penalty 
end
xa = [x(1); 0; 0; x(2); an/opts.xscale(5)]; % augmented state
ua = [u(1); jn/opts.uscale(2)]; % augmented control
getAugVars = casadi.Function('getAugState', {t,x,u},{xa,ua});
[xadot, ca, l] = carfm.ggmlts.ocpEquations(t,xa,ua,aux,opts);
xdot = [xadot(1); xadot(4)];
c = ca(3:end); % exclude road constraints
c = [jn/opts.uscale(2); c]; % add jerk constraint (possibly 0) 
nc = numel(c);
% bcs
try
    b = opts.bcsFunc(x0 .* opts.xscale([1 4])', ...
                     xn .* opts.xscale([1 4])');
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
f = casadi.Function('f', {t, x, u}, {xdot, l});
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
        [fj, lj] = f(tj, xj, uj);
        dw = [dw; (fj*h-xpj)];
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
    switch opts.scheme
        case 'euler' % Euler integration
            [f1, l1] = f(t,x1,u1);
            c1 = g(t,x1,u1);
            dw = x1 + h*f1 - x2;
            dl = h*l1;
            c = c1;
        case 'midpoint'
            [f1, l1] = f(t+.5*h,0.5*(x1+x2),u1);
            c1 = g(t,x1,u1);
            dw = x1 + h*f1 - x2;
            dl = h*l1;
            c = c1;
        % case 'trapz' % Trapezoidal rule (fallback)
        otherwise % Fallback to 'trapz'
            [f1, l1] = f(t,x1,u1);
            [f2, l2] = f(t+h,x2,u1);
            c1 = g(t,x1,u1);
            dw = x1 + 0.5*h*(f1+f2) - x2;
            dl = 0.5*h*(l1+l2);
            c = c1;
    end
end
% apply scale to dw
dw = dw ./ repmat(opts.fscale([1 4])', [d 1]);

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
p = casadi.MX.sym('p',0);
ocp_runcost = casadi.Function('run_cost', {t, w1, z1, w2, z2, p, h}, {sum(dl)});
ocp_runcost2 = casadi.Function('run_cost2', {t, w1, z1, w2, z2, p, h}, {dl}); % separates laptime and penalty
ocp_bcscost = casadi.Function('bcs_cost', {w0, z0, wn, zn, p}, {m});
ocp_dyn = casadi.Function('dyn_constr', {t, w1, z1, w2, z2, p, h}, {dw});
ocp_path = casadi.Function('path_constr', {t, w1, z1, p, h}, {c});
ocp_bcs = casadi.Function('bcs_constr', {w0, z0, wn, zn, p}, {b});
ocp_int = casadi.Function('int_constr', {t, w1, z1, w2, z2, p, h}, {[]});

% Bounds
lbx = [minV -4*opts.g] ./ opts.xscale([1 4]); % lower state bound
ubx = [maxV +4*opts.g] ./ opts.xscale([1 4]); % upper state bound
lbu = opts.minLongJerk*opts.g ./ opts.uscale(1);
ubu = opts.maxLongJerk*opts.g ./ opts.uscale(1);
lbc = [ -opts.maxLatJerk*opts.g/opts.uscale(2), [-1, ming] ./ opts.cscale(end-1 : end) ];
ubc = [ +opts.maxLatJerk*opts.g/opts.uscale(2), [1,  maxg] ./ opts.cscale(end-1 : end) ];
lbb = zeros(nb,1);
ubb = zeros(nb,1);
% fix lbc,ubc for geq/g constraint in the case of 2D g-g is provided
if numel(gg(1).g)<2 % 2D g-g
    % disable limits - these should be large enough (geq/g typically within [0,2])
    % these are also consistent with the one harcoded in interpGG
    lbc(end) = 0;
    ubc(end) = +2;
end
% repeat bounds for augmented vars
lbw = repmat(lbx(:), [d,1]); ubw = repmat(ubx(:), [d,1]);
lbz = repmat(lbu(:), [d,1]); ubz = repmat(ubu(:), [d,1]);
lbc = repmat(lbc(:), [d,1]); ubc = repmat(ubc(:), [d,1]);

% Mesh
% N = round(opts.numMeshPts / d); % Value set by user is the total number of pts (including collocated pts)
N = opts.numMeshPts; % Value set by user is number of mesh pts (excluding internal collocated pts)
smesh = carfm.common.createMesh(zetai, zetaf, N, traj.zeta, traj.Gammaz, opts);
tmesh = smesh / opts.sscale;
hmesh = diff(tmesh); 
% get time at collocation points + end time
tcol = tmesh(1:end-1) + hmesh.*tau_root(1:end-1)';
tcol = tcol(:)'; tcol = [tcol, tmesh(end)];

% Guess
if isempty(fieldnames(guess))
    % generate guess for x, u
    x0 = [gg(1).V(1) 0.01] ./ opts.xscale([1 4]); % guess for states
    u0 = 0 ./ opts.uscale(1); % guess for control
    % repeat guess for augmented vars
    w0 = repmat(x0(:), [d N]);
    z0 = repmat(u0(:), [d N]);
else
    % Extract vars from guess
    x0 = [guess.V; guess.at] ./ opts.xscale([1 4])';
    u0 = guess.jt ./ opts.uscale(1);
    t0 = guess.zeta ./ opts.sscale;
    % fill missing
    x0 = fillmissing(x0,'nearest',2);
    u0 = fillmissing(u0,'nearest',2);
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
        problem.iteration_callback  = @carfm.ggmlts.iterCallbackFixed;
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
            @carfm.ggmlts.iterCallbackFixed);
        nlp_opts.iteration_callback = casadiIPOPTCallback;
    end
    % Call to Ipopt
    solver = casadi.nlpsol('solver', 'ipopt', prob, nlp_opts);
    if opts.debugSolve
        casadiIPOPTCallback.set_solver(solver);
    end
    sol = solver('x0', y0, 'lbx', lby, 'ubx', uby, 'lbg', lbg, 'ubg', ubg);
    % Undocumented feature: refine the solution using the exact Hessian and 
    % the default Ipopt tolerances
    if opts.refineSolution
        clear solver % needed to destruct the solver to close the .log file
        % Ipopt options
        nlp_opts.ipopt = refine_ipopt_options;
        % additional ipopt options
        nlp_opts.ipopt.print_level = 5;
        nlp_opts.ipopt.print_timing_statistics = 'yes';
        nlp_opts.ipopt.print_frequency_iter = opts.printInt;
        nlp_opts.ipopt.file_print_level = 5;
        nlp_opts.ipopt.output_file = [opts.problemName '.log'];
        solver = casadi.nlpsol('solver', 'ipopt', prob, nlp_opts);
        if opts.debugSolve
            casadiIPOPTCallback.set_solver(solver);
        end
        sol = solver('x0', sol.x, 'lam_x0', sol.lam_x, 'lam_g0', sol.lam_g, 'lbx', lby, 'ubx', uby, 'lbg', lbg, 'ubg', ubg);
    end
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
% lamfopt = lamfopt ./ hmesh; % NOT NECESSARY b/c diff constraint is multiplied by h
lamcopt = lamcopt ./ [hmesh, hmesh(end)]; % repeat last
% check for cyclic bcs
xBcsTry = rand(nx,1)+1; % test x
if ~opts.bcsRelax && all( opts.bcsFunc(xBcsTry,xBcsTry) == 0 )
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
% Get data
[Xaopt, Uaopt] = getAugVars(tcol, Xopt, Uopt);
[~, ~, ~, mltsout] = carfm.ggmlts.ocpEquations(tcol,full(Xaopt),full(Uaopt),aux,opts); % eval ocp equations at optimal solution
mltsout = carfm.ggmlts.calcSensitivities(mltsout, Lxopt, Luopt, Lfopt, Lcopt, opts); % ggmlts sensitivities
mltsout.laptime = lopt(1)*opts.lscale; % lap time
mltsout.pnlt = lopt(2)*opts.lscale; % control penalty
mltsout.t = mltsout.t / mltsout.t(end) * mltsout.laptime; % make res.t and res.laptime consistent
% Remove fields that are not used for fixed MLTS
fields2rem = {'n','chi','ndot','chidot','x','y','z','psi','sigma','beta','Gammax','Gammay','Gammaz'};
mltsout = rmfield(mltsout, fields2rem);

% Rem path
clearvars -except mltsout filepath
carfm.common.setEnvironment(filepath, false);

end