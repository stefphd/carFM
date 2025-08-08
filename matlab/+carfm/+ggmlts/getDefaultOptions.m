function opts = getDefaultOptions(opts)
    % GETDEFAULTOPTIONS Get default options.
    %
    % INPUT:
    % opts: user-defined option structure
    %
    % OUTPUT:
    % opts: structure containing -user-defined options plus default (not provided by user) options

    % check arguments
    arguments
        opts struct = struct(); %init to empty
    end
    % Default options
    % Scaling factors
    default_opts.g = 9.806; % gravity (m/s2)
    default_opts.sscale = 100; % s scale
    default_opts.xscale = [ 70, 6, 1, ... % V, n, chi
                            9.806, 9.806 ... % at, an
                            ]; % x vector scale
    default_opts.uscale = [ 25, 25 ... % jt, jn
                            ]; % u vector scale
    default_opts.lscale = 1; % l scale
    default_opts.fscale = [ 1, 1, 1, ... % V', n', chi'
                            1, 1, ... % at', an'
                            ]; % f scale
    default_opts.cscale = [ 1, 1, ... % road
                            1, ... % g-g map
                            1 ... % geq/g
                            ]; % c scale
    % Penalty weights and controls
    default_opts.wt = 5e-7; %t-control weight
    default_opts.wn = 5e-6; %n-control weight
    default_opts.maxLongJerk = 8; % max longitudinal jerk (g/s)
    default_opts.minLongJerk = -8; % min longitudinal jerk (g/s)
    default_opts.maxLatJerk = 8; % max lateral jerk (g/s)
    default_opts.useLatJerk = false; % include lateral jerk penalty and bounds in ggmltsfixed (Gammazp required)
    % Solving options
    default_opts.problemName = 'ggmlts'; % Name of the problem
    default_opts.sRange = [-inf, inf]; % Initial and final travelled distance
    default_opts.bcsFunc = @(xi, xf) xf-xi; % function handle to define the boundary conditions (x=[V,n,chi,ax,ay] for free, x=[V,ax] for fixed)
    default_opts.numMeshPts = 2000; % number of mesh points
    default_opts.meshStrategy = 'adaptive'; % mesh strategy: 'equally-spaced', 'adaptive', 'manual'
    default_opts.meshRatio = 3; % max/min mesh size for adaptive mesh strategy
    default_opts.meshMinSecLen = 50; % min len of mesh sector for adaptive mesh strategy
    default_opts.meshTransLen = 50; % transient len for adaptive mesh strategy
    default_opts.meshThFactor = 0.25; % threshold factor for adaptive mesh strategy
    default_opts.meshFractions = 1; % mesh fractions for manual mesh strategy
    default_opts.scheme = 'trapz'; % integration scheme ('euler','trapz','midpoint','lgr<N>')
    default_opts.gginterpMethod = 'bspline'; % G-G interpolation method
    default_opts.trackinterpMethod = 'linear'; % track & traj interpolation method
    default_opts.maxIter = 2000; % NLP max iter (3000 default used in IPOPT)
    default_opts.exactHessian = false; % use exact NLP Hessian (instead of 'limited-memory' option in IPOPT)
    default_opts.printInt = 5; % Print iteration interval
    default_opts.mex = false; % compile mex
    default_opts.usePrebuilt = false; % use pre-built mex
    default_opts.buildOnly = false; % only build mex, without run sim
    default_opts.debugSolve = false; % plot control figure during solving
    default_opts.numThreads = 1; % number of threads
    default_opts.linearSolver = 'mumps'; % Linear solver in NLP solver
    % Solver tolerances
    default_opts.tol = 1e-8; % NLP convergence tol (tol=1e-8 default in IPOPT)
    default_opts.conTol = 1e-7; % NLP constraint tol (constr_viol_tol=1e-4 default in IPOPT)
    default_opts.optTol = 1e-4; % NLP optimality tol (dual_inf_tol=1 default in IPOPT)
    default_opts.complTol = 1e-4; % NLP complementarity tol (compl_inf_tol=1e-4 default in IPOPT)   
    default_opts.iterAccept = 5; % NLP number of acceptable iterates (acceptable_iter=15 default in IPOPT)
    default_opts.tolAccept = 1e-4; % NLP convergence tol for acceptable (acceptable_tol=1e-6 default in IPOPT)
    default_opts.conTolAccept = 1e-7; % NLP constraint tol for acceptable (acceptable_constr_viol_tol=1e-4 default in IPOPT)
    default_opts.optTolAccept = 1e-4; % NLP optimality tol for acceptable (acceptable_dual_inf_tol=1e10 default in IPOPT)
    default_opts.complTolAccept = 1e-4; % NLP complementarity tol for acceptable (acceptable_compl_inf_tol=1e-2 default in IPOPT)
    default_opts.objChangeAccept = 5e-6; % NLP objective change for acceptable (acceptable_obj_change_tol=1e20 default in IPOPT)
    % [...]
    % override default options
    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        default_opts.(opts_fields{k}) = opts.(opts_fields{k});
    end
    opts = default_opts;
end