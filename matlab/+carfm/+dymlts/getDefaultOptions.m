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
    % Model options
    default_opts.rtol = 1e-3; % residual tolerance for dynamics solved
    default_opts.iTyreSide = -1; % tyre side: left (-1) or right (+1)
    default_opts.iTyreType = [1 1 1]; % tyre type (axis z, y, x): practical model (1) or ideal model (0)
    default_opts.iSuspensionSide = +1; % suspension side: left (-1) or right (+1)
    default_opts.iSuspensionType = 1; % suspension type: flexible (1) or rigid (0)
    default_opts.iEngineBrake = 1; % engine braking: include (1) or exclude (0)
    default_opts.iRotaryInertia = 1; % inertia of rotating bodies: include (1) or exclude (0)
    default_opts.iAeroComps = [1 1 1 1 1 1]; % aero components (drag, side, lift, roll, pitch, yaw): include (1) or eaclude (0)
    default_opts.suspTrimRigid = [0 0 0 0.5]; % trim for rigid suspension: phi (>0 clockwise), mu (>0 when front lifts), z (>0 when bounce), and front roll balance
    default_opts.tyreDeformationRigid = [0 0 0 0]; % fl, fr, rl, rr vertical deformation for rigid tyres
    default_opts.gearSharpness = 5; % sharpness factor for gear shift
    default_opts.Taueps = 10; % torque epsilon for regularization (Nm)
    default_opts.Neps = eps; % zero vertical load (in units of m*g)
    default_opts.extForce  = [0 0 0]; % x,y,z external force applied to the car in ref. point P (N)
    default_opts.extTorque = [0 0 0]; % x,y,z external torque applied to the car
    default_opts.sensitivityPar = {}; % parameters for sensitivities
    % Scaling factors
    default_opts.sscale = 1e1; % s scale
    default_opts.xscale = [ 1 1 1 1 ... % phi, mu, z, delta, 
                            1 1 1 1 ... % z__fl, z__fr, z__rl, z__rr
                            1e2 1 1 ... % V__P, lambda__P, V__z
                            1 1 1 1 1 ... % delta__dot, z__fldot, z__frdot, z__rldot, z__rrdot
                            1 1e1 1e1 ... % Omega__z, Omega__x, Omega__y
                            1e2 1e2 1e2 1e2 ... % omega__fl, omega__fr, omega__rl, omega__rr
                            ... % 1 1 1 1 ... % alpha__fl, alpha__fr, alpha__rl, alpha__rr
                            5 1 ... % n, chi
                            ]; % x vector scale 
    default_opts.uscale = [1 1e3... % delta__ddot, Tau__t
                            ]; % u vector scale
    default_opts.upscale = [1 1e3 ... % delta__ddot', Tau__t'
                            ]; % u'=du/ds vector scale
    default_opts.rscale = [1e-1 1e-1 1e-1 1e-1 1e-1 1e-1 1e-1 1e-1 1e-1 ... % velocity eqns
                            1e3 1e3 1e3 1e3 1e3 1e3 ... % newton and euler eqns
                            1e3 1e3 1e3 1e3 ... % suspension eqns
                            1e3 1e3 1e3 1e3 ... % spin eqns
                            ... % 1e0 1e0 1e0 1e0 ... % tyre relax eqns
                            ]; % dynamical eqns vector scale
    default_opts.fscale = [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e2, 1e2, 1e2, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e1, 1e1, 1e1, 1e1, ... 1e1, 1e1, 1e1, 1e1, ... % dynamical
                           1e1, 1e1, ... % n', chi'
                           1e1, 1e1 ... % control integration chains
                           ]; % f scale
    default_opts.cscale = [ 1e1 1e1 ... % road 
                            1e1 1e1 1e1 1e1 ... % positive normal loads
                            1e3 % engine
                            ]; % c scale
    default_opts.lscale = 1; % l scale
    % Penalty weights
    default_opts.wdeltaddot = 1e-3; % delta__ddot weight
    default_opts.wOmegaxdot = 5e-4; % Omegax__dot weight
    default_opts.wOmegaydot = 5e-4; % Omegay__dot weight
    default_opts.wOmegazdot = 1e-3; % Omegaz__dot weight
    default_opts.wuDelta = 1e-3; % delta-control weight
    default_opts.wuTaut = 1e-4; % Tau__t-control weight
    % Solving options
    default_opts.problemName = 'dymlts'; % Name of the problem
    default_opts.sRange = [-inf, inf]; % Initial and final travelled distance
    default_opts.speedGuess = 60; % speed for default guess (m/s)
    default_opts.minDecLen = 0; % Minimum decimation length for track/trajectory data (0 = no downsampling)
    default_opts.bcsFunc = @(xi, ui, xf, uf) [xf-xi; uf-ui]; % function handle to define the boundary conditions
    default_opts.numMeshPts = 3000; % number of mesh points
    default_opts.meshStrategy = 'adaptive'; % mesh strategy: 'equally-spaced', 'adaptive', 'manual'
    default_opts.meshRatio = 3; % max/min mesh size for adaptive mesh strategy
    default_opts.meshMinSecLen = 50; % min len of mesh sector for adaptive mesh strategy
    default_opts.meshTransLen = 50; % transient len for adaptive mesh strategy
    default_opts.meshThFactor = 0.25; % threshold factor for adaptive mesh strategy
    default_opts.meshFractions = 1; % mesh fractions for manual mesh strategy
    default_opts.scheme = 'euler'; % integration scheme ('euler','trapz','midpoint','lgr<N>')
    default_opts.trackinterpMethod = 'linear'; % track & traj interpolation method
    default_opts.exactHessian = false; % use exact NLP Hessian (instead of 'limited-memory' option in IPOPT)
    default_opts.maxIter = 5000; % NLP max iter (3000 default used in IPOPT)
    default_opts.printInt = 1; % Print iteration interval
    default_opts.mex = false; % compile mex
    default_opts.usePrebuilt = false; % use pre-built mex
    default_opts.buildOnly = false; % only build mex, without run sim
    default_opts.debugSolve = false; % plot control figure during solving
    default_opts.numThreads = 4; % number of threads
    default_opts.linearSolver = 'mumps'; % Linear solver in NLP solver
    % Solver tolerances
    default_opts.tol = 1e-5; % NLP convergence tol (tol=1e-8 default in IPOPT)
    default_opts.conTol = 1e-7; % NLP constraint tol (constr_viol_tol=1e-4 default in IPOPT)
    default_opts.optTol = 1e-4; % NLP optimality tol (dual_inf_tol=1 default in IPOPT)
    default_opts.complTol = 1e-4; % NLP complementarity tol (compl_inf_tol=1e-4 default in IPOPT)
    default_opts.iterAccept = 2; % NLP number of acceptable iterates (acceptable_iter=15 default in IPOPT)
    default_opts.tolAccept = 1e-3; % NLP convergence tol for acceptable (acceptable_tol=1e-6 default in IPOPT)
    default_opts.conTolAccept = 1e-7; % NLP constraint tol for acceptable (acceptable_constr_viol_tol=1e-4 default in IPOPT)
    default_opts.optTolAccept = 1e-3; % NLP optimality tol for acceptable (acceptable_dual_inf_tol=1e10 default in IPOPT)
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