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
    default_opts.xscale = [1 1 1 1 1 1 1 1 1 ... % phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda
                           1 1 1 1 ... % kappa__fl, kappa__fr, kappa__rl, kappa__rr
                           1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 ... % X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
                           1e3 1e3 1e3 1e3 ... % F__fl, F__fr, F__rl, F__rr
                           1e3 ... % Tau__t = Tau__fl+Tau__fr+Tau__rl+Tau__rr
                           1e2 1 1e1 ... % V, yaw__rate, a__t
                           ]; % x vector scale
    default_opts.xsign = [-1 +1 +1 -1 +1 +1 +1 +1 -1 ... % phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda
                          +1 +1 +1 +1 ... % kappa__fl, kappa__fr, kappa__rl, kappa__rr
                          +1 +1 +1 +1 -1 -1 -1 -1 +1 +1 +1 +1 ... % X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr, 
                          +1 +1 +1 +1 ... % F__fl, F__fr, F__rl, F__rr
                          +1 ... % Tau__t = Tau__fl+Tau__fr+Tau__rl+Tau__rr
                          +1 -1 +1 ... % V, yaw__rate, a__t
                          ]; % x signs for symmetrization
    default_opts.rscale = [1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3... % tyres
                           1e3 1e3 1e3 1e3 ... % suspensions
                           1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 1e3 ... % SS equations
                           1e2 1e1 1e3 ... % active long inputs
                           1 1 1 ... active lat inputs
                           ]; % residual vector scale
    % Solver options
    default_opts.rtol = 1e-9; % residual tolerance for steady-state solved
    default_opts.mex = false; % compile mex
    default_opts.usePrebuilt = false; % use pre-built mex
    default_opts.buildOnly = false; % only build mex, without run sim
    % Model options
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
    default_opts.Neps = 0.02; % zero vertical load (in units of m*g)
    default_opts.extForce  = [0 0 0]; % x,y,z external force applied to the motorcycle in ref. point P (N)
    default_opts.extTorque = [0 0 0]; % x,y,z external torque applied to the motorcycle
    % SSA options
    default_opts.ssaMexName = 'ssaSolver'; % mex filename for ssa solver
    default_opts.ssActiveLongInputs = [true true false]; % active longitudinal inputs: V, a__t Tau__fl+Tau__fr+Tau__rl+Tau__rr
    default_opts.ssActiveLatInputs = [true false false]; % active lateral inputs: yaw__rate, phi, delta, tau, lambda
    default_opts.ssSolver = 'fast_newton'; % steady-state solver: 'fast_newton' or 'newton'
    default_opts.useLastSSA = false; % in ssa use the last (not failed) SSA solution for guess when running multiple SSAs
    default_opts.gz_g = 1; % ratio gz/g    
    % GG options
    default_opts.ggMexName = 'ggSolver'; % mex filename for gg solver
    default_opts.fillFailed = true; % fill failed GG points with spline interp
    default_opts.kappaLim = 0.3; % maximum long slip (/)
    default_opts.wdelta = 25; % steer weighting factor (/)
    default_opts.wlambda = 25; % drift weighting factor (/)
    default_opts.wkappa = 25; % long slip weighting factor (/)
    default_opts.numGGpts = 181; % number of gg points from alpha=-pi/2 to alpha=+pi/2
    default_opts.isSymGG = true; % assume symmetric GG, i.e. avoid computation of left-side
    default_opts.GGshift = @(V) 0; % function handle (function of V) for the GG vertical shift (expressed in g)
    default_opts.algorithmGGopt = 'interior-point'; % algorithm employed for GG optimization ('interior-point' or 'sqp')
    % [...]
    % override default options
    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        default_opts.(opts_fields{k}) = opts.(opts_fields{k});
    end
    opts = default_opts;
end