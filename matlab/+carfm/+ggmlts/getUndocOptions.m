function opts = getUndocOptions(opts)
    % GETUNDOCOPTIONS Gets some undocumented options which are hidden to the 
    % end-user, either because only for internal use, or because they are 
    % associated with undocumented features not fully tested yet or that 
    % we don't want to make them visible.
    %
    % INPUT:
    % opts: user-defined option structure
    %
    % OUTPUT:
    % opts: structure containing -user-defined options plus undoc options

    % Undocumented features
    % Relax boundary conditions using undocumented  option 'opts.bcsRelax' 
    % (default false): BCs are removed and added into the OCP cost as a 
    % Mayer term. This may be usefull when starting from  unfeasible BCs, 
    % but has not been extensively tested yet.
    default_opts.bcsRelax = false; % Relax the boundary conditions (add them as a penalty)
    % Refine the solution using the exact Hessian and the default Ipopt
    % tolerances
    default_opts.refineSolution = false; % Refine the solution
    default_opts.refineMaxIter = 500; % Max number of iterations for solution refinement
    default_opts.refineInitBarrier = 1e-8; % Initial barrier parameter of IPOPT for solution refinement
    % Constraint the raceline on the track centerline (default false)
    % The lower and upper bounds of the lateral position n are set to 0, so
    % that the centerline is used as the raceline. Ipopt recognizes it as a
    % fixed variable and removes from the problem. This is usefull to
    % perform a GG fixed-trajectory MLTS with the GG free-trajectory MLTS, 
    % however the differential equation system become DAE and the problem 
    % is somehow ill-posed.
    default_opts.constrainRaceline = false;
    % Hidden options
    % None
    % override default options
    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        default_opts.(opts_fields{k}) = opts.(opts_fields{k});
    end
    opts = default_opts;
end

