function opts = getUndocOptions(opts)
%GETUNDOCOPTIONS This function is to get some undocumented options which
%are hidden to the end-user, either because only for internal use, or
%because they are associated with undocumented features not fully tested yet 
%or that we don't want to make them visible.
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
    % Refine the solution if mex=true using undocumented options
    % opts.refineSolution (default false), opts.refineSolver (default
    % 'worhp'), and opts.refineMaxIter (default 500).
    default_opts.refineSolution = false; % Refine the solution using another NLP solver (only if mex=true)
    default_opts.refineSolver = 'worhp'; % NLP solver used for solution refinement
    default_opts.refineMaxIter = 500; % NLP max number of iterations for solution refinement
    % Hidden options
    % None
    %[...] 
    % override default options
    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        default_opts.(opts_fields{k}) = opts.(opts_fields{k});
    end
    opts = default_opts;
end

