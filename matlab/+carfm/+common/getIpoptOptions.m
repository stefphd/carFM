function [ipopt_options, refine_ipopt_options] = getIpoptOptions(options)
% GETIPOPTOPTIONS Maps MLTS option to Ipopt options.

% Ipopt options
ipopt_options = struct();
% Undocumented feature: read 'ipopt_options' field in opt for a manual set
% of Ipopt options. 
% Note that these may be overwritten by the next options
if isfield(options,'ipopt_options')
   ipopt_options = options.ipopt_options; 
end
% Max iter
ipopt_options.max_iter = options.maxIter; % maxIter -> max_iter
% Hessian model
if options.exactHessian
    ipopt_options.hessian_approximation = 'exact';
else
    ipopt_options.hessian_approximation = 'limited-memory';
end
% Opt for Optimal EXIT
ipopt_options.tol = options.tol; % tol -> tol
ipopt_options.constr_viol_tol = options.conTol; % conTol -> constr_viol_tol
ipopt_options.dual_inf_tol = options.optTol; % optTol -> dual_inf_tol
ipopt_options.compl_inf_tol = options.complTol; % complTol -> compl_inf_tol
% Opt for Acceptable EXIT
ipopt_options.acceptable_tol = options.tolAccept; % tolAccept -> acceptable_tol
ipopt_options.acceptable_constr_viol_tol = options.conTolAccept; % conTolAccept -> acceptable_constr_viol_tol
ipopt_options.acceptable_dual_inf_tol = options.optTolAccept; % optTolAccept -> acceptable_dual_inf_tol
ipopt_options.acceptable_compl_inf_tol = options.complTolAccept; % complTolAccept -> acceptable_compl_inf_tol
ipopt_options.acceptable_iter = options.iterAccept; % iterAccept -> acceptable_iter
ipopt_options.acceptable_obj_change_tol = options.objChangeAccept; % objChangeAccept -> acceptable_obj_change_tol
% Linear solver
ipopt_options.linear_solver = options.linearSolver; % linearSolver -> linear_solver

% Ipopt options for solution refinement (available if options.refineSolution = true)
% Undocumented feature: read 'refine_ipopt_options' field in opt for a
% manual set of Ipopt options. 
% Note that these may be overwritten by the next options
refine_ipopt_options = struct();
if isfield(options,'refine_ipopt_options')
   refine_ipopt_options = options.refine_ipopt_options; 
end
if isfield(options,'refineSolution') && options.refineSolution
    refine_ipopt_options.max_iter = options.refineMaxIter;
    refine_ipopt_options.hessian_approximation = 'exact'; % always exact hessian used
    % options for warm start
    refine_ipopt_options.mu_init = options.refineInitBarrier; 
    refine_ipopt_options.warm_start_init_point = 'yes';
end

end