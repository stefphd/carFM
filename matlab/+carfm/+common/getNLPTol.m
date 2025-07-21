function nlpopt = getNLPTol(opt)
%getNLPTol Map MLTS option to NLP IPOPT options

% Undocumented feature: read 'nlpopt' field in opt for a manual set of IPOPT
% options. Note that these may be overwritten by the next options
if isfield(opt,'nlpopt')
   nlpopt = opt.nlpopt; 
end

% Opt for Optimal EXIT
nlpopt.tol = opt.tol; % tol -> tol
nlpopt.constr_viol_tol = opt.conTol; % conTol -> constr_viol_tol
nlpopt.dual_inf_tol = opt.optTol; % optTol -> dual_inf_tol
nlpopt.compl_inf_tol = opt.complTol; % complTol -> compl_inf_tol
% Opt for Acceptable EXIT
nlpopt.acceptable_tol = opt.tolAccept; % tolAccept -> acceptable_tol
nlpopt.acceptable_constr_viol_tol = opt.conTolAccept; % conTolAccept -> acceptable_constr_viol_tol
nlpopt.acceptable_dual_inf_tol = opt.optTolAccept; % optTolAccept -> acceptable_dual_inf_tol
nlpopt.acceptable_compl_inf_tol = opt.complTolAccept; % complTolAccept -> acceptable_compl_inf_tol
nlpopt.acceptable_iter = opt.iterAccept; % iterAccept -> acceptable_iter
nlpopt.acceptable_obj_change_tol = opt.objChangeAccept; % objChangeAccept -> acceptable_obj_change_tol
% Linear solver
nlpopt.linear_solver = opt.linearSolver; % linearSolver -> linear_solver
end

