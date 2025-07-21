function tyreMetric = tyreUsageMetric(tyreData,tyreRes)
%tyreUsageMetric - See help/tyreUsageMetric.m for the help of this function.

% No argument checking performed currently!

% Imports
filepath = fileparts(mfilename('fullpath')); % path of current file
[flag, errid, errmsg] = carfm.common.setEnvironment(filepath); % check tools and set path
if flag
    % Give error if sth went wrong
    error(errid, '%s', errmsg);
end

% CasADi vars
N = casadi.MX.sym('N');
Vs = casadi.MX.sym('Vs');
Vn = casadi.MX.sym('Vn');
Vr = casadi.MX.sym('Vr');
Omega = casadi.MX.sym('Omega');
gamma = casadi.MX.sym('gamma');
phit = casadi.MX.sym('phit'); % NOT IMPLEMENTED
R = casadi.MX.sym('R');
args = {N,Vs,Vn,Vr,Omega,gamma,phit};
% Eval tyre model
[Fx,Fy,~,~,~,kappa,alpha] = tyreData.Forces(tyreData, args{1:6}, 0);
% % Using Jacobian J = d(Fx,Fy)/d(kappa,alpha)
% x = [Vn;Vr;Omega];
% y = [kappa;alpha];
% F = [Fx;Fy];
% dF_dx = jacobian(F,x);
% dy_dx = jacobian(y,x);
% J = dF_dx*pinv(dy_dx);
% detJ = J(1,1)*J(2,2) - J(1,2)*J(2,1); % determinant
% Using explicit formula from Maple
dFx_dVn = gradient(Fx,Vn); dFx_dVr = gradient(Fx,Vr); dFx_dW  = gradient(Fx,Omega);
dFy_dVn = gradient(Fy,Vn); dFy_dVr = gradient(Fy,Vr); dFy_dW  = gradient(Fy,Omega);
dk_dVn = gradient(kappa,Vn); dk_dVr = gradient(kappa,Vr); dk_dW  = gradient(kappa,Omega); 
da_dVn = gradient(alpha,Vn); da_dVr = gradient(alpha,Vr); da_dW  = gradient(alpha,Omega);
detJ = (((dFy_dW * dFx_dVn - dFy_dVn * dFx_dW) * dk_dVn - ...
        dk_dVr * (-dFy_dW * dFx_dVr + dFy_dVr * dFx_dW)) * da_dW + ...
        ((-dFy_dW * dFx_dVn + dFy_dVn * dFx_dW) * da_dVn + ...
        da_dVr * (-dFy_dW * dFx_dVr + dFy_dVr * dFx_dW)) * dk_dW + ...
        (da_dVn * dk_dVr - da_dVr * dk_dVn) * (-dFx_dVn * dFy_dVr + dFy_dVn * dFx_dVr)); % / ...
        % ((dk_dVn ^ 2 + dk_dVr ^ 2) * da_dW ^ 2 - ...
        % 2 * dk_dW * (dk_dVn * da_dVn + dk_dVr * da_dVr) * da_dW + ...
        % (da_dVn ^ 2 + da_dVr ^ 2) * dk_dW ^ 2 + ...
        % (da_dVn * dk_dVr - da_dVr * dk_dVn) ^ 2); % rem den to avoid division by 0: should we keep the denominator?
% metric
detJ0 = casadi.substitute(detJ,[Vn, Vr, Omega],[0*Vs, Vs, Vs/R]); % detJ at zero slip
Delta = casadi.Function('metric',{[N,Vs,Vn,Vr,Omega,gamma,phit]', R}, { detJ / detJ0 });
% Calc usage metric
args0 = [tyreRes.tyreModelArgs];
args0 = reshape([args0{:}], [numel(tyreRes(1).tyreModelArgs) numel(tyreRes)]);
radius = [tyreRes.radius];
tyreMetric = full(Delta(args0,radius));

% Rem path
clearvars -except tyreMetric filepath
carfm.common.setEnvironment(filepath, false);

end
