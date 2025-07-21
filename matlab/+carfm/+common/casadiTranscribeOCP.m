function [z, g, J, lbz, ubz, lbg, ubg] = casadiTranscribeOCP(t, ...
        ocp_runcost, ocp_bcscost, ocp_dyn, ocp_path, ocp_bcs, ...
        lbx, ubx, lbu, ubu, lbc, ubc, lbb, ubb, num_threads)
%CASADITRANSCRIBEOCP Transcribe ggmlts OCP using casadi.

import casadi.*

N = numel(t); % num of mesh pts

% Map functions
ocp_runcost_map = ocp_runcost.map(N-1, 'thread', num_threads);
ocp_dyn_map = ocp_dyn.map(N-1, 'thread', num_threads);
ocp_path_map = ocp_path.map(N, 'thread', num_threads);

% Vars
X = cell(1,N); % state array [X0,X1,X2,...,XN]
U = cell(1,N); % control array [U0,U1,U2,...,UN]
Z = cell(1,2*N); % NLP vars [X0,U0,X1,U1,...,XN,UN]
for k = 0 : N-1
    X{k+1} = casadi.MX.sym(['X' num2str(k)],numel(lbx));
    U{k+1} = casadi.MX.sym(['U' num2str(k)],numel(lbu));
    Z{1+2*k} = X{k+1}; 
    Z{2+2*k} = U{k+1}; 
end
X = horzcat(X{:});
U = horzcat(U{:});
z = vertcat(Z{:});

% Eval
dl = ocp_runcost_map(t(1:end-1), X(:,1:end-1), U(:,1:end-1), X(:,2:end), U(:,2:end), [], diff(t));
m = ocp_bcscost(X(:,1), U(:,1), X(:,end), U(:,end), []);
dx = ocp_dyn_map(t(1:end-1), X(:,1:end-1), U(:,1:end-1), X(:,2:end), U(:,2:end), [], diff(t));
c = ocp_path_map(t, X, U, [], [diff(t), t(end)-t(end-1)]);
b = ocp_bcs(X(:,1), U(:,1), X(:,end), U(:,end), []);
g = [dx; c(:,1:end-1)];
g = [g(:); c(:,end); b];
J = sum(dl) + m;

% Bounds
lbz = repmat([lbx(:); lbu(:)], [N, 1]);
ubz = repmat([ubx(:); ubu(:)], [N, 1]);
lbg = repmat([lbx(:)*0; lbc(:)], [N-1, 1]); lbg = [lbg; lbc(:); lbb(:)];
ubg = repmat([ubx(:)*0; ubc(:)], [N-1, 1]); ubg = [ubg; ubc(:); ubb(:)];

end

