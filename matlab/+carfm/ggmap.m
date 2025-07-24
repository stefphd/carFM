function res = ggmap(car, V0, g0, opts, guess)
% GGMAP - See help/ggmap.m for the help of this function.

% Check args
arguments
    car (1,1) struct {carfm.common.checkCarStruct(car)}
    V0 (:,:) {mustBeNumeric,mustBeReal,mustBePositive}
    g0 (:,:) {mustBeNumeric,mustBeReal,mustBePositive,carfm.common.mustBeEqualSize1(V0,g0)} = 1
    opts (1,1) struct {carfm.ssa.checkOptStruct(opts)} = struct()
    guess (:,1) struct {carfm.ssa.checkGGguess(guess, V0, g0, opts)} = struct()
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

% Default options and override with user-defined options
opts = carfm.ssa.getDefaultOptions(opts);

% Give error for ideal tyres
if any(opts.iTyreType(2:3)==0)
    error('carfm:invalidType', 'Tyres must use the practical model (1) for g-g computation')
end
    
% Generate symbolic expression using CasADi only if ~usePrebuilt or ~mex
% this is to reduce overhead when not necessary, thus reducing function inizialization time
if ~opts.mex || ~opts.usePrebuilt
    % Create symbolic alpha, V, gz, a__t0, rho, y
    alpha = casadi.MX.sym('alpha');
    V = casadi.MX.sym('V');
    g__z = casadi.MX.sym('gz');
    a__t0 = casadi.MX.sym('at0'); % a__t shift
    rho = casadi.MX.sym('rho');
    y = casadi.MX.sym('y', 30, 1); % phi, mu, ..., Tau__t
    
    % Create opt vars z = [rho
    %                      phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
    %                      X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
    %                      F__fl, F__fr, F__rl, F__rr, Tau__t
    %                      ]
    % z = [rho;y] with y = [phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
    %                       X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
    %                       F__fl, F__fr, F__rl, F__rr, Tau__t
    %                       ]
    z = [rho; 
         y];
    
    % Build SS vector x = [phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
    %                      X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
    %                      F__fl, F__fr, F__rl, F__rr, Tau__t, V, yaw__rate, a__t]
    % shifted polar coordinates alpha (angle) and rho (radius), centered at (0, a__t0)
    % at0 = opts.GGshift(V); % a__t shift
    a__t = (rho*sin(alpha) + a__t0) * car.gravity;
    a__n = rho*cos(alpha) * car.gravity;
    yaw__rate = a__n/V;
    x = [y; % from phi to Tau__f
         V/opts.xscale(31); % V 
         yaw__rate/opts.xscale(32); % yaw__rate
         a__t/opts.xscale(33);  % a__t
         ];
    
    % Fake u_long, u_lat: these are not used inside g-g, b/c necessary
    % variables (i.e. V, yaw__rate, etc) are inside the x vars
    % This also simplifies the SSA: SS is totally defined by x
    u_long0 = nan(3,1); 
    u_lat0 = nan(3,1);
    opts.ssActiveLongInputs = false(1,3); % discard active long eqs
    opts.ssActiveLatInputs = false(1,3); % discard active lat eqs
    
    % Set vertical acc gz_g
    opts.gz_g = g__z;
    
    % Create constraint function [gneq, geq, grad_gneq, grad_geq] = g(z,alpha,V,gz,at0)
    car_casadi = carfm.common.matlab2casadiInterpolant(car); % use casadi interpolants instead of griddedInterpolant
    [ssres, ssdata] = carfm.ssa.steadyStateEquations(x, u_long0, u_lat0, car_casadi, opts); % SS residual and data
    gneq = [opts.Neps - ssdata.frontLeftTyre.N/car.mass/car.gravity  % N__fl > opts.Neps
            opts.Neps - ssdata.frontRightTyre.N/car.mass/car.gravity % N__fr > opts.Neps
            opts.Neps - ssdata.rearLeftTyre.N/car.mass/car.gravity   % N__fl > opts.Neps
            opts.Neps - ssdata.rearRightTyre.N/car.mass/car.gravity  % N__fr > opts.Neps
            (ssdata.engineTorque - ssdata.engineMaxTorque)/1000      % engineTorque < engineMaxTorque
            ]; % inequality constraint
    geq = ssres; % equality constraints
    g = casadi.Function('g',{z, alpha, V, g__z, a__t0}, {gneq, geq, jacobian(gneq,z)', jacobian(geq,z)'});
    
    % Create cost function [L, grad_L] = f(z,alpha,V,gz,at0)
    % pospart = @(x,eps) 0.5*(x+sqrt(x.^2+eps));
    pnlt = opts.wdelta*ssdata.steerAngle^2 + ...
           opts.wlambda*ssdata.driftAngle^2 + ...
           opts.wkappa*(ssdata.rearLeftTyre.longSlip^2 + ssdata.rearRightTyre.longSlip^2); % + ...
           % opts.wkappa*(ssdata.frontLeftTyre.longSlip^2 + ssdata.frontRightTyre.longSlip^2);
    L = -rho^2 + pnlt*0;
    f = casadi.Function('f', {z, alpha, V, g__z, a__t0}, {L, gradient(L,z)});
    
    % Create Hessian H = H(z,lambda_neq,lambda_eq,alpha,V,gz,at0)
    % of the lagragian L + lambda_neq' * gneq + lambda_eq' * geq
    % lambda_neq = casadi.MX.sym('lambda_neq', numel(gneq), 1);
    % lambda_eq = casadi.MX.sym('lambda_eq', numel(geq), 1);
    % H = casadi.Function('H', {z, lambda_neq, lambda_eq, alpha, V, gz, at0}, {hessian(L + dot(lambda_neq,gneq) + dot(lambda_eq,geq), z)});
    
    % Create get z function z = getzfromx(x)
    x1 = casadi.MX.sym('x', numel(x), 1);
    V1 = x1(31)*opts.xscale(31);
    yaw__rate1 = x1(32)*opts.xscale(32);
    a__t1 = x1(33)*opts.xscale(33);
    a__n1 = yaw__rate1*V1;
    rho1 = sqrt((a__t1-opts.GGshift(V1)*car.gravity)^2+a__n1^2) / car.gravity;
    z1 = [rho1; x1(1:end-3)];
    getzfromx = casadi.Function('getzfromx', {x1}, {z1});
    
    % Create guess function z0 = get_z0(rho,alpha,V,gz,at0)
    x0 = carfm.ssa.steadyStateGuess([V;a__t;0], [yaw__rate;0;0], car_casadi, opts);
    z0 = getzfromx(x0);
    get_z0 = casadi.Function('get_z0', {rho,alpha,V,g__z,a__t0}, {z0});
    
    % Create post-process function
    getpost = casadi.Function('getpost', {z,alpha,V,g__z,a__t0}, { x ...
        ssdata.tangentialAcc, ssdata.normalAcc, ...
        ssdata.longitudinalAcc, ssdata.lateralAcc});
end

% Variable bounds
% z = [rho
%      phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
%      X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
%      F__fl, F__fr, F__rl, F__rr, Tau__t
%      ]
lbz = [0; % rho
       [-15/180*pi; -15/180*pi; -0.25; -30/180*pi; -0.5; -0.5; -0.5; -0.5; -30/180*pi; % phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda,
       -opts.kappaLim; -opts.kappaLim; -opts.kappaLim; -opts.kappaLim; % kappa__fl, kappa__fr, kappa__rl, kappa__rr
       -10e3; -10e3; -10e3; -10e3; -10e3; -10e3; -10e3; -10e3; 0; 0; 0; 0; % X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
       -10e3; -10e3; -10e3; -10e3; -10e3; % F__fl, F__fr, F__rl, F__rr, Tau__t
       ]./opts.xscale(1:30)']; % lower bound for z
ubz = [5; % rho
       [+15/180*pi; +15/180*pi; +0.25; +30/180*pi; +0.5; +0.5; +0.5; +0.5; +30/180*pi; % phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda,
       +opts.kappaLim; +opts.kappaLim; +opts.kappaLim; +opts.kappaLim; % kappa__fl, kappa__fr, kappa__rl, kappa__rr
       10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; % X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
       10e3; 10e3; 10e3; 10e3; 10e3; % F__fl, F__fr, F__rl, F__rr, Tau__t
       ]./opts.xscale(1:30)']; % upper bound for z

% Mex
% for mex=true, force build if buildOnly=true, regardless usePrebuilt
if opts.mex && (~opts.usePrebuilt || opts.buildOnly)
    [flag, errid, errmsg] = carfm.ssa.mexFile(opts.ggMexName, f, g, get_z0, getzfromx, getpost); %, H); % mex functions
    if flag
        % Reset and exit
        clearvars -except filepath errmsg
        carfm.common.setEnvironment(filepath, false);
        % Give error if sth wrong
        error(errid, '%s', errmsg);
    end
end

% Check for build only
if opts.mex && opts.buildOnly
    % Reset and exit
    clearvars -except filepath
    carfm.common.setEnvironment(filepath, false);
    res = struct([]); % empty output argument
    return;
end
    
% Mex exists?
if opts.mex 
    if exist(opts.ggMexName, 'file')~=3 % id=3 for mex file (https://it.mathworks.com/help/matlab/ref/exist.html)
        % Reset and exit
        clearvars -except filepath opts
        carfm.common.setEnvironment(filepath, false);
        % Give error
        error('carfm:notFound','Unable to find MEX function %s.%s', opts.ggMexName, mexext)
    end
end

% Get function
if opts.mex
    % Function handles for easier access to the functions in the mex
    ggsolver_fh = str2func(opts.ggMexName); % function handle for ggsolver
    eval_cost = @(z, alpha, V, gz, at0) ggsolver_fh('f', z, alpha, V, gz, at0);
    eval_nlcon = @(z, alpha, V, gz, at0) ggsolver_fh('g', z, alpha, V, gz, at0);
    eval_get_z0 = @(rho, alpha, V, gz, at0) ggsolver_fh('get_z0', rho, alpha, V, gz, at0);
    eval_getzfromx = @(x) ggsolver_fh('getzfromx', x);
    eval_getpost = @(z, alpha, V, gz, at0) ggsolver_fh('getpost', z, alpha, V, gz, at0);
    % eval_hessian = @(z, lambda_neq, lambda_eq, alpha, V, gz, at0) ggsolver_fh('H', z, lambda_neq, lambda_eq, alpha, V, gz, at0);
else
    % Function handles which return data in double (and not casadi.DM type)
    eval_cost = returntypes('full', f);
    eval_nlcon = returntypes('full', g);
    eval_get_z0 = returntypes('full', get_z0);
    eval_getzfromx = returntypes('full', getzfromx);
    eval_getpost = returntypes('full', getpost);
    % eval_hessian = returntypes('full', H);
end

% Solver g-g
alpha0 = linspace(-pi/2, pi/2, opts.numGGpts); % -pi/2 to pi/2 (right-hand side)
if ~opts.isSymGG
    alpha1 = linspace(-pi/2, -3/2*pi, opts.numGGpts); % -pi/2 to -3/2*pi (left-hand side)
    % alpha1(alpha1<-pi) = alpha1(alpha1<-pi)+2*pi; % [-pi,-3/2*pi] to [pi,pi/2]
    alpha0 = [alpha0, alpha1(2:end-1)]; % cat alpha0 and alpha1 for non-sym G-G
end
na = numel(alpha0);
nV = size(V0,2);
ng = size(g0,2);
nx = numel(opts.xscale);
rhoopt = nan(na, nV, ng); % matrix containing optimal rho
atopt = nan(na, nV, ng); % matrix containing optimal at
anopt = nan(na, nV, ng); % matrix containing optimal an
axopt = nan(na, nV, ng); % matrix containing optimal ax
ayopt = nan(na, nV, ng); % matrix containing optimal ay
Xopt = nan(na, nV, ng, nx); % matrix containing optimal x
at0 = zeros(nV, ng); % matrix containing at0
exitflag = zeros(na, nV, ng); % matrix containing exit flag
% vars for double-sided G-G
if ~opts.isSymGG
    i1 = opts.numGGpts+(1:ceil((opts.numGGpts-1)/2));
    i2 = opts.numGGpts+(ceil((opts.numGGpts-1)/2):(opts.numGGpts-2));
    i0 = 1:opts.numGGpts;
    cat_var = @(x) cat(1, flip(x(i1,:,:,:), 1), ...
                          x(i0,:,:,:), ...
                          flip(x(i2,:,:,:), 1));
end

% fmincon options
opti_opt = optimoptions('fmincon',...
                       'Algorithm', opts.algorithmGGopt, ...
                       'Display', 'none',...
                       'SpecifyObjectiveGradient', true,...
                       'SpecifyConstraintGradient', true, ...
                       'SubproblemAlgorithm','factorization' ...
                       ); 
% loop over rows of V0,g0
N = size(V0,1); % number of grids
res = repmat(struct('alpha', [], 'shift', [], ...
            'V', [], 'g', [], ... 
            'at0', [], 'rho', [], ... 'gamma', [], ...
            'at', [], 'an', [], 'ax', [], 'ay', [], ...
            'x', [], 'sseval', [], 'exitflag', []), ...
      [N, 1]); % init res
for iN = 1 : N
    % loop over g
    for ig = 1 : ng
        gk = g0(iN,ig);
        % Loop over V
        for iV = 1 : nV
            Vk = V0(iN,iV); % current speed
            % Vertical shift
            at0k = opts.GGshift(Vk);
            at0(iV,ig) = at0k;
            % Find suitabe guess for alpha0(1) (i.e. pure ax)
            if isempty(fieldnames(guess))
                rho_try = linspace(lbz(1), ubz(1), 1001); % try alpha from lbz to ubz
                nlcon_try = @(rho) eval_nlcon(eval_get_z0(rho,alpha0(1),Vk,gk,at0k),alpha0(1),Vk,gk,at0k); % eval nlcon in alpha=alpha0(1),V=Vk,gtilda=gtildak
                rho0 = 0; % init
                for k = 1 : numel(rho_try)
                    rho0 = rho_try(k);
                    nlcon0 = nlcon_try(rho0);
                    if any(nlcon0(1:end-1)>-5e-3) % stop cycle as soon as one of the limits excedeed
                        break;
                    end
                end
                % Get guess z
                z0 = eval_get_z0(rho0, alpha0(1), Vk, gk, at0k);
            end
            % Print current computation
            fprintf("Calculating G-G at V(%d,%d)=%.1fm/s, gz(%d,%d)=%.2fg\n", iN, iV, Vk, iN, ig, gk);
            % Loop over alpha
            fails = 0;
            for ia = 1 : na
                alphak = alpha0(ia); % current alpha
                args = {alphak,Vk,gk,at0k};
                % Non-sym G-G at 
                if ia==(opts.numGGpts+1) % left side begins (if any)
                    x0 = Xopt(ia-1,iV,ig,:);
                    z0 = eval_getzfromx(x0(:));
                end
                % Get the guess if any
                % TODO fix for sym/non-sym G-G!!!
                if ~isempty(fieldnames(guess))
                    x0 = guess(iN).x(ia,iV,ig,:);
                    z0 = eval_getzfromx(x0(:));
                end
                % Provide hessian 
                % opti_opt.HessianFcn = @(z, lam) (eval_hessian(z, lam.ineqnonlin, lam.eqnonlin, alphak, Vk, gk, at0k));
                % Call to fmincon
                zopt = z0; % init 
                [zopt,~, exitflag(ia,iV,ig), ~, lamopt] = fmincon(@(z) eval_cost(z,args{:}), zopt,[],[],[],[], lbz,ubz, @(z) eval_nlcon(z,args{:}),opti_opt);
                if exitflag(ia,iV,ig)<=0 % try again ...
                    [zopt,~, exitflag(ia,iV,ig), ~, lamopt] = fmincon(@(z) eval_cost(z,args{:}), zopt,[],[],[],[], lbz,ubz, @(z) eval_nlcon(z,args{:}),opti_opt);
                end
                if exitflag(ia,iV,ig)<=0 % change guess ...
                    z0 = eval_get_z0(1, alphak, Vk, gk, at0k);
                    zopt = z0; % init 
                    [zopt,~, exitflag(ia,iV,ig), ~, lamopt] = fmincon(@(z) eval_cost(z,args{:}), zopt,[],[],[],[], lbz,ubz, @(z) eval_nlcon(z,args{:}),opti_opt);
                end
                if exitflag(ia,iV,ig)<=0 % failed
                    fprintf("Fail at alpha(%d)=%.3f\n", ia, alphak);
                    fails = fails + 1;
                    Xopt(ia,iV,ig,:) = nan(nx,1);
                else
                    % Save solution
                    z0 = zopt;
                    rhoopt(ia,iV,ig) = zopt(1);
                    % Post process
                    [Xopt(ia,iV,ig,:), atopt(ia,iV,ig), anopt(ia,iV,ig), ...
                        axopt(ia,iV,ig), ayopt(ia,iV,ig)] = eval_getpost(zopt, alphak, Vk, gk, at0k);
                    % check grad(f) + lambda' * J = 0
                    % lam_eq = lamopt.eqnonlin;
                    % lam_neq = lamopt.ineqnonlin;
                    % lam_u = lamopt.upper;
                    % lam_l = lamopt.lower;
                    % [f, gradf] = eval_cost(z0,args{:});
                    % [gneq, geq, Jneqp, Jeqp] = eval_nlcon(z0,args{:});
                    % lam = [lam_eq; lam_neq; lam_u; lam_l];
                    % Jp = [Jeqp, Jneqp, eye(numel(z)), -eye(numel(z))];                              
                    % gradf + Jp*lam;
                end
            end
            % Print
            fprintf("Done (%d fails)\n", fails);
        end
    end
    
    % Fix near zero values
    % atopt(abs(atopt)<1e-12) = 0;
    % anopt(abs(anopt)<1e-12) = 0;
    % axopt(abs(axopt)<1e-12) = 0;
    % ayopt(abs(ayopt)<1e-12) = 0;
    
    % Save res
    if opts.isSymGG % single-sided G-G
        alphares = alpha0;
        rhores = rhoopt;
        atres = atopt;
        anres = anopt;
        axres = axopt;
        ayres = ayopt;
        Xres = Xopt;
        exitflagres = exitflag;
    else % double-sided G-G
        alphares = cat(2, flip(alpha0(i1), 2), alpha0(i0), 2*pi+flip(alpha0(i2), 2));
        rhores = cat_var(rhoopt);
        atres = cat_var(atopt);
        anres = cat_var(anopt);
        axres = cat_var(axopt);
        ayres = cat_var(ayopt);
        Xres = cat_var(Xopt);
        exitflagres = cat_var(exitflag);
    end
    res(iN).alpha = alphares;
    res(iN).shift = opts.GGshift;
    res(iN).V = V0(iN,:);
    res(iN).g = g0(iN,:);
    res(iN).at0 = at0;
    res(iN).rho = rhores;
    res(iN).at = atres;
    res(iN).an = anres;
    res(iN).ax = axres;
    res(iN).ay = ayres;
    res(iN).x = Xres;
    opts.gz_g = res(iN).g; % all g and rem CasADi MX
    res(iN).sseval = @(ia,iV,ig) carfm.ssa.sseval(ia, iV, ig, res(iN).x, car, opts);
    res(iN).exitflag = exitflagres;

    % Reset matrices - is this necessary?
    rhoopt(:) = nan;
    atopt(:) = nan;
    anopt(:) = nan;
    axopt(:) = nan;
    ayopt(:) = nan;
    Xopt(:) = nan;
    at0(:) = 0;
    exitflag(:) = 0;

end

% Fill failed points
if opts.fillFailed
    fields = {'rho','at','an','ax','ay','x'}; % fields to fill
    for iN = 1 : N % loop over V,g rows
        for k1 = 1 : numel(res(iN).V) % loop over V
            for k2 = 1 : numel(res(iN).g) % loop over g
                if any(res(iN).exitflag(:,k1,k2)<=0) % exitflag >= 1
                    numFailed = sum(res(iN).exitflag(:,k1,k2)<=0);
                    fprintf("Warning: filled %d missing values in g-g at V(%d,%d)=%g, g(%d,%d)=%g\n", ...
                        numFailed, iN, k1, res(iN).V(k1), iN, k2, res(iN).g(k2));
                    % fill each field
                    for k0 = 1 : numel(fields)
                        field = fields{k0};
                        for k3 = 1 : size(res(iN).(field), 4)
                            res(iN).(field)(:,k1,k2,k3) = fillmissing(res(iN).(field)(:,k1,k2,k3),"linear");
                        end
                    end
                end
            end
        end 
    end
end

% Clear mex meory
if opts.mex
    clear(opts.ggMexName)
end

% Rem path
clearvars -except res filepath
carfm.common.setEnvironment(filepath, false);

end