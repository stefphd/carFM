function gg = ggmap(car, V0, g0, opts, guess)
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

% Get undocumented options and override with user-defined options
opts = carfm.ssa.getUndocOptions(opts);

% Give error for ideal tyres
if any(opts.iTyreType(2:3)==0)
    error('carfm:invalidType', 'Tyres must use the practical model (1) for g-g computation')
end

% Check if sensitivity parameters exist in bike struct
[flag, errid, errmsg] = carfm.common.checkSensitivityPar(opts.sensitivityPar, car);
if flag
    % Reset and exit
    clearvars -except filepath errmsg errid
    carfm.common.setEnvironment(filepath, false);
    % Give error if sth went wrong
    error(errid, '%s', errmsg);
end

% Extract sensitivity parameters
opts.sensitivityPar = carfm.common.expandSensitivityPar(opts.sensitivityPar, car);
np = numel(opts.sensitivityPar); % num of sensitivies

% Variable bounds
% z = [rho
%      phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda, kappa__fl, kappa__fr, kappa__rl, kappa__rr
%      X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
%      F__fl, F__fr, F__rl, F__rr, Tau__t
%      ]
Nmin = opts.Neps*car.gravity*car.mass; % N__ij > Nmin
lbz = [0; % rho
       [-15/180*pi; -15/180*pi; -0.25; -30/180*pi; -0.5; -0.5; -0.5; -0.5; -30/180*pi; % phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda,
       opts.minLongSlip(1); opts.minLongSlip(1); opts.minLongSlip(2); opts.minLongSlip(2); % kappa__fl, kappa__fr, kappa__rl, kappa__rr
       -10e3; -10e3; -10e3; -10e3; -10e3; -10e3; -10e3; -10e3; Nmin; Nmin; Nmin; Nmin; % X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
       -10e3; -10e3; -10e3; -10e3; -10e3; % F__fl, F__fr, F__rl, F__rr, Tau__t
       ]./opts.xscale(1:30)']; % lower bound for z
ubz = [5; % rho
       [+15/180*pi; +15/180*pi; +0.25; +30/180*pi; +0.5; +0.5; +0.5; +0.5; +30/180*pi; % phi, mu, z, delta, z__fl, z__fr, z__rl, z__rr, lambda,
       opts.maxLongSlip(1); opts.maxLongSlip(1); opts.maxLongSlip(2); opts.maxLongSlip(2); % kappa__fl, kappa__fr, kappa__rl, kappa__rr
       10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; 10e3; % X__fl,X__fr,X__rl,X__rr,Y__fl,Y__fr,Y__rl,Y__rr,N__fl,N__fr,N__rl,N__rr
       10e3; 10e3; 10e3; 10e3; 10e3; % F__fl, F__fr, F__rl, F__rr, Tau__t
       ]./opts.xscale(1:30)']; % upper bound for z

% Generate symbolic expression using CasADi only if ~usePrebuilt or ~mex
% this is to reduce overhead when not necessary, thus reducing function inizialization time
if ~opts.mex || ~opts.usePrebuilt
    % Create symbolic alpha, V, gz, a__t0, rho, y
    alpha = casadi.MX.sym('alpha');
    V = casadi.MX.sym('V');
    g__z = casadi.MX.sym('gz');
    w = casadi.MX.sym('w', 3); % wdelta, wlambda, wkappa
    a__t0 = casadi.MX.sym('at0'); % a__t shift
    p = casadi.MX.sym('p', np); % 0x1 if no sensitivities
    rho = casadi.MX.sym('rho');
    y = casadi.MX.sym('y', 30, 1); % phi, mu, ..., Tau__t

    % args
    args = {alpha, V, g__z, a__t0, w};
    
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
    
    % Set vertical acc gravityFactors
    opts.gravityFactors = [0,0,g__z];
    
    % Create constraint function [gneq, geq, grad_gneq, grad_geq] = g(z,alpha,V,gz,at0)
    car_casadi = carfm.common.matlab2casadiInterpolant(car); % use casadi interpolants instead of griddedInterpolant
    [car_casadi, p0] = carfm.common.replaceField(car_casadi, opts.sensitivityPar, p);
    [ssres, ssdata] = carfm.ssa.steadyStateEquations(x, u_long0, u_lat0, car_casadi, opts); % SS residual and data
    gneq = [(ssdata.engineTorque - ssdata.engineMaxTorque)/1000      % engineTorque < engineMaxTorque
            ssdata.frontLeftTyre.sideSlip^2 - opts.maxSideSlip(1)^2  % |alpha__fl| < opts.maxSideSlip
            ssdata.frontRightTyre.sideSlip^2 - opts.maxSideSlip(1)^2 % |alpha__fr| < opts.maxSideSlip
            ssdata.rearLeftTyre.sideSlip^2 - opts.maxSideSlip(1)^2   % |alpha__rl| < opts.maxSideSlip
            ssdata.rearRightTyre.sideSlip^2 - opts.maxSideSlip(1)^2  % |alpha__rr| < opts.maxSideSlip
            ]; % inequality constraint
    geq = ssres; % equality constraints
    Jneq = jacobian(gneq,z); % dgneq/dz
    Jeq = jacobian(geq,z); % dgeq/dz
    g = casadi.Function('g',[{z}, args(:)'], {casadi.substitute(gneq,p,p0), casadi.substitute(geq,p,p0), casadi.substitute(Jneq,p,p0)', casadi.substitute(Jeq,p,p0)'});
    
    % Create cost function [L, grad_L] = f(z,alpha,V,gz,at0)
    % Undoc feature: opts.useKappafPenalty to add penalty on front slip
    % ratio (default false)
    pnlt = [w(1)*ssdata.steerAngle^2, ...
        w(2)*ssdata.driftAngle^2, ...
        w(3)*(ssdata.rearLeftTyre.longSlip^2 + ssdata.rearRightTyre.longSlip^2) ...
        + w(3)*(ssdata.frontLeftTyre.longSlip^2 + ssdata.frontRightTyre.longSlip^2)*opts.useKappafPenalty, ...
        ]; % penalty terms
    l = -rho^2 + sum(pnlt);
    lz = gradient(l,z); % dl/dz
    f = casadi.Function('f', [{z}, args(:)'], {casadi.substitute(l,p,p0), casadi.substitute(lz,p,p0)});

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
    get_z0 = casadi.Function('get_z0', [{rho},args(:)'], {casadi.substitute(z0,p,p0)});
    
    % KKT equations using barrier method
    lamneq = casadi.MX.sym('lamneq', numel(gneq), 1);
    lameq = casadi.MX.sym('lameq', numel(geq), 1);
    lamlb = casadi.MX.sym('lamlb', numel(z), 1);
    lamub = casadi.MX.sym('lamub', numel(z), 1);
    L = l + dot(lamneq,gneq) + dot(lameq,geq); % lagragian
    Lp = gradient(L,p); % cost sensitivity
    H = hessian(L,z); % hessian
    % % Using CasADi
    % q = [z; lameq; lamneq; lamlb; lamub]; % primal-dual variables
    % Lz = gradient(L,z);
    % G = [Lz-lamlb+lamub;
    %      geq;
    %      (z-lbz).*lamlb;
    %      (ubz-z).*lamub;
    %      -gneq.*lamneq];
    % Gq = jacobian(G,q);
    % Gp = jacobian(G,p);
    % getkkt = casadi.Function('KKT',{z,alpha,V,g__z,a__t0,lameq,lamneq,lamlb,lamub},...
    %             {casadi.substitute(G,p,p0),casadi.substitute(Gq,p,p0),casadi.substitute(Gp,p,p0)});
    % Sensitivity matrices
    H0 = H; % exact hessian
    H1 = casadi.MX.sym('H', H0.sparsity()); % approx hessian: same sparsity pattern of exact hessian?
    H = H0; % H0: use exact hessian, H1: use approx hessian from fmincon to reduce MEXing time
    Gq = [H, jacobian(geq,z)' jacobian(gneq,z)' -diag(casadi.MX.ones(numel(z),1)) diag(casadi.MX.ones(numel(z),1))
          jacobian(geq,z) casadi.MX(numel(geq),numel(geq)) casadi.MX(numel(geq),numel(gneq)) casadi.MX(numel(geq),numel(z)) casadi.MX(numel(geq),numel(z))
          diag(lamlb) casadi.MX(numel(z),numel(geq)) casadi.MX(numel(z),numel(gneq)) diag(z-lbz) casadi.MX(numel(z),numel(z))
          -diag(lamub) casadi.MX(numel(z),numel(geq)) casadi.MX(numel(z),numel(gneq)) casadi.MX(numel(z),numel(z)) diag(ubz-z)
          -diag(lamneq)*jacobian(gneq,z) casadi.MX(numel(gneq),numel(geq)) -diag(gneq) casadi.MX(numel(gneq),numel(z)) casadi.MX(numel(gneq),numel(z))];
    Gp = [jacobian(gradient(L,z),p)
          jacobian(geq,p)
          casadi.MX(numel(z),numel(p))
          casadi.MX(numel(z),numel(p))
          -diag(lamneq)*jacobian(gneq,p)
          ];
    % Sensitivity analysis
    if opts.useExactSensitivity
        % apply IFT to G(q,p)=0: Gq*dq+Gp*dp=0 -> dq/dp = -inv(Gq)*Gp
        % to find primal-dual sensitivity
        Qp = -Gq \ Gp; % casadi mldivide uses casadi.Linsol with 'qr' internally
        spar = Qp(1,:); % first var is rho
    else
        % use lagrange multipliers to find cost sensitivity
        % approx b/c of penalty in cost, and thus need correction to find
        % rho sensitivity. Analytical formula is 
        % drho/dp = (dl/dp) / (dl/drho) * (1 - (dpnlt/dp)/(dl/dp))
        % i.e. approx is (dpnlt/dp)/(dl/dp) = pnlt/l, i.e. ratio between
        % penalty and cost sensitivities is similar to the penalty-cost ratio
        spar = Lp / gradient(l,rho) * (1 - sum(pnlt)/l);
    end

    % Create post-process function
    spar = casadi.substitute(spar, p, p0); % eval at p=p0
    getpost = casadi.Function('getpost', [{z},args(:)',{lameq},{lamneq},{lamlb},{lamub},{H1}], { x, ...
        ssdata.tangentialAcc, ssdata.normalAcc, ...
        ssdata.longitudinalAcc, ssdata.lateralAcc, ...
        spar,casadi.substitute(pnlt,p,p0)});
end

% Reset options to rem casadi vars
opts.gravityFactors = [0, 0, opts.gz_g];

% Mex
% for mex=true, force build if buildOnly=true, regardless usePrebuilt
if opts.mex && (~opts.usePrebuilt || opts.buildOnly)
    [flag, errid, errmsg] = carfm.ssa.mexFile(opts.ggMexName, f, g, get_z0, getzfromx, getpost); % mex functions
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
    gg = struct([]); % empty output argument
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
    eval_cost = @(varargin) ggsolver_fh('f', varargin{:});
    eval_nlcon = @(varargin) ggsolver_fh('g', varargin{:});
    eval_get_z0 = @(varargin) ggsolver_fh('get_z0', varargin{:});
    eval_getzfromx = @(varargin) ggsolver_fh('getzfromx', varargin{:});
    eval_getpost = @(varargin) ggsolver_fh('getpost', varargin{:});
else
    % Function handles which return data in double (and not casadi.DM type)
    eval_cost = returntypes('full', f);
    eval_nlcon = returntypes('full', g);
    eval_get_z0 = returntypes('full', get_z0);
    eval_getzfromx = returntypes('full', getzfromx);
    eval_getpost = returntypes('full', getpost);
end

% Solver g-g
alphar = linspace(-pi/2, pi/2, opts.numGGpts); % -pi/2 to pi/2 (right side)
alphar( (alphar<opts.GGAngleRange(1)) | (alphar>opts.GGAngleRange(end)) ) = []; % exclude angles outside GGAngleRange
if ~opts.isSymGG
    alphal = flip(alphar - pi); % -3*pi/2 to -pi/2 (right side)
    % right and left sides
    alpha0 = [alphar, alphal]; % alpha0 = [-pi/2 to +pi/2, -pi/2 to -3*pi/2], from braking to acceleration
else
    % right side only
    alpha0 = alphar; % alpha0 = [-pi/2 to +pi/2], from braking to acceleration
end
na = numel(alpha0);
na1 = numel(alphar); % num alpha of one side only
nV = size(V0,2);
ng = size(g0,2);
nx = numel(opts.xscale);
sz = {na,nV,ng}; % sizes
rhoopt = nan(sz{:}); % matrix containing optimal rho
atopt = nan(sz{:}); % matrix containing optimal at
anopt = nan(sz{:}); % matrix containing optimal an
axopt = nan(sz{:}); % matrix containing optimal ax
ayopt = nan(sz{:}); % matrix containing optimal ay
Xopt = nan(sz{:},nx); % matrix containing optimal x
at0 = zeros(1,nV); % matrix containing at0
sparopt = zeros(sz{:},np); % matrix containing optimal sensitivities
pnltopt = zeros(sz{:},3); % penalty terms
exitflag = zeros(sz{:}); % matrix containing exit flag
% vars for double-sided GG
if ~opts.isSymGG
    i0 = find((alpha0>=-pi/2) & (alpha0<=+pi/2)); % right-side: alpha in [-pi/2,+pi/2]
    i1 = find((alpha0>=-pi) & (alpha0<-pi/2));    % lower left-side: alpha in [-pi,-pi/2)
    i2 = find((alpha0<=-pi) & (alpha0>-3*pi/2));  % upper left-side: alpha in (-3/2*pi,-pi]
    cat_ggvar = @(x) cat(1, flip(x(i1,:,:,:,:,:,:,:), 1), ...
                            x(i0,:,:,:,:,:,:,:), ...
                            flip(x(i2,:,:,:,:,:,:,:), 1));
end
% weights
w0 = [opts.wdelta, opts.wlambda, opts.wkappa];
% fmincon options
opti_opt = optimoptions('fmincon',...
                       'Algorithm', opts.algorithmGGopt, ...
                       'Display', 'none',...
                       'SpecifyObjectiveGradient', true,...
                       'SpecifyConstraintGradient', true, ...
                       'SubproblemAlgorithm','factorization' ...
                       ); 
% grids (V0,g0)
N = size(V0,1); % number of grids
gg = repmat(struct('alpha', [], 'shift', [], ...
            'V', [], 'g', [], ... 
            'at0', [], 'rho', [], ... 'gamma', [], ...
            'at', [], 'an', [], 'ax', [], 'ay', [], ...
            'x', [], 'sseval', [], 'exitflag', []), ...
      [N, 1]); % init res
if opts.computeGGFromBraking
    iavec = [1 : na1, na1+1 : na]; % from -pi/2 to +pi/2 for right side, from -pi/2 to -3/2*pi for left side
    ia1 = 1; % initial point of left side
    ia2 = na1+1; % initial point of right side
else
    iavec = [na1 : -1 : 1, na : -1 : na1+1]; %  from +pi/2 to -pi/2 for right side, from -3/2*pi to -pi/2 for left side
    ia1 = na1; % initial point of left side
    ia2 = na; % initial point of right side
end
% loop over gg
for l = 1 : N
    % loop over grid using linear index
    for i = 1 : nV*ng
        [iV,ig] = ind2sub([nV,ng],i);
        Vk = V0(l,iV);
        gk = g0(l,ig);
        at0k = opts.GGshift(Vk);
        at0(iV) = at0k;
        % Find guess for alpha0(ia1)
        if isempty(fieldnames(guess))
            % Use always rho=1
            args0 = {alpha0(ia1),Vk,gk,at0k,w0};
            z0 = eval_get_z0(1, args0{:});
        end
        % Print current computation
        fprintf("Calculating G-G at V(%d,%d)=%.1fm/s, gz(%d,%d)=%.2fg\n", l, iV, Vk, l, ig, gk);
        % Loop over alpha
        fails = 0;
        for ia = iavec
            alphak = alpha0(ia); % current alpha
            argsk = {alphak,Vk,gk,at0k,w0};
            idxk = {ia,iV,ig};
            % Non-sym GG
            if ~opts.isSymGG
                if ia==ia2 % left side begins
                    x0 = Xopt(ia1,idxk{2:end},:);
                    x0 = opts.xsign(:) .* x0(:); % fix the sign for opposed side
                    z0 = eval_getzfromx(x0); 
                end
            end
            % Get the guess (if any) 
            if ~isempty(fieldnames(guess))
                % no sequential guess OR first alpha 
                if ~opts.useGGSeqGuess || ia==ia1
                    % find nearest indexes
                    idx0 = carfm.ssa.findNearestGGPoint(argsk{1:7},guess);
                    x0 = guess(l).x(idx0{:},:);
                    z0 = eval_getzfromx(x0(:));
                end
            end
            % Call to fmincon
            zopt = z0; % init 
            [zopt,~, exitflag(idxk{:}), ~, lamopt, ~, Hopt] = fmincon(@(z) eval_cost(z,argsk{:}), zopt,[],[],[],[], lbz,ubz, @(z) eval_nlcon(z,argsk{:}),opti_opt);
            if exitflag(idxk{:})<=0 % try again ...
                [zopt,~, exitflag(idxk{:}), ~, lamopt, ~, Hopt] = fmincon(@(z) eval_cost(z,argsk{:}), zopt,[],[],[],[], lbz,ubz, @(z) eval_nlcon(z,argsk{:}),opti_opt);
            end
            if exitflag(idxk{:})<=0 % change guess ...
                z0 = eval_get_z0(1, argsk{:});
                zopt = z0; % init 
                [zopt,~, exitflag(idxk{:}), ~, lamopt, ~, Hopt] = fmincon(@(z) eval_cost(z,argsk{:}), zopt,[],[],[],[], lbz,ubz, @(z) eval_nlcon(z,argsk{:}),opti_opt);
            end
            if exitflag(idxk{:})<=0 % failed
                fprintf("Fail at alpha(%d)=%.3f\n", ia, alphak);
                fails = fails + 1;
                Xopt(idxk{:},:) = nan(nx,1);
            else
                % Save solution
                z0 = zopt;
                rhoopt(idxk{:}) = zopt(1);
                % Lagrange multipliers
                lameqopt = lamopt.eqnonlin;
                lamneqopt = lamopt.ineqnonlin;
                lamlbopt = lamopt.lower;
                lamubopt = lamopt.upper;    
                % Post process
                [Xopt(idxk{:},:), atopt(idxk{:}), anopt(idxk{:}), ...
                    axopt(idxk{:}), ayopt(idxk{:}), ...
                    sparopt(idxk{:},:), pnltopt(idxk{:},:)] = eval_getpost(zopt, argsk{:}, lameqopt, lamneqopt, lamlbopt, lamubopt, Hopt);
            end
        end
        % Print
        fprintf("Done (%d fails)\n", fails);
    end
    % Fix near zero values
    % atopt(abs(atopt)<1e-12) = 0;
    % anopt(abs(anopt)<1e-12) = 0;
    % axopt(abs(axopt)<1e-12) = 0;
    % ayopt(abs(ayopt)<1e-12) = 0;
    % Save res
    if opts.isSymGG % single-sided GG
        alphares = alpha0;
        rhores = rhoopt;
        atres = atopt;
        anres = anopt;
        axres = axopt;
        ayres = ayopt;
        sparres = sparopt;
        X = Xopt;
        pnltres = pnltopt;
        exitflagres = exitflag;
    else % double-sided GG
        alphares = cat(2, flip(alpha0(i1), 2), alpha0(i0), 2*pi+flip(alpha0(i2), 2));
        rhores = cat_ggvar(rhoopt);
        atres = cat_ggvar(atopt);
        anres = cat_ggvar(anopt);
        axres = cat_ggvar(axopt);
        ayres = cat_ggvar(ayopt);
        sparres = cat_ggvar(sparopt);
        X = cat_ggvar(Xopt);
        pnltres = cat_ggvar(pnltopt);
        exitflagres = cat_ggvar(exitflag);
    end
    gg(l).alpha = alphares;
    gg(l).shift = opts.GGshift;
    gg(l).V = V0(l,:);
    gg(l).g = g0(l,:);
    gg(l).at0 = at0;
    gg(l).rho = rhores;
    gg(l).at = atres;
    gg(l).an = anres;
    gg(l).ax = axres;
    gg(l).ay = ayres;
    gg(l).x = X;
    gg(l).spar = sparres;
    gg(l).sparnames = opts.sensitivityPar;
    gg(l).pnlt = pnltres;
    opts.gz_g = gg(l).g;
    gg(l).sseval = @(ia, iV, ig) carfm.ssa.sseval(ia, iV, ig, X, car, opts);
    gg(l).exitflag = exitflagres;
end

% Fill failed points
if opts.fillFailed
    fields = {'rho','at','an','ax','ay','x','spar','pnlt'}; % fields to fill
    for l = 1 : N % loop over V,g rows
        % loop over grid using linear index
        for i = 1 : nV*ng
            [iV,ig] = ind2sub([nV,ng],i);
            idx = {iV,ig};
            if any(gg(l).exitflag(:,idx{:})<=0) % exitflag >= 1
                numFailed = sum(gg(l).exitflag(:,idx{:})<=0);
                fprintf("Warning: filled %d missing values in g-g at V(%d,%d)=%g, g(%d,%d)=%g\n", ...
                    numFailed, l, iV, V0(l,iV), l, ig, g0(l,ig));
                % fill each field
                for k = 1 : numel(fields)
                    field = fields{k};
                    for j = 1 : size(gg(l).(field), 8)
                        gg(l).(field)(:,idx{:},j) = fillmissing(gg(l).(field)(:,idx{:},j),"linear");
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
clearvars -except gg filepath
carfm.common.setEnvironment(filepath, false);

end