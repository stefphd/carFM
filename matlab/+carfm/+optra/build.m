% File build.m 
% Build OPTra problem
% Copyright (C) 2024 Stefano Lovato

function build(name, ocp_runcost, ocp_bcscost, ocp_dyn, ocp_path, ocp_bcs, ocp_int, skip_hessian, outdir)
    % Checks args
    arguments 
        name (1,:) char
        ocp_runcost casadi.Function {check_fun(ocp_runcost, 'runcost', ocp_runcost)}
        ocp_bcscost casadi.Function {check_fun(ocp_bcscost, 'bcscost', ocp_runcost)}
        ocp_dyn casadi.Function {check_fun(ocp_dyn, 'dyn', ocp_runcost)}
        ocp_path casadi.Function {check_fun(ocp_path, 'path', ocp_runcost)}
        ocp_bcs casadi.Function {check_fun(ocp_bcs, 'bcs', ocp_runcost)}
        ocp_int casadi.Function {check_fun(ocp_int, 'int', ocp_runcost)}
        skip_hessian (1,1) {islogical} = false
        outdir (1,:) char = './'
    end

    % Codegen directories
    if ~endsWith(outdir, '/')
        outdir = [outdir '/'];
    end
    % Split name if contains path and assign to outdir
    [path, name] = fileparts(name);
    outdir = [outdir path];
    if ~endsWith(outdir, '/')
        outdir = [outdir '/'];
    end

    % Generate ocp functions
    import casadi.* % import casadi
    nx = ocp_runcost.size1_in(1);
    nu = ocp_runcost.size1_in(2);
    np = ocp_runcost.size1_in(5);
    na = -1;
    if (ocp_runcost.n_in() > 7)
        na = ocp_runcost.size1_in(7);
    end
    % variables
    x = casadi.MX.sym('x', nx);
    x1 = casadi.MX.sym('x1', nx); 
    x2 = casadi.MX.sym('x2', nx);
    u = casadi.MX.sym('u', nu); 
    u1 = casadi.MX.sym('u1', nu); 
    u2 = casadi.MX.sym('u2', nu); 
    xi = casadi.MX.sym('xi', nx); 
    ui = casadi.MX.sym('ui', nu); 
    xf = casadi.MX.sym('xf', nx);
    uf = casadi.MX.sym('uf', nu);
    p = casadi.MX.sym('p', np);
    t = casadi.MX.sym('t'); 
    h = casadi.MX.sym('h');
    % arguments
    args_runcost = {t, x1, u1, x2, u2, p, h};
    args_bcscost = {xi, ui, xf, uf, p};
    args_dyn = {t, x1, u1, x2, u2, p, h};
    args_path = {t, x, u, p, h};
    args_bcs = {xi, ui, xf, uf, p};
    args_int = {t, x1, u1, x2, u2, p, h};
    % add auxdata
    if na>=0
        auxdata = casadi.MX.sym('auxdata', na);
        args_runcost{end+1} = auxdata;
        args_bcscost{end+1} = auxdata;
        args_dyn{end+1} = auxdata;
        args_path{end+1} = auxdata;
        args_bcs{end+1} = auxdata;
        args_int{end+1} = auxdata;
    end
    % gradients
    ocp_runcost_grad = casadi.Function('ocp_runcost_grad', args_runcost, ...
                                    {gradient(ocp_runcost(args_runcost{:}), [x1; u1; x2; u2; p])});
    ocp_bcscost_grad = casadi.Function('ocp_bcscost_grad', args_bcscost, ...
                                    {gradient(ocp_bcscost(args_bcscost{:}), [xi; ui; xf; uf; p])});
    % jacobians
    ocp_dyn_jac = casadi.Function('ocp_dyn_jac', args_dyn, ...
                                    {jacobian(ocp_dyn(args_dyn{:}), [x1; u1; x2; u2; p])});
    ocp_path_jac = casadi.Function('ocp_path_jac', args_path, ...
                                    {jacobian(ocp_path(args_path{:}), [x; u; p])});
    ocp_bcs_jac = casadi.Function('ocp_bcs_jac', args_bcs, ...
                                    {jacobian(ocp_bcs(args_bcs{:}), [xi; ui; xf; uf; p])});
    ocp_int_jac = casadi.Function('ocp_int_jac', args_int, ...
                                    {jacobian(ocp_int(args_int{:}), [x1; u1; x2; u2; p])});
    % collect all function
    ocp_funcs = {ocp_dyn, ocp_path, ocp_bcs, ocp_int, ocp_runcost, ocp_bcscost, ...
                 ocp_dyn_jac, ocp_path_jac, ocp_bcs_jac, ocp_int_jac, ocp_runcost_grad, ocp_bcscost_grad}; 
    % hessians if not skip_hessian
    if ~skip_hessian
        % num of path and bcs
        nc = ocp_path.size1_out(0);
        nb = ocp_bcs.size1_out(0);
        nq = ocp_int.size1_out(0);
        % create CASADI variables for multipliers
        sigma = casadi.MX.sym('sigma'); % cost multiplier
        lamf = casadi.MX.sym('lamf', nx); % dynamic multiplier
        lamc = casadi.MX.sym('lamc', nc); % path multiplier
        lamb = casadi.MX.sym('lamb', nb); % bcs multiplier
        lamq = casadi.MX.sym('lamq', nq); % int multiplier
        % build lagragians
        lagb = sigma*ocp_bcscost(args_bcscost{:}); % boundary lagragian
        lagi = sigma*ocp_runcost(args_runcost{:}); % internal lagragian
        if nb > 0 % add bcs lagragian
            lagb = lagb + lamb'*ocp_bcs(args_bcs{:});
        end
        if nx > 0 % add dynamic lagragian
            lagi = lagi + lamf'*ocp_dyn(args_dyn{:});
        end
        if nc > 0 % add path lagragian
            lagb = lagb + lamc'*ocp_path(args_path{1}, xf, uf, args_path{4:end});
            lagi = lagi + lamc'*ocp_path(args_path{1}, x1, u1, args_path{4:end});
        end
        if nq > 0 % add integral lagragian
            lagi = lagi + lamq'*ocp_int(args_int{:});
        end
        % generate hessians of lagragians
        hessb = tril(hessian(lagb,[xi; ui; xf; uf; p]));
        hessi = tril(hessian(lagi,[x1; u1; x2; u2; p]));
        % create CASADI functions
        args_hessb = {t, xi, ui, xf, uf, p, h, sigma, lamc, lamb};
        args_hessi = {t, x1, u1, x2, u2, p, h, sigma, lamc, lamf, lamq};
        if na>=0
            args_hessb{end+1} = auxdata;
            args_hessi{end+1} = auxdata;
        end
        ocp_hessb = casadi.Function('ocp_hessb', args_hessb, { hessb });
        ocp_hessi = casadi.Function('ocp_hessi', args_hessi, { hessi });
        % append to ocp_funcs
        ocp_funcs{end+1} = ocp_hessb;
        ocp_funcs{end+1} = ocp_hessi;
    end

    % Create outdir if not exists
    if ~isfolder(outdir)
        mkdir(outdir);
    end

    % Generate code
    fprintf("Generating C code.\n")
    cfilename = [name '.c'];
    codegen_options.casadi_int = 'int'; % use casadi_int = int for consistency
    codegen_options.verbose = false; % remove comments in generated code to decrease dimension
    cg = CodeGenerator(cfilename, codegen_options); 
    % Append ocp functions to cg
    for k = 1 : numel(ocp_funcs)
        cg.add(ocp_funcs{k});
    end
    cg.generate(outdir); % generate c and h file in out dir
    fprintf("Done.\n")
    pause(0) % just to print out all fprintf

    % Compile library
    compile([outdir cfilename], name, outdir);

end

function compile(csource, libname, outdir)
    % Use MEX functionality to compile into fake MEX file, then 
    % change extension to DLL (or whatever it is depending on the
    % current platform). This allows to use the compiler set from
    % MATLAB, w/o the need to provide a compiler with OPTra.
    % Check if C file exists
    if ~isfile(csource)
        error('optra:buildFailed', 'Unable to find source C file ''%s''.', csource); 
    end
    % Set the library extension based on the platform
    if ispc
        libext = '.dll'; % 'dll' for Windows
    elseif ismac
        libext = '.dylib'; % 'dylib' for MacOS
    else % isunix
        libext = '.so';  % 'so' for Linux
    end
    % Check compiler
    compilerConfig = mex.getCompilerConfigurations('C', 'Selected');
    if isempty(compilerConfig)
        delete(csource);
        error('optra:buildFailed', 'No compiler configured for MEX. Configure a compiler using ''mex -setup''.'); 
    end
    % Set LDEXT
    flags{1} = sprintf('LDEXT=%s',libext);
    % Set OPTIMFLAGS
    flags{2} = '';
    if contains(compilerConfig.ShortName, 'msvc','IgnoreCase',true) % MSVC - this should apply to Windows only
        flags{2} = sprintf('OPTIMFLAGS=%s', '/O1 /Oy- /DNDEBUG'); % MSVC optimization
    else % gcc, clang - this applies to all platforms
        flags{2} = sprintf('COPTIMFLAGS=%s', '-O1 -fPIC -fwrapv -DNDEBUG'); % GCC-like optimization
    end
    % Unset MATLAB flags
    % this is to compile without linking to MATLAB libs
    flags{3} = 'MATLABMEX=';
    flags{4} = 'LINKEXPORT=';
    flags{5} = 'LINKEXPORTVER=';
    flags{6} = 'LINKLIBS=';
    if ~ispc
        flags{6} = 'LINKLIBS=-lm'; % this is to fix "undefined reference to `sincos'" when cc in linux
    end
    % Run the build process
    fprintf("Building library %s%s.\n", [outdir libname], libext);
    pause(0) % just to print out all fprintf
    tic;
    try
        mex(csource, "-output", libname, "-outdir", outdir, flags{:});
    catch err
        delete(csource);
        error('optra:buildFailed', '%s', err.message);
    end
    compileTime = toc;
    % Print end message
    fprintf("Library %s%s built in %.2fs.\n", [outdir libname], libext, compileTime);
    pause(0) % just to print out all fprintf
    % Clean
    delete(csource)
end

function check_name(fun, name)
    if ~strcmp(fun.name, name)
        error('build:invalidInputs', '%s must have name ''%s''.', name, name);
    end
end

function check_nargs(fun, nin, nout)
    if (fun.n_in ~= nin) || (fun.n_out ~= nout)
        error('build:invalidInputs','%s must have %d inputs and %d output (found %d and %d).', fun.name, nin, nout, fun.n_in, fun.n_out);
    end
end

function check_argin(fun, i, sz)
    sz0 = fun.size_in(i);
    if ~isequal(sz0, sz)
        error('build:invalidInputs','Input[%d] of %s must have size %d-by-%d (found %d-by-%d).', i, fun.name, sz(1), sz(2), sz0(1), sz0(2));
    end
end

function check_argout(fun, i, sz)
    sz0 = fun.size_out(i);
    if ~isequal(sz0(~isinf(sz)), sz(~isinf(sz)))
        error('build:invalidInputs','Input[%d] of %s must have size %d-by-%d (found %d-by-%d).', i, fun.name, sz(1), sz(2), sz0(1), sz0(2));
    end
end

function check_fun(fun, flag, reffun)
    nx = reffun.size1_in(1);
    nu = reffun.size1_in(2);
    np = reffun.size1_in(5);
    na = -1;
    if (reffun.n_in() > 7)
        na = reffun.size1_in(7);
    end
    switch (flag)
        case 'runcost'
            check_name(fun, 'ocp_runcost');
            check_nargs(fun, 7+(na>=0), 1);
            check_argin(fun, 0, [1 1]);
            check_argin(fun, 1, [nx 1]);
            check_argin(fun, 2, [nu 1]);
            check_argin(fun, 3, [nx 1]);
            check_argin(fun, 4, [nu 1]);
            check_argin(fun, 5, [np 1]);
            check_argin(fun, 6, [1 1]);
            if na>=0
                check_argin(fun, 7, [na 1]);
            end
            check_argout(fun, 0, [1 1]);
        case 'bcscost'
            check_name(fun, 'ocp_bcscost');
            check_nargs(fun, 5+(na>=0), 1);
            check_argin(fun, 0, [nx 1]);
            check_argin(fun, 1, [nu 1]);
            check_argin(fun, 2, [nx 1]);
            check_argin(fun, 3, [nu 1]);
            check_argin(fun, 4, [np 1]);
            if na>=0
                check_argin(fun, 5, [na 1]);
            end
            check_argout(fun, 0, [1 1]);
        case 'dyn' 
            check_name(fun, 'ocp_dyn');
            check_nargs(fun, 7+(na>=0), 1);
            check_argin(fun, 0, [1 1]);
            check_argin(fun, 1, [nx 1]);
            check_argin(fun, 2, [nu 1]);
            check_argin(fun, 3, [nx 1]);
            check_argin(fun, 4, [nu 1]);
            check_argin(fun, 5, [np 1]);
            check_argin(fun, 6, [1 1]);
            if na>=0
                check_argin(fun, 7, [na 1]);
            end
            check_argout(fun, 0, [nx 1]);
        case 'path'
            check_name(fun, 'ocp_path');
            check_nargs(fun, 5+(na>=0), 1);
            check_argin(fun, 0, [1 1]);
            check_argin(fun, 1, [nx 1]);
            check_argin(fun, 2, [nu 1]);
            check_argin(fun, 3, [np 1]);
            check_argin(fun, 4, [1 1]);
            if na>=0
                check_argin(fun, 5, [na 1]);
            end
            sz = fun.size_out(0);
            if sz(1)>0, check_argout(fun, 0, [inf 1]); end
        case 'bcs'
            check_name(fun, 'ocp_bcs');
            check_nargs(fun, 5+(na>=0), 1);
            check_argin(fun, 0, [nx 1]);
            check_argin(fun, 1, [nu 1]);
            check_argin(fun, 2, [nx 1]);
            check_argin(fun, 3, [nu 1]);
            check_argin(fun, 4, [np 1]);
            if na>=0
                check_argin(fun, 5, [na 1]);
            end
            sz = fun.size_out(0);
            if sz(1)>0, check_argout(fun, 0, [inf 1]); end
        case 'int' 
            check_name(fun, 'ocp_int');
            check_nargs(fun, 7+(na>=0), 1);
            check_argin(fun, 0, [1 1]);
            check_argin(fun, 1, [nx 1]);
            check_argin(fun, 2, [nu 1]);
            check_argin(fun, 3, [nx 1]);
            check_argin(fun, 4, [nu 1]);
            check_argin(fun, 5, [np 1]);
            check_argin(fun, 6, [1 1]);
            if na>=0
                check_argin(fun, 7, [na 1]);
            end
            sz = fun.size_out(0);
            if sz(1)>0, check_argout(fun, 0, [inf 1]); end
    end
end