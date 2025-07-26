function checkOptStruct(opts)
    % CHECKOPTSTRUCT Check the contents of the option structure
    % INPUT: 
    % opts: the option structure

    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        switch opts_fields{k}
            case 'problemName'
                if ~ischar(opts.problemName) || ~(size(opts.problemName,1)==1)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "problemName" must be a char array.';
                    error(eid,msg)
                end
                if contains(opts.problemName, carfm.common.invalidChars)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "problemName" contains illegal characters.';
                    error(eid,msg)
                end
            case 'iTyreSide'
                if ~isequal(size(opts.iTyreSide),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iTyreSide" must be [1,1].';
                    error(eid,msg);
                end
                if ~all((opts.iTyreSide == 1) | (opts.iTyreSide == -1))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iTyreSide" must be either -1 or 1.';
                    error(eid,msg);
                end
            case 'iTyreType'
                if ~isequal(size(opts.iTyreType),[1,3])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iTyreType" must be [1,3].';
                    error(eid,msg);
                end
                if ~all((opts.iTyreType == 1) | (opts.iTyreType == 0))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iTyreType" must be either 0 or 1.';
                    error(eid,msg);
                end
            case 'iSuspensionSide'
                if ~isequal(size(opts.iSuspensionSide),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iSuspensionSide" must be [1,1].';
                    error(eid,msg);
                end
                if ~all((opts.iSuspensionSide == 1) | (opts.iSuspensionSide == -1))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iSuspensionSide" must be either -1 or 1.';
                    error(eid,msg);
                end
            case 'iSuspensionType'
                if ~isequal(size(opts.iSuspensionType),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iSuspensionType" must be [1,1].';
                    error(eid,msg);
                end
                if ~all((opts.iSuspensionType == 1) | (opts.iSuspensionType == 0))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iSuspensionType" must be either 0 or 1.';
                    error(eid,msg);
                end
            case 'iEngineBrake'
                if ~isequal(size(opts.iEngineBrake),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iEngineBrake" must be [1,1].';
                    error(eid,msg);
                end
                if ~all((opts.iEngineBrake == 1) | (opts.iEngineBrake == 0))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iEngineBrake" must be either 0 or 1.';
                    error(eid,msg);
                end
            case 'iRotaryInertia'
                if ~isequal(size(opts.iRotaryInertia),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iRotaryInertia" must be [1,1].';
                    error(eid,msg);
                end
                if ~all((opts.iRotaryInertia == 1) | (opts.iRotaryInertia == 0))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iRotaryInertia" must be either 0 or 1.';
                    error(eid,msg);
                end
            case 'iAeroComps'
                if ~isequal(size(opts.iAeroComps),[1,6])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iAeroComps" must be [1,6].';
                    error(eid,msg);
                end
                if ~all((opts.iAeroComps == 1) | (opts.iAeroComps == 0))
                    eid = 'carfm:incorrectType';
                    msg = 'Value of field "iAeroComps" must be either 0 or 1.';
                    error(eid,msg);
                end
            case 'suspTrimRigid'
                if ~isequal(size(opts.suspTrimRigid),[1,4])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "suspTrimRigid" must be [1,4].';
                    error(eid,msg);
                end
                if (opts.suspTrimRigid(4) < 0) || (opts.suspTrimRigid(4) > 1)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field suspTrimRigid(4) must be between 0 and 1';
                    error(eid, msg);
                end
            case 'tyreDeformationRigid'
                if ~isequal(size(opts.tyreDeformationRigid),[1,4])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "tyreDeformationRigid" must be [1,4].';
                    error(eid,msg);
                end
            case 'Neps'
                if ~isequal(size(opts.Neps),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "Neps" must be [1,1].';
                    error(eid,msg);
                end
                if opts.Neps<=0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "Neps" must be strictly greater than 0.';
                    error(eid,msg);
                end
            case 'Taueps'
                if ~isequal(size(opts.Taueps),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "Taueps" must be [1,1].';
                    error(eid,msg);
                end
                if opts.Taueps<=0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "Taueps" must be strictly greater than 0.';
                    error(eid,msg);
                end
            case 'extForce'
                if ~isequal(size(opts.extForce),[1,3])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "extForce" must be [1,3].';
                    error(eid,msg);
                end
            case 'extTorque'
                if ~isequal(size(opts.extTorque),[1,3])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "extTorque" must be [1,3].';
                    error(eid,msg);
                end
            case 'rtol'
                if ~isequal(size(opts.rtol),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "rtol" must be [1,1].';
                    error(eid,msg)
                end
                if opts.rtol<=0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "rtol" must be strictly greater than 0.';
                    error(eid,msg);
                end
            case 'sscale'
                if ~isequal(size(opts.sscale),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "sscale" must be [1,1].';
                    error(eid,msg)
                end
                if opts.sscale <= 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "sscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'lscale'
                if ~isequal(size(opts.lscale),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "lscale" must be [1,1].';
                    error(eid,msg)
                end
                if opts.lscale <= 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "lscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'xscale'
                if ~isequal(size(opts.xscale),[1,25])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "xscale" must be [1,25].';
                    error(eid,msg)
                end
                if any(opts.xscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "xscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'uscale'
                if ~isequal(size(opts.uscale),[1,2])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "uscale" must be [1,2].';
                    error(eid,msg)
                end
                if any(opts.uscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "uscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'upscale'
                if ~isequal(size(opts.upscale),[1,2])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "upscale" must be [1,2].';
                    error(eid,msg)
                end
                if any(opts.upscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "upscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'rscale'
                if ~isequal(size(opts.rscale),[1,23])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "rscale" must be [1,23].';
                    error(eid,msg)
                end
                if any(opts.rscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "rscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'fscale'
                if ~isequal(size(opts.fscale),[1,27])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "fscale" must be [1,27].';
                    error(eid,msg)
                end
                if any(opts.fscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "fscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'cscale'
                if ~isequal(size(opts.cscale),[1,7])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "cscale" must be [1,7].';
                    error(eid,msg)
                end
                if any(opts.cscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "cscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wuDelta'
                if ~isequal(size(opts.wuDelta),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wuDelta" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wuDelta < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wuDelta" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wuTaut'
                if ~isequal(size(opts.wuTaut),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wuTaut" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wuTaut < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wuTaut" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wdeltadot'
                if ~isequal(size(opts.wdeltadot),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wdeltadot" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wdeltadot < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wdeltadot" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wdeltaddot'
                if ~isequal(size(opts.wdeltaddot),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wdeltaddot" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wdeltaddot < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wdeltaddot" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wlambda'
                if ~isequal(size(opts.wlambda),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wlambda" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wlambda < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wlambda" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wlambdadot'
                if ~isequal(size(opts.wlambdadot),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wlambdadot" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wlambdadot < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wlambdadot" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wkappa'
                if ~isequal(size(opts.wkappa),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wkappa" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wkappa < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wkappa" must be greater than 0.';
                    error(eid,msg)
                end
            case 'mex'
                if ~isequal(size(opts.mex),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "mex" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.mex)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "mex" must be logical.';
                    error(eid,msg);
                end
            case 'usePrebuilt'
                if ~isequal(size(opts.usePrebuilt),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "usePrebuilt" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.usePrebuilt)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "usePrebuilt" must be logical.';
                    error(eid,msg);
                end
            case 'buildOnly'
                if ~isequal(size(opts.buildOnly),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "buildOnly" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.buildOnly)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "buildOnly" must be logical.';
                    error(eid,msg);
                end
            case 'debugSolve'
                if ~isequal(size(opts.debugSolve),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "debugSolve" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.debugSolve)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "debugSolve" must be logical.';
                    error(eid,msg);
                end
            case 'exactHessian'
                if ~isequal(size(opts.exactHessian),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "exactHessian" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.exactHessian)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "exactHessian" must be logical.';
                    error(eid,msg);
                end
            case 'maxIter'
                if ~isequal(size(opts.maxIter),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "maxIter" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.maxIter, 1) ~= 0 || opts.maxIter<0
                    eid = 'carfm:incorrectType';
                    msg = 'Field "maxIter" must be a non-negative integer number.';
                    error(eid,msg);
                end
            case {'tol','conTol','optTol','complTol','tolAccept','conTolAccept', ...
                    'optTolAccept','complTolAccept','objChangeAccept'}
                if ~isequal(size(opts.(opts_fields{k})),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "%s" must be [1,1].';
                    error(eid,msg,opts_fields{k})
                end
                if opts.(opts_fields{k}) < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "%s" must be greater than 0.';
                    error(eid,msg,opts_fields{k})
                end
            case 'iterAccept'
                if ~isequal(size(opts.iterAccept),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "iterAccept" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.iterAccept, 1) ~= 0 || opts.iterAccept<1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "iterAccept" must be a positive integer number.';
                    error(eid,msg);
                end
            case 'printInt'
                if ~isequal(size(opts.printInt),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "printInt" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.printInt, 1) ~= 0 || opts.printInt<1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "printInt" must be a positive integer number.';
                    error(eid,msg);
                end
            case 'gginterpMethod'
                if ~any(strcmp(opts.gginterpMethod,{'linear','bspline'}))
                    eid = 'carfm:incorrectType';
                    msg = 'Field "gginterpMethod" must be either ''linear'' or ''bspline''.';
                    error(eid,msg);
                end
            case 'trackinterpMethod'
                if ~any(strcmp(opts.trackinterpMethod,{'linear','bspline'}))
                    eid = 'carfm:incorrectType';
                    msg = 'Field "trackinterpMethod" must be either ''linear'' or ''bspline''.';
                    error(eid,msg);
                end
            case 'numMeshPts'
                if ~isequal(size(opts.numMeshPts),[1 1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "numMeshPts" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.numMeshPts, 1)~= 0 || opts.numMeshPts<1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "numMeshPts" must be a positive integer number.';
                    error(eid,msg);
                end
            case 'meshStrategy'
                if ~any(strcmp(opts.meshStrategy,{'equally-spaced','adaptive','manual'}))
                    eid = 'carfm:incorrectType';
                    msg = 'Field "meshStrategy" must be either ''equally-spaced'',''adaptive'', or ''manual''';
                    error(eid,msg);
                end
            case 'meshFractions'
                if ~isequal(size(opts.meshFractions,1),1)
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "meshFractions" must be [1,numMeshPts-1].';
                    error(eid,msg);
                end
                if ~isnumeric(opts.meshFractions)
                    eid = 'carfm:incorrectType';
                    msg = 'Field "meshFractions" must be numeric.';
                    error(eid,msg)
                end
                if any(opts.meshFractions<=0)
                    eid = 'carfm:incorrectType';
                    msg = 'Values of field "meshFractions" must be greater than .';
                    error(eid,msg)
                end
            case 'meshRatio'
                if ~isequal(size(opts.meshRatio),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "meshRatio" must be [1,1].';
                    error(eid,msg);
                end
                if opts.meshRatio<1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "meshRatio" must be a number greater than 1.';
                    error(eid,msg);
                end
            case 'meshMinSecLen'
                if ~isequal(size(opts.meshMinSecLen),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "meshMinSecLen" must be [1,1].';
                    error(eid,msg);
                end
                if opts.meshMinSecLen<5
                    eid = 'carfm:incorrectType';
                    msg = 'Field "meshMinSecLen" must be a number greater than 5.';
                    error(eid,msg);
                end
            case 'meshTransLen'
                if ~isequal(size(opts.meshTransLen),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "meshTransLen" must be [1,1].';
                    error(eid,msg);
                end
                if opts.meshTransLen<5
                    eid = 'carfm:incorrectType';
                    msg = 'Field "meshTransLen" must be a number greater than 5.';
                    error(eid,msg);
                end
            case 'meshThFactor'
                if ~isequal(size(opts.meshThFactor),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "meshThFactor" must be [1,1].';
                    error(eid,msg);
                end
                if opts.meshThFactor<=0
                    eid = 'carfm:incorrectType';
                    msg = 'Field "meshThFactor" must be a number greater than 0.';
                    error(eid,msg);
                end    
            case 'scheme'
                if ~any(strcmp(opts.scheme,{'euler','trapz','midpoint'})) && isempty(regexp(opts.scheme, '^lgr[2-8]$', 'once'))
                    eid = 'carfm:incorrectType';
                    msg = 'Field "scheme" must be either ''euler'', ''trapz'', ''midpoint'' or ''lgr<N>'' (N=2,..,8).';
                    error(eid,msg);
                end
            case 'init_iter'
                if ~isequal(size(opts.init_iter),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "init_iter" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.init_iter, 1) ~= 0 || opts.init_iter<0
                    eid = 'carfm:incorrectType';
                    msg = 'Field "init_iter" must be a non-negative integer number.';
                    error(eid,msg);
                end
            case 'sRange'
                if ~isequal(size(opts.sRange),[1,2])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "sRange" must be [1,2].';
                    error(eid,msg)
                end
                if ~isnumeric(opts.sRange)
                    eid = 'carfm:incorrectType';
                    msg = 'Field "sRange" must be numeric.';
                    error(eid,msg)
                end
                if opts.sRange(1) >= opts.sRange(end)
                    eid = 'carfm:incorrectType';
                    msg = '"sRange(2)" must be greater than sRange(1).';
                    error(eid,msg)
                end
            case 'bcsFunc'
                if ~isa(opts.bcsFunc, 'function_handle')
                    eid = 'carfm:incorrectType';
                    msg = 'Field "bcsFunc" must be a function handle.';
                    error(eid,msg);
                end
                if nargin(opts.bcsFunc) ~= 4 ||  abs(nargout(opts.bcsFunc)) ~= 1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "bcsFunc" must be a function handle with four input arguments and  one output argument.';
                    error(eid,msg);
                end
                try
                    bcs = opts.bcsFunc(ones(21,1), ones(3,1), ones(21,1), ones(3,1));
                    validateattributes( bcs, { 'numeric' }, { 'column' });
                catch e
                    eid = 'carfm:unableEval';
                    error(eid, 'Unable to evaluate "bcsFunc" function handle: %s', e.message)
                end
            case 'bcsRelax'
                if ~isequal(size(opts.bcsRelax),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "bcsRelax" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.bcsRelax)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "bcsRelax" must be logical.';
                    error(eid,msg);
                end
            case 'numThreads'
                if ~isequal(size(opts.numThreads),[1 1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "numThreads" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.numThreads, 1)~= 0 || opts.numThreads<1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "numThreads" must be a positive integer number.';
                    error(eid,msg);
                end
            case 'linearSolver'
                if ~any(strcmp(opts.linearSolver,{'mumps', 'ma27'}))
                    eid = 'carfm:incorrectType';
                    msg = 'Field "linearSolver" must be either ''mumps'' or ''ma27''.';
                    error(eid,msg);
                end
        end
    end
end