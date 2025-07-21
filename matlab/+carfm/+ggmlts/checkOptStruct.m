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
            case 'g'
                if ~isequal(size(opts.g),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "g" must be [1,1].';
                    error(eid,msg)
                end
                if opts.g <= 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "g" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wt'
                if ~isequal(size(opts.wt),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wt" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wt < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wt" must be greater than 0.';
                    error(eid,msg)
                end
            case 'wn'
                if ~isequal(size(opts.wn),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wn" must be [1,1].';
                    error(eid,msg)
                end
                if opts.wn < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "wn" must be greater than 0.';
                    error(eid,msg)
                end
            case 'maxLongJerk'
                if ~isequal(size(opts.maxLongJerk),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "maxLongJerk" must be [1,1].';
                    error(eid,msg)
                end
                if opts.maxLongJerk < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "maxLongJerk" must be greater than 0.';
                    error(eid,msg)
                end
            case 'minLongJerk'
                if ~isequal(size(opts.minLongJerk),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "minLongJerk" must be [1,1].';
                    error(eid,msg)
                end
                if opts.minLongJerk > 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "minLongJerk" must be greater than 0.';
                    error(eid,msg)
                end
            case 'maxLatJerk'
                if ~isequal(size(opts.maxLatJerk),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "maxLatJerk" must be [1,1].';
                    error(eid,msg)
                end
                if opts.maxLatJerk < 0
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "maxLatJerk" must be greater than 0.';
                    error(eid,msg)
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
                if ~isequal(size(opts.xscale),[1,5])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "xscale" must be [1,5].';
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
            case 'fscale'
                if ~isequal(size(opts.fscale),[1,5])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "fscale" must be [1,5].';
                    error(eid,msg)
                end
                if any(opts.fscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "fscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'cscale'
                if ~isequal(size(opts.cscale),[1,4])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "cscale" must be [1,4].';
                    error(eid,msg)
                end
                if any(opts.cscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "cscale" must be greater than 0.';
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
                if nargin(opts.bcsFunc) ~= 2 ||  abs(nargout(opts.bcsFunc)) ~= 1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "bcsFunc" must be a function handle with two input arguments and  one output argument.';
                    error(eid,msg);
                end
                try
                    bcs = opts.bcsFunc(ones(5,1), ones(5,1));
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