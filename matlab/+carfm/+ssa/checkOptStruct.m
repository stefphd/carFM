function checkOptStruct(opts)
    % CHECKOPTSTRUCT Check the contents of the option structure
    % INPUT: 
    % opts: the option structure

    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        switch opts_fields{k}
            case 'xscale'
                if ~isequal(size(opts.xscale),[1,33])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "xscale" must be [1,33].';
                    error(eid,msg)
                end
                if any(opts.xscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "xscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'xsign'
                if ~isequal(size(opts.xsign),[1,33])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "xsign" must be [1,33].';
                    error(eid,msg)
                end
                if any(abs(opts.xsign) ~= 1)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "xsign" must be either +1 or -1.';
                    error(eid,msg)
                end
            case 'xswap'
                if ~isequal(size(opts.xswap, 2),2)
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "xswap" must be [:,2].';
                    error(eid,msg)
                end
                if (min(opts.xswap) < 1) || (min(opts.xswap) > 33)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "xswap" must be greater than 1 and lower than 33.';
                    error(eid,msg)
                end
            case 'rscale'
                if ~isequal(size(opts.rscale),[1,36])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "rscale" must be [1,36].';
                    error(eid,msg)
                end
                if any(opts.rscale <= 0)
                    eid = 'carfm:invalidValue';
                    msg = 'Values of field "rscale" must be greater than 0.';
                    error(eid,msg)
                end
            case 'ssActiveLongInputs'
                if ~isequal(size(opts.ssActiveLongInputs),[1,3])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "ssActiveLongInputs" must be [1,3].';
                    error(eid,msg);
                end
                if ~islogical(opts.ssActiveLongInputs)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "ssActiveLongInputs" must be logical.';
                    error(eid,msg);
                end
                if sum(opts.ssActiveLongInputs) ~= 2
                    eid = 'carfm:incorrectValue';
                    msg = 'Number of active longitudinal inputs in field "ssActiveLongInputs" must be 2 (found %d).';
                    error(eid,msg,sum(opts.ssActiveLongInputs));
                end
                if ~isequal(opts.ssActiveLongInputs, [true true false]) && ...
                   ~isequal(opts.ssActiveLongInputs, [true false true])
                    eid = 'carfm:incorrectValue';
                    msg = 'Invalid active longitudinal inputs in field "ssActiveLongInputs".\nValid selections are [true true false] and [true false true].';
                    error(eid,msg);
                end
            case 'ssActiveLatInputs'
                if ~isequal(size(opts.ssActiveLatInputs),[1,3])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "ssActiveLatInputs" must be [1,3].';
                    error(eid,msg);
                end
                if ~islogical(opts.ssActiveLatInputs)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "ssActiveLatInputs" must be logical.';
                    error(eid,msg);
                end
                if sum(opts.ssActiveLatInputs) ~= 1
                    eid = 'carfm:incorrectValue';
                    msg = 'Number of active lateral inputs in field "ssActiveLatInputs" must be 1 (found %d).';
                    error(eid,msg,sum(opts.ssActiveLatInputs));
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
            case 'ssaMexName'
                if ~ischar(opts.ssaMexName) || ~(size(opts.ssaMexName,1)==1)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "ssaMexName" must be a char array.';
                    error(eid,msg)
                end
                if contains(opts.ssaMexName, carfm.common.invalidChars)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "ssaMexName" contains illegal characters.';
                    error(eid,msg)
                end
            case 'ggMexName'
                if ~ischar(opts.ggMexName) || ~(size(opts.ggMexName,1)==1)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "ggMexName" must be a char array.';
                    error(eid,msg)
                end
                if contains(opts.ggMexName, carfm.common.invalidChars)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "ggMexName" contains illegal characters.';
                    error(eid,msg)
                end
            case 'useLastSSA'
                if ~isequal(size(opts.useLastSSA),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "useLastSSA" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.useLastSSA)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "useLastSSA" must be logical.';
                    error(eid,msg);
                end
            case 'gz_g'
                if ~isequal(size(opts.gz_g),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "gz_g" must be [1,1].';
                    error(eid,msg)
                end
                if opts.gz_g<=0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "gz_g" must be strictly greater than 0.';
                    error(eid,msg);
                end
            case 'ssSolver'
                if ~any(strcmp(opts.ssSolver,{'fast_newton','newton'}))
                    eid = 'carfm:invalidType';
                    msg = 'Field "ssSolver" must be either ''fast_newton'' or ''newton''.';
                    error(eid,msg);
                end
            case 'gearSharpness'
                if ~isequal(size(opts.gearSharpness),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "gearSharpness" must be [1,1].';
                    error(eid,msg)
                end
                if opts.gearSharpness<=0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "gearSharpness" must be strictly greater than 0.';
                    error(eid,msg);
                end
            case 'numGGpts'
                if ~isequal(size(opts.numGGpts),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "numGGpts" must be [1,1].';
                    error(eid,msg);
                end
                if rem(opts.numGGpts, 1) ~= 0 || opts.numGGpts<1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "numGGpts" must be a positive integer number.';
                    error(eid,msg);
                end
            case 'isSymGG'
                if ~isequal(size(opts.isSymGG),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "isSymGG" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.isSymGG)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "isSymGG" must be logical.';
                    error(eid,msg);
                end
            case 'GGshift'
                if ~isa(opts.GGshift, 'function_handle')
                    eid = 'carfm:incorrectType';
                    msg = 'Field "GGshift" must be a function handle.';
                    error(eid,msg);
                end
                if nargin(opts.GGshift) ~= 1 ||  abs(nargout(opts.GGshift)) < 1
                    eid = 'carfm:incorrectType';
                    msg = 'Field "GGshift" must be a function handle with one input argument and at least one output argument.';
                    error(eid,msg);
                end
                try
                    ax0 = opts.GGshift(10);
                catch e
                    eid = 'carfm:unableEval';
                    error(eid, 'Unable to evaluate "GGshift" function handle: %s', e.message)
                end
                if ~isnumeric(ax0)
                    eid = 'carfm:incorrectType';
                    msg = 'The output argument of the "GGshift" function handle must be numeric.';
                    error(eid,msg);
                end
            case 'algorithmGGopt'
                if ~any(strcmp(opts.algorithmGGopt,{'interior-point','sqp'}))
                    eid = 'carfm:invalidType';
                    msg = 'Field "algorithmGGopt" must be either ''interior-point'' or ''sqp''.';
                    error(eid,msg);
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
            case 'fillFailed'
                if ~isequal(size(opts.fillFailed),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "fillFailed" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.fillFailed)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "fillFailed" must be logical.';
                    error(eid,msg);
                end
            case 'kappaLim'
                if ~isequal(size(opts.kappaLim),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "kappaLim" must be [1,1].';
                    error(eid,msg);
                end
                if opts.kappaLim<=0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "kappaLim" must be strictly greater than 0.';
                    error(eid,msg);
                end
            case 'wlambda'
                if ~isequal(size(opts.wlambda),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wlambda" must be [1,1].';
                    error(eid,msg);
                end
                if opts.wlambda<0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "wlambda" must be greater or equal than 0.';
                    error(eid,msg);
                end
            case 'wdelta'
                if ~isequal(size(opts.wdelta),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wdelta" must be [1,1].';
                    error(eid,msg);
                end
                if opts.wdelta<0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "wdelta" must be greater or equal than 0.';
                    error(eid,msg);
                end
            case 'wkappa'
                if ~isequal(size(opts.wkappa),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "wkappa" must be [1,1].';
                    error(eid,msg);
                end
                if opts.wkappa<0
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "wkappa" must be greater or equal than 0.';
                    error(eid,msg);
                end
            case 'ggmlts2ssMexName'
                if ~ischar(opts.ggmlts2ssMexName) || ~(size(opts.ggmlts2ssMexName,1)==1)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "ggmlts2ssMexName" must be a char array.';
                    error(eid,msg)
                end
                if contains(opts.ggmlts2ssMexName, carfm.common.invalidChars)
                    eid = 'carfm:invalidValue';
                    msg = 'Value of field "ggmlts2ssMexName" contains illegal characters.';
                    error(eid,msg)
                end
            case 'GGRadiusTol'
                if ~isequal(size(opts.GGRadiusTol),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "GGRadiusTol" must be [1,1].';
                    error(eid,msg);
                end
                if opts.GGRadiusTol<=0 || opts.GGRadiusTol>=1
                    eid = 'carfm:incorrectValue';
                    msg = 'Value of field "GGRadiusTol" must be strictly within 0 and 1.';
                    error(eid,msg);
                end
            case 'refineMLTS'
                if ~isequal(size(opts.refineMLTS),[1,1])
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "refineMLTS" must be [1,1].';
                    error(eid,msg);
                end
                if ~islogical(opts.refineMLTS)
                    eid = 'carfm:incorrectType';
                    msg = 'Type of field "refineMLTS" must be logical.';
                    error(eid,msg);
                end
            case 'refineRange'
                if ~isequal(size(opts.refineRange,2),2)
                    eid = 'carfm:notEqual';
                    msg = 'Size of field "refineRange" must be [:,1].';
                    error(eid,msg);
                end
                if ~isnumeric(opts.refineRange)
                    eid = 'carfm:incorrectType';
                    msg = 'Field "refineRange" must be numeric.';
                    error(eid,msg)
                end
                if any(opts.refineRange(:,1) >= opts.refineRange(:,end))
                    eid = 'carfm:incorrectType';
                    msg = '"refineRange(:,2)" must be greater than refineRange(:,1).';
                    error(eid,msg)
                end
        end
    end
end