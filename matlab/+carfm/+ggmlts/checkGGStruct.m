function checkGGStruct(gg)
    % CHECKGGSTRUCT Check the contents of the g-g structure
    % INPUT: 
    % gg: the g-g structure (possibly vector)
    
    % Loop over gg strcut vector
    for l = 1 : numel(gg)
        % Add optional field g
        gg_fields = fieldnames(gg(l));
        if ~any(strcmp('g', gg_fields))
            gg(l).g = 1; % set to 1 by default
        end
        % Check fields
        required_fields = {'alpha', 'V', 'rho', 'g', 'shift', 'x'};
        type_fields = {'double', 'double', 'double', 'double', 'function_handle', 'double'};
        for k = 1 : numel(required_fields)
            % check if exists
            if ~any(strcmp(required_fields{k}, gg_fields))
                eid = 'carfm:noSuchVariable';
                msg = ['Unrecognized field name "' required_fields{k} '".'];
                error(eid,msg)
            end
            % check type
            if ~isa(gg(l).(required_fields{k}), type_fields{k}) 
                eid = 'carfm:incorrectType';
                msg = ['Incorrect type of field "' required_fields{k} '".'];
                error(eid,msg)
            end
        end
        % check alpha
        if size(gg(l).alpha,1)~=1 || size(gg(l).alpha,2)<2
            eid = 'carfm:notEqual';
            msg = 'Size of field "alpha" must be [1,:].';
            error(eid,msg)
        end
        try
            validateattributes( gg(l).alpha, { 'numeric' }, { 'vector', 'increasing' } )
            % if ~isuniform( gg(l).alpha )
            %     error('Expected input must be equally spaced.')
            % end
        catch e
            eid = 'carfm:incorrectType';
            error(eid,'Invalid field "alpha". %s', e.message);
        end
        if ~(ismembertol(gg(l).alpha(1), -pi/2) || ismembertol(gg(l).alpha(1), -pi)) || ~(ismembertol(gg(l).alpha(end), pi/2) || ismembertol(gg(l).alpha(end), pi))
            eid = 'carfm:notEqual';
            msg = 'Field "alpha" must start at -pi/2 and end at +pi/2.';
            error(eid,msg)
        end
        % check V
        if size(gg(l).V,1)~=1 || size(gg(l).V,2)<2
            eid = 'carfm:notEqual';
            msg = 'Size of field "V" must be [1,:].';
            error(eid,msg)
        end
        try
            validateattributes( gg(l).V, { 'numeric' }, { 'vector', 'increasing' } )
            % if ~isuniform( gg(l).V )
            %     error('Expected input must be equally spaced.')
            % end
        catch e
            eid = 'carfm:incorrectType';
            error(eid,'Invalid field "V". %s', e.message);
        end
        % check g
        if size(gg(l).g,1)~=1
            eid = 'carfm:notEqual';
            msg = 'Size of field "g" must be [1,:]';
            error(eid,msg)
        end
        if (size(gg(l).g,2)~=1 && size(gg(l).g,2)<4)
            eid = 'carfm:notEqual';
            msg = 'Number of elements of field "g" must be 1 or greater than 4';
            error(eid,msg)
        end
        try
            validateattributes( gg(l).g, { 'numeric' }, { 'vector', 'increasing' } )
            % if ~isuniform( gg(l).g )
            %     error('Expected input must be equally spaced.')
            % end
        catch e
            eid = 'carfm:incorrectType';
            error(eid,'Invalid field "g". %s', e.message);
        end
        % check rho
        if size(gg(l).rho,1)~=size(gg(l).alpha,2) || size(gg(l).rho,2)~=size(gg(l).V,2) || size(gg(l).rho,3)~=size(gg(l).g,2)
            eid = 'carfm:notEqual';
            msg = 'Size of field "rho" must be [length(alpha),length(V),length(g)].';
            error(eid,msg)
        end
        % check x
        if size(gg(l).x,1)~=size(gg(l).alpha,2) || size(gg(l).x,2)~=size(gg(l).V,2) || size(gg(l).x,3)~=size(gg(l).g,2)
            eid = 'carfm:notEqual';
            msg = 'Size of field "x" must be [length(alpha),length(V),length(g)].';
            error(eid,msg)
        end
        % check shift
        try
            gg(l).shift(10);
        catch ME
            eid = 'carfm:unableEval';
            msg = 'Unable to evaluate g-g shift.';
            err = MException(eid, msg);   
            throw(err);
        end
    end
end