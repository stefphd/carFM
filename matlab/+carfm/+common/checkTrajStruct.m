function traj = checkTrajStruct(traj)
    % CHECKTRAJSTRUCT Check the contents of the trajectory structure
    % INPUT: 
    %
    % track: the traj structure

    % eventually use elap_dist for zeta
    if ~isfield(traj, 'zeta') && isfield(traj, 'elap_dist')
        traj.zeta = traj.elap_dist;
    end
    
    required_fields = {'zeta', 'x', 'y', 'psi', 'Gammaz'};
    optional_fields = {'z', 'sigma', 'beta', 'Gammax', 'Gammay', 'rha'};
    track_fields = fieldnames(traj);
    % Check required fields
    for k = 1 : numel(required_fields)
        % check if exists
        if ~any(strcmp(required_fields{k}, track_fields))
            eid = 'mltsfm:noSuchVariable';
            msg = ['Unrecognized field name "' required_fields{k} '".'];
            error(eid,msg)
        end
        % check type
        if ~isa(traj.(required_fields{k}), 'double') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' required_fields{k} '".'];
            error(eid,msg)
        end
        % check size
        if numel(traj.zeta) ~= numel(traj.(required_fields{k}))
            eid = 'mltsfm:notEqual';
            msg = ['Field "' required_fields{k} '" has an inconsistent number of elements.'];
            error(eid,msg)
        end
    end
    % Check optional fields
    for k = 1 : numel(optional_fields)
        % check if exists
        if ~any(strcmp(optional_fields{k}, track_fields))
            continue; % skip to next optional fields (check type and size not necessary)
        end
        % check type
        if ~isa(traj.(optional_fields{k}), 'double') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' optional_fields{k} '".'];
            error(eid,msg)
        end
        % check size
        if numel(traj.zeta) ~= numel(traj.(optional_fields{k}))
            eid = 'mltsfm:notEqual';
            msg = ['Field "' optional_fields{k} '" has an inconsistent number of elements.'];
            error(eid,msg)
        end
    end
    % Check for monotone zeta
    try
        validateattributes(traj.zeta, { 'numeric' }, { 'vector', 'increasing' } )
        % if ~isuniform(traj.zeta )
        %     error('Expected input must be equally spaced.')
        % end
    catch e
        eid = 'mltsfm:incorrectType';
        error(eid,'Invalid field "zeta". %s', e.message);
    end
end