function checkTrackStruct(track)
    % CHECKTRACKSTRUCT Check the contents of the track structure
    % INPUT: 
    %
    % track: the track structure
    
    required_fields = {'s', 'rwr', 'rwl', 'x', 'y', 'theta', 'Omegaz', 'xl', 'yl', 'xr', 'yr'};
    optional_fields = {'z', 'zl', 'zr', 'mu', 'phi', 'Omegax', 'Omegay'};
    track_fields = fieldnames(track);
    % Check required fields
    for k = 1 : numel(required_fields)
        % check if exists
        if ~any(strcmp(required_fields{k}, track_fields))
            eid = 'mltsfm:noSuchVariable';
            msg = ['Unrecognized field name "' required_fields{k} '".'];
            error(eid,msg)
        end
        % check type
        if ~isa(track.(required_fields{k}), 'double') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' required_fields{k} '".'];
            error(eid,msg)
        end
        % check size
        if numel(track.s) ~= numel(track.(required_fields{k}))
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
        if ~isa(track.(optional_fields{k}), 'double') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' optional_fields{k} '".'];
            error(eid,msg)
        end
        % check size
        if numel(track.s) ~= numel(track.(optional_fields{k}))
            eid = 'mltsfm:notEqual';
            msg = ['Field "' optional_fields{k} '" has an inconsistent number of elements.'];
            error(eid,msg)
        end
    end
    % Check for monotone s
    try
        validateattributes( track.s, { 'numeric' }, { 'vector', 'increasing' } )
        % if ~isuniform(track.s )
        %     error('Expected input must be equally spaced.')
        % end
    catch e
        eid = 'mltsfm:incorrectType';
        error(eid,'Invalid field "s". %s', e.message);
    end
end