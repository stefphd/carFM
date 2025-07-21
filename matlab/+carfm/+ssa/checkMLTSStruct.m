function checkMLTSStruct(mlts)
    % CHECKMLTSSTRUCT Check the contents of the MLTS structure
    % INPUT: 
    %
    % mlts: the track structure
    
    required_fields = {'s', 'V', 'alpha', 'geq', 'at0', 'rho'};
    mlts_fields = fieldnames(mlts);
    % Check required fields
    for k = 1 : numel(required_fields)
        % check if exists
        if ~any(strcmp(required_fields{k}, mlts_fields))
            eid = 'mltsfm:noSuchVariable';
            msg = ['Unrecognized field name "' required_fields{k} '".'];
            error(eid,msg)
        end
        % check type
        if ~isa(mlts.(required_fields{k}), 'double') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' required_fields{k} '".'];
            error(eid,msg)
        end
        % check size
        if numel(mlts.s) ~= numel(mlts.(required_fields{k}))
            eid = 'mltsfm:notEqual';
            msg = ['Field "' required_fields{k} '" has an inconsistent number of elements.'];
            error(eid,msg)
        end
    end
end