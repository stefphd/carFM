function checkGuessStruct(guess,fields)
%CHECKGUESSSTRUCT Check the guess structure
    % Do not check empty struct
    if isempty(fieldnames(guess))
        return;
    end
    % Check fields
    for k = 1 : numel(fields)
        % check if exists
        if ~isfield(guess, fields{k})
            eid = 'mltsfm:noSuchVariable';
            msg = ['Unrecognized field name "' fields{k} '".'];
            error(eid,msg)
        end
        % check size
        if numel(guess.(fields{1})) ~= numel(guess.(fields{k}))
            eid = 'mltsfm:notEqual';
            msg = ['Field "' fields{k} '" has an inconsistent number of elements.'];
            error(eid,msg)
        end
        % check type
        if strcmp(fields{k}, 'data') && ~isa(guess.(fields{k}), 'struct') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' fields{k} '".'];
            error(eid,msg)
        elseif ~strcmp(fields{k}, 'data') && ~isa(guess.(fields{k}), 'double') 
            eid = 'mltsfm:incorrectType';
            msg = ['Incorrect type of field "' fields{k} '".'];
            error(eid,msg)
        end
    end
end

