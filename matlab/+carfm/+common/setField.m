function s = setField(s, expr, value)
    % Convert string expression into subsasgn structure
    % IA-generated
    expr = ['.' expr]; 

    tokens = regexp(expr, ...
        '(\.\w+)|(\([\d,: ]+\))|(\{\d+\})', 'match');

    S = struct('type', {}, 'subs', {});
    for i = 1:numel(tokens)
        t = tokens{i};
        if startsWith(t, '.')
            S(end+1).type = '.';
            S(end).subs = t(2:end);
        elseif startsWith(t, '(')
            S(end+1).type = '()';
            S(end).subs = {eval(['[' t(2:end-1) ']'])};
        elseif startsWith(t, '{')
            S(end+1).type = '{}';
            S(end).subs = {str2double(t(2:end-1))};
        end
    end

    % Human modification: here I need to set possibly casadi values!
    % Need to promote the parent array before assignment
    Sparent = S(1:end-1);
    Slast = S(end);
    if ~isempty(Sparent)
        parent = subsref(s, Sparent);
        if ~isnumeric(value) && isnumeric(parent) % is casadi
            parent = casadi.MX(parent); % promote to casadi.MX
        end
        parent = subsasgn(parent, Slast, value); % write parent
    else
        Sparent = Slast;
        parent = value;
    end
    s = subsasgn(s, Sparent, parent); % write parent back
end