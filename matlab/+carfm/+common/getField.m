function value = getField(s, expr)
    % Convert string expression into subsref structure
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

    value = subsref(s, S);
end