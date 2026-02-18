function checkInterpolantType(data, types)
%CHECKINTERPOLANTTYPE Check interpolant types and give warning
    checkInterpolantType_internal(data, types, '');
end

function checkInterpolantType_internal(data, types, str)
    switch class(data)
    case 'griddedInterpolant' %griddedInterpolant
        method = data.Method;
        if ~any(strcmp(method, types))
            fprintf('Warning: interpolant "%s" has method "%s", which is C0.\n', str, method)
        end
    case 'struct'
        % look for interpolants in each field
        fields = fieldnames(data);
        for k = 1 : numel(fields)
            if ~isempty(str)
                str1 = [str '.' fields{k}];
            else
                str1 = [str fields{k}];
            end
            checkInterpolantType_internal(data.(fields{k}), types, str1);
        end
    otherwise %other variables
        %nothing: return data as is
        return
    end
end
