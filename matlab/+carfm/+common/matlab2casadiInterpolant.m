function [data] = matlab2casadiInterpolant(data)
%MATLAB2CASADIINTERPOLANT This function converts matlab interpolants (pp-form, b-form, or
% griddedInterpolant to casadi.interpolant. If the input is an interpolant, it
%is converted to a casadi.interpolant. If the input is a struct, then the
%function looks for interpolants recursively in each field and
%convert them. If the input is another variable, it is returned as is.
%Only 1D griddedInterpolants are converted.
    data = gridInt2casadiInt_internal(data, 'none');
end

function data = gridInt2casadiInt_internal(data, name)
    switch class(data)
        case 'griddedInterpolant' %griddedInterpolant
            data = convert(data, class(data), name); % convert to casadi
        case 'struct'
            % detect pp-form or b-form by looking for 'form', 'order', and
            % 'dim' fields
            if isfield(data,'form') && isfield(data,'order') && isfield(data,'dim') % is pp-form or b-form
                error('carfm:invalidType', 'pp-form or B-form interpolants not supported: use griddedInterpolant instead')
            end
            % not a pp-form or b-form, look for interpolants in each field
            fields = fieldnames(data);
            for k = 1 : numel(fields)
                data.(fields{k}) = gridInt2casadiInt_internal(data.(fields{k}), fields{k});
            end
        otherwise %other variables
            %nothing: return data as is
            return
    end
end

function data = convert(data, form, name)
    import casadi.*
    % convert data to b-form
    % name = sprintf('%s_interp', name);
    switch form
        case 'griddedInterpolant'
            x = data.GridVectors;
            y = data.Values;
            method = data.Method;
            % switch method
            switch method
                case 'linear'
                    data = interpolant(name, 'linear', x, y(:));
                case 'spline'
                    data = interpolant(name, 'bspline', x, y(:));
                otherwise
                    warning('carfm:interpolantNotSupported', ['griddedInterpolant with method ''' method ''' not supported: using linear interpolant instead'])
                    data = interpolant(name, 'linear', x, y(:));
            end
        otherwise
            % sth wrong, return data as is
            return
    end
end