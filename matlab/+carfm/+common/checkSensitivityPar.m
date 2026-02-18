function [flag, errid, errmsg] = checkSensitivityPar(sensitivityPar, car)
%CHECKSENSITIVITYPAR Check sensitivity parameters

% init no err
flag = false; 
errid = '';
errmsg = '';

% Check all entries
for ip = 1 : numel(sensitivityPar)
    field = sensitivityPar{ip};
    try
        % fields may contain "." and "(...)"
        par = carfm.common.getField(car, field);
        % parameter must be numeric scalar
        if ~isvector(par) || ~isnumeric(par)
            % error
            flag = true;
            errid = 'carfm:invalidType';
            errmsg = ['Cannot compute sensitivity for car parameter "' field '": value must be a numeric vector'];
            return;
        end
    catch e
        % error
        flag = true;
        errid = 'carfm:notFound';
        errmsg = ['Cannot compute sensitivity for car parameter "' field '": value not found'];
        return;
    end
end

end