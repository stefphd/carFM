function sensitivityPar1 = expandSensitivityPar(sensitivityPar, bike)
% EXPANDSENSITIVITYPAR Expand the sensisitity parameter in the case of
% vector value, to give only scalar sensitivity parameters, e.g. a vector
% 'v' is expanded to 'v(1)', 'v(2)', 'v(3)', ...
% INPUT:
% sensitivityPar: names of sensitivity parameters
% bike: data structure
% OUTPUT:
% sensitivityPar1: expanded sensitivity parameters

% Loop all entries
sensitivityPar1 = {};
for ip = 1 : numel(sensitivityPar)
    field = sensitivityPar{ip};
    par = carfm.common.getField(bike, field);
    if isscalar(par)
        sensitivityPar1{end+1} = field; % keep scalar parameters
        continue;
    end
    % loop over the vector and add
    for j = 1 : numel(par)
        sensitivityPar1{end+1} = sprintf('%s(%d)', field, j);
    end
end