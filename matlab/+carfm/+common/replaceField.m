function [news, old_values] = replaceField(s, names, values)
%REPLACEFIELD Replace fields in struct and possible substructs
% Returns the modified struct and the old parameters

old_values = zeros(size(names)); % init
news = s; % init
% loop over parameters
for ip = 1 : numel(names)
    % get nominal
    % fields may contain "." and "(...)"
    old_values(ip) = carfm.common.getField(s, names{ip});
    news = carfm.common.setField(news, names{ip}, values(ip));
end

end