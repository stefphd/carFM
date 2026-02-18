function [grid_min,grid_max,flag,errid,errmsg] = getGGBoundGrid(gg)
%GETGGBOUNDGRID 

% Find GG grid bounds
grid_fields = {'V','g'};
% hardcoded limits for singleton dimensions
grid_min = [1,-1];
grid_max = [150,3];
for l = 1 : numel(gg)
    for k = 1 : numel(grid_fields)
        field = grid_fields{k};
        tmp_max = max(gg(l).(field));
        tmp_min = min(gg(l).(field));
        if numel(gg(l).(field))<2 % singleton dimension
            tmp_max = grid_max(k);
            tmp_min = grid_min(k);
        end
        % compare
        grid_max(k) = min(grid_max(k), tmp_max);
        grid_min(k) = max(grid_min(k), tmp_min);
    end
end

% Check inconsistent boundaries in gg grid
flag = false;
errid = '';
errmsg = '';
if any(grid_min>=grid_max)
    flag = true;
    errid = 'carfm:invalidValue';
    errmsg = 'Inconsistent boundaries found in GG grid';
end

end