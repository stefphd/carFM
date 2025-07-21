function ggfull = ggmerge(varargin)
%GGMERGE Merge multiple g-g map structure
%
% INPUT:
% varargin: g-g maps, specified as multiple arguments or as a struct vector
%
% OUTPUT:
% ggfull: merged g-g structure

% Cat args
gg = [];
for k = 1 : nargin
    for l = 1 : numel(varargin{k})
        if iscell(varargin{k})
            gg0 = varargin{k}{l};
        else
            gg0 = varargin{k}(l);
        end
        gg = [gg, gg0];
    end
end

% Check alpha and shift
ggfull.shift = gg(1).shift;
ggfull.alpha = gg(1).alpha;
for k = 1 : numel(gg)
    if ~isequal(gg(l).alpha, ggfull.alpha)
        error('ggmerge:notEqual','Values of alpha must be indentical')
    end
    if ~isequal(func2str(gg(l).shift), func2str(ggfull.shift))
        error('ggmerge:notEqual','Shift function handles must be indentical')
    end
end

% Find V & g
ggfull.V = unique([gg.V]);
ggfull.g = unique([gg.g]);

% Init rho
ggfull.rho = nan(numel(ggfull.alpha), numel(ggfull.V), numel(ggfull.g));

% Init x
ggfull.x = nan(numel(ggfull.alpha), numel(ggfull.V), numel(ggfull.g), size(gg(1).x, 4));

% Iterate over g and V 
for kg = 1 : numel(ggfull.g)
    for kV = 1 : numel(ggfull.V)
        V = ggfull.V(kV);
        g = ggfull.g(kg);
        [i, iV, ig] = findgg(gg, V, g);
        if i < 0
            error('ggmerge:unableMerge','Unable to merge g-g')
        end
        ggfull.rho(:,kV,kg) = gg(i).rho(:,iV,ig);
        ggfull.x(:,kV,kg,:) = gg(i).x(:,iV,ig,:);
    end
end

% Calc at, an
at0 = ggfull.V;
for k = 1 : numel(ggfull.V)
    at0(k) = ggfull.shift(ggfull.V(k));
end
ggfull.at = (ggfull.rho.*sin(ggfull.alpha(:)) + at0) * 9.806;
ggfull.an = (ggfull.rho.*cos(ggfull.alpha(:)) + 0  ) * 9.806;

end

function [i, iV, ig] = findgg(gg, V, g)
    i = -1;
    iV = [];
    ig = [];
    % find exaxt gg
    for k = 1 : numel(gg)
        iV = find(V == gg(k).V);
        ig = find(g == gg(k).g);
        if ~isempty(iV) && ~isempty(ig)
            i = k;
            return;            
        end
    end
    % gg not found: find nearest along V
    for k = 1 : numel(gg)
        ig = find(g == gg(k).g);
        if ~isempty(ig)
            i = k;
            % find nearest V
            [~, iV] = min(abs(V - gg(k).V));
            return
        end
    end
    % gg not found: find nearest along g
    for k = 1 : numel(gg)
        iV = find(V == gg(k).V);
        if ~isempty(iV)
            i = k;
            % find nearest g
            [~, ig] = min(abs(g - gg(k).g));
            return
        end
    end
    % error at this point
end

