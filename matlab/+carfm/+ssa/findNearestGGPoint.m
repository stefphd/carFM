function idx = findNearestGGPoint(alpha,V,g,gg)
%FINDNEARESTGGPOINT Find the neareast GG point for given alpha, V, g

% Find nearest alpha
[~, ia] = min(abs(gg.alpha-alpha));
% Find nearest V
[~, iV] = min(abs(gg.V-V));
% Find nearest g
[~, ig] = min(abs(gg.g-g));
% Index
idx = {ia,iV,ig};

end