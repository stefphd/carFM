function  ssdata = sseval(ia, iV, ig, X, bike, opts)
    % Evaluate steady-state at a g-g-g point
    % Check arguments
    arguments
        ia (:,:) {mustBeNumeric,mustBeVector,mustBeInteger,mustBePositive}
        iV (:,:) {mustBeNumeric,mustBeVector,mustBeInteger,mustBePositive}
        ig (:,:) {mustBeNumeric,mustBeVector,mustBeInteger,mustBePositive}
        X % nothing
        bike % nothing
        opts % nothing
    end
    u_long0 = nan(3,1);
    u_lat0 = nan(3,1);
    % check if not col vec with same size -> create grid
    dogrid = (isrow(ia) || isrow(iV) || isrow(ig)) || ...
             (~isequal(size(ia),size(iV)) || ~isequal(size(ia),size(ig)));
    if dogrid
        [IA,IV,IG] = ndgrid(ia,iV,ig);
        ia = IA(:); iV = IV(:); ig = IG(:); % vectorize
    end
    % eval
    gz_g = opts.gz_g;
    for k = 1 : numel(ia)
        opts.gz_g = gz_g(ig(k));
        x = X(ia(k),iV(k),ig(k),:);
        [~, ssdata(k)] = carfm.ssa.steadyStateEquations(x(:), u_long0, u_lat0, bike, opts);
    end
    if dogrid
        ssdata = reshape(ssdata, size(IA));
    end
end