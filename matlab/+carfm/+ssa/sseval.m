function  ssdata = sseval(ia, iV, ig, X, car, opts)
    % Evaluate steady-state at a g-g-g point
    % Check arguments
    arguments
        ia (:,:) {mustBeNumeric,mustBeVector,mustBeInteger,mustBePositive}
        iV (:,:) {mustBeNumeric,mustBeVector,mustBeInteger,mustBePositive}
        ig (:,:) {mustBeNumeric,mustBeVector,mustBeInteger,mustBePositive}
        X % nothing
        car % nothing
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
    % add possily missing options
    opts = carfm.ssa.getDefaultOptions(opts);
    % remove active inputs
    opts.ssActiveLongInputs(:) = false;
    opts.ssActiveLatInputs(:) = false;
    % eval
    gz_g = opts.gz_g;
    for k = 1 : numel(ia)
        opts.gravityFactors = [0, 0, gz_g(ig(k))];
        x = X(ia(k),iV(k),ig(k),:);
        [~, ssdata(k)] = carfm.ssa.steadyStateEquations(x(:), u_long0, u_lat0, car, opts);
    end
    if dogrid
        ssdata = reshape(ssdata, size(IA));
    end
end