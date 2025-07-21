function checkGGguess(guess,V0,g0,opts)
%CHECKGGGUESS Check the guess for the g-g map computation. Trown
%an error in the case of something is wrong.
% INPUT:
% 
% guess: g-g guess
% V0: speed in m/s (1-by-Nv vector)
% g0: vertical total acceleration in g (1-by-Ng vector)
% opts: option structure (override default options) - optional

% Do not check if guess is empty (i.e. not provided)
if isempty(fieldnames(guess))
    return
end

% Check field
if ~isfield(guess, 'x')
    eid = 'carfm:notFound';
    msg = 'Field ''x'' not found in guess';
    error(eid,msg)
end

% Check guess size
if ~isequal(numel(guess), size(V0,1))
    eid = 'carfm:notEqual';
    msg = 'Wrong size of guess input.';
    error(eid,msg)
end
    
% Default options and override with user-defined options
opts = carfm.ssa.getDefaultOptions(opts);

% Check size of grids
for iN = 1 : size(V0,1)
    X = guess(iN).x; % veh states
    if ~isequal(size(X,1:3), [opts.numGGpts, size(V0,2), size(g0,2)])
        eid = 'carfm:notEqual';
        msg = 'Wrong size of guess input.';
        error(eid,msg)
    end
end