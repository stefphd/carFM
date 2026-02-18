function ggout = ggrep(ggout, varargin)
% GGREP See help/+carfm/+ssa/ggrep.m for the help of this function

% Check nargin
if nargin < 2
    error('ggrep:inputError', 'At least two GG maps must be provided as input argument');
end

% Cat args
try
    gg = [];
    for k = 1 : nargin-1
        for l = 1 : numel(varargin{k})
            if iscell(varargin{k})
                gg = varargin{k}{l};
            else
                gg = varargin{k}(l);
            end
            gg = [gg, gg];
        end
    end
catch ME
    error('ggrep:invalidInputs', 'Unable to concatenate GG map structure with different set of fields.');
end

% Check GG consistency
sseval = functions(ggout.sseval);
car = sseval.workspace{1}.car;
opts = sseval.workspace{1}.opts;
sparnames = ggout.sparnames;
for k = 1 : numel(gg)
    % ssevalk = functions(ggout.sseval);
    % bikek = ssevalk.workspace{1}.car;
    if ~isequal(func2str(gg(l).shift), func2str(ggout.shift))
        error('ggrep:notEqual','Shift function handles must be indentical')
    end
%    if ~isequal(car,bikek)
%        error('ggrep:notEqual','GG maps must be generated from identical car data')
%    end
    if ~strcmp(sparnames,gg(k).sparnames)
        error('ggrep:unableMerge','Values of sparnames must be identical')
    end
end

% Loop over gg
for k = 1 : numel(gg)
    ggk = gg(k);
    % intersection
    arg1 = {ggout.alpha, ggout.V, ggout.g};
    arg2 = {ggk.alpha, ggk.V, ggk.g};
    [idx1,idx2] = carfm.ssa.gridIntersection(arg1, arg2);
    % assignment
    ggout.rho(idx1{:}) = ggk.rho(idx2{:});
    ggout.at(idx1{:}) = ggk.at(idx2{:});
    ggout.an(idx1{:}) = ggk.an(idx2{:});
    ggout.ax(idx1{:}) = ggk.ax(idx2{:});
    ggout.ay(idx1{:}) = ggk.ay(idx2{:});
    ggout.x(idx1{:},:) = ggk.x(idx2{:},:);
    ggout.exitflag(idx1{:}) = ggk.exitflag(idx2{:});
    ggout.spar(idx1{:},:) = ggk.spar(idx2{:},:);
    ggout.pnlt(idx1{:}) = ggk.pnlt(idx2{:});
end

% Populate sseval
opts.gz_g = ggout.g;
ggout.sseval = @(ia, iV, ig) carfm.ssa.sseval(ia, iV, ig, ggout.x, car, opts);