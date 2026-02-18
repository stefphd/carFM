function ggout = ggmerge(varargin)
%GGMERGE See help/+carfm/+ssa/ggmerge.m for the help of this function

% Check nargin
if nargin < 1
    error('ggmerge:inputError', 'At least one GG map must be provided as input argument');
end

% Cat args
try
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
catch ME
    error('ggmerge:invalidInputs', 'Unable to concatenate GG map structure with different set of fields.');
end

% Check GG consistency
ggout.shift = gg(1).shift;
ggout.alpha = gg(1).alpha;
sseval = functions(gg(1).sseval);
car = sseval.workspace{1}.car;
opts = sseval.workspace{1}.opts;
sparnames = gg(1).sparnames;
for k = 1 : numel(gg)
    % ssevalk = functions(gg(1).sseval);
    % bikek = ssevalk.workspace{1}.car;
    if ~isequal(gg(l).alpha, ggout.alpha)
        error('ggmerge:notEqual','Values of alpha must be indentical')
    end
    if ~isequal(func2str(gg(l).shift), func2str(ggout.shift))
        error('ggmerge:notEqual','Shift function handles must be indentical')
    end
%    if ~isequal(car,bikek)
%        error('ggmerge:notEqual','GG maps must be generated from identical car data')
%    end
    if ~strcmp(sparnames,gg(k).sparnames)
        error('ggmerge:unableMerge','Values of sparnames must be identical')
    end
end

% Find V,g,Tf,Tr,wf,wr
ggout.V = unique([gg.V]);
ggout.g = unique([gg.g]);

% Sizes
na = numel(ggout.alpha);
nV = numel(ggout.V);
ng = numel(ggout.g);
sz = {na,nV,ng}; % sizes

% Init
ggout.at0 = nan(nV,1);
ggout.rho = nan(sz{:});
ggout.at = nan(sz{:});
ggout.an = nan(sz{:});
ggout.ax = nan(sz{:});
ggout.ay = nan(sz{:});
ggout.x = nan(sz{:}, size(gg(1).x, 4));
ggout.sseval = []; % filled later
ggout.spar = nan(sz{:}, size(gg(1).spar, 4));
ggout.sparnames = sparnames; 
ggout.pnlt = nan(sz{:}, size(gg(1).pnlt, 4));
ggout.exitflag = nan(sz{:});

% Loop over grid vectors
% loop over g
for ig = 1 : numel(ggout.g)
    g = ggout.g(ig);
    % loop over V
    for iV = 1 : numel(ggout.V)
        V = ggout.V(iV);
        args = {V,g};
        idx = {iV,ig};
        [k, idxk] = findgg(gg, args);
        if k < 0
            error('ggmerge:unableMerge','Unable to merge GG: missing GG at grid point V(%d)=%g, g(%d)=%g\n', ...
            iV,V,ig,g);
        end
        ggout.at0(iV) = gg(k).at0(idxk{1}); % idxk{1} = kV
        ggout.rho(:,idx{:}) = gg(k).rho(:,idxk{:});
        ggout.at(:,idx{:}) = gg(k).at(:,idxk{:});
        ggout.an(:,idx{:}) = gg(k).an(:,idxk{:});
        ggout.ax(:,idx{:}) = gg(k).ax(:,idxk{:});
        ggout.ay(:,idx{:}) = gg(k).ay(:,idxk{:});
        ggout.x(:,idx{:},:) = gg(k).x(:,idxk{:},:);
        ggout.spar(:,idx{:},:) = gg(k).spar(:,idxk{:},:);
        ggout.pnlt(:,idx{:},:) = gg(k).pnlt(:,idxk{:},:);
        ggout.exitflag(:,idx{:}) = gg(k).exitflag(:,idxk{:});
    end
end

% Populate sseval
opts.gz_g = ggout.g;
ggout.sseval = @(ia, iV, ig) carfm.ssa.sseval(ia, iV, ig, ggout.x, car, opts);

end

function [k, idxk] = findgg(gg, args)
    % args = {V,g}
    % idx = {kV,kg}
    k = -1;
    idxk = cell(size(args));
    % find exaxt gg
    for i = 1 : numel(gg)
        args0 = {gg(i).V, gg(i).g};
        idxk = cellfun(@(v1,v2) find(v1==v2, 1), args, args0, 'UniformOutput', false); % UniformOutput=false to output cells
        if ~any(cellfun(@isempty, idxk))
            k = i;
            return;            
        end
    end
end