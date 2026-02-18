function mltsout = calcSensitivities(mltsout, lamx, lamu, lamf, lamc, opts)
%CALCSENSITIVITIES Compute dymlts sensitivities

% average multipliers
% for dymlts multipliers are a bit noisy: moving average for smoothness
averageFactor = 200;
n = size(lamx,2); % num of elems, including possible collocations
k = round(n/averageFactor); k(k<1) = 1; % num of average samples (>=1)
lamu = movmean(lamu', [k,k])';

% parameter sensitivity
lamp = -lamu(4:end,:); % sensitivity parameter multupliers
% apply scaling
mltsout.spar = lamp * opts.lscale / opts.sscale; % apply scaling
mltsout.sparnames = opts.sensitivityPar;

end