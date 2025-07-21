function mltsout = calcSensitivities(mltsout, lamf, lamc, opts)
%CALCSENSITIVITIES Compute ggmlts sensitivities

% gg sensitivity
lamgg = lamc(end-1,:); % end-1 is the gg constraint
rho = mltsout.rho;
rhomax = mltsout.rhomax;
srhomax = - lamgg .* 2.*(rho.^2+1).*rhomax./(rhomax.^2+1).^2;
mltsout.srhomax = srhomax * opts.lscale / opts.cscale(end-1) / opts.sscale; % apply scaling

end