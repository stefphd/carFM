function smesh = createMesh(si, sf, N, s, y, opts)
%CREATEMESH Creates the mesh

switch opts.meshStrategy
    case 'adaptive' % Adpative mesh
        ntmp = sum((s>si) & (s<sf));
        stmp = interp1(s, s, linspace(si, sf, ntmp)','linear','extrap');
        ytmp = interp1(s, y, linspace(si, sf, ntmp)','linear','extrap');
        smesh = carfm.common.createAdaptiveMesh(stmp, ytmp, N, opts.meshRatio, ...
            opts.meshMinSecLen, opts.meshTransLen, opts.meshThFactor, opts.debugSolve);
    case 'manual' % Manual mesh
        % fix if numel(opts.meshFractions) is different from N-1
        if numel(opts.meshFractions)>(N-1) % del last vals
            opts.meshFractions = opts.meshFractions(1:(N-1));
        elseif numel(opts.meshFractions)<(N-1) % rep last val
            opts.meshFractions = [opts.meshFractions, repmat(opts.meshFractions(end), [1 (N-1)-numel(opts.meshFractions)])];
        end
        % calc tmesh
        smesh = [0, cumsum(opts.meshFractions/sum(opts.meshFractions))]*(sf-si) + si;
    % case 'equally-spaced' % Equally-spaced mesh
    otherwise % fallback to 'equally-spaced'
        smesh = linspace(si, sf, N); % equally spaced
end

end