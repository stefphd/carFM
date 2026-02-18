function opts = getUndocOptions(opts)
    % GETUNDOCOPTIONS Gets some undocumented options which are hidden to the 
    % end-user, either because only for internal use, or because they are 
    % associated with undocumented features not fully tested yet or that 
    % we don't want to make them visible.
    %
    % INPUT:
    % opts: user-defined option structure
    %
    % OUTPUT:
    % opts: structure containing -user-defined options plus undoc options

    % Undocumented features
    % Compute the GG starting from the braking (true) or the
    % acceleration (false), i.e. with alpha from -90deg to +90deg (true) 
    % and alpha from +90deg to -90deg (false).
    default_opts.computeGGFromBraking = true; 
    % Use penalty on front slip ratio in GG cost function (false).
    default_opts.useKappafPenalty = false;
    % Gravity factors = [gx/g, gy/g, gz/g] is internally used to account
    % for the road three-dimensionality, but is not documented
    % Only gz/g is exposed to the end user via the option gz_g
    default_opts.gravityFactors = [0, 0, opts.gz_g];
    % Hidden options
    % None
    % override default options
    opts_fields = fieldnames(opts);
    for k = 1 : numel(opts_fields)
        default_opts.(opts_fields{k}) = opts.(opts_fields{k});
    end
    opts = default_opts;
end

