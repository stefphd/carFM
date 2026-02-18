function output = interpField(s, backend, xfield, yfield, method, prefix)  
%INTERPFIELD Interpolates the yfield onto xfield of the input struct s
    % get data
    xdata = s.(xfield);
    ydata = s.(yfield);
    % interp name
    name = [prefix '_' yfield];
    % add two more point at the beggining and end to avoid possible nan
    % evaluations at the near of domain limits
    % data added are just the first and end point repeated
    xdata = [xdata(1)-(xdata(2)-xdata(1));
             xdata(:);
             xdata(end)+(xdata(end)-xdata(end-1))
             ];
    ydata = [ydata(1);
             ydata(:);
             ydata(end)];
    % create interpolant
    switch backend
        case 'casadi' % using casadi.interpolant
            if strcmp(method, 'spline')
                method = 'bspline';
            end
            if all(ydata==0)
                method = 'linear'; % workaround for NaN in the case of all zero if 'bspline'
            end
            output = casadi.interpolant(name, method, {xdata}, ydata);
        case 'internal' % using mltsfm.common.interpolant
            if strcmp(method, 'bspline')
                method = 'spline';
            end
            output = carfm.common.interpolant(name, {xdata}, ydata, method);
        case 'matlab' % using matlab griddedInterpolant
            if strcmp(method, 'bspline')
                method = 'spline';
            end
            output = griddedInterpolant({xdata}, ydata, method);
        otherwise
            error('mltsfm:invalidType','backend must be either ''matlab'', ''casadi'', or ''internal''.');
    end
end