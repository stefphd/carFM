function interp = interpolant(name, X, V, method)
%INTERPOLANT Create an interpolant using casadi and MATLAB
% Using high-level casadi interface for spline (casadi.interpolant), the 
% construction time is large when interpolation grid has many points. 
% This can be avoided by calculating the interpolant coefficients using 
% MATLAB (function spapi) and then using the low-level casadi interface for 
% splines (casadi.Function.bspline). However, using MATLAB function 'spapi' 
% requires the Curve Fitting Toolbox.

% spline order = degree + 1
switch method
    case 'linear' % linear has degree 1
        order = 2; 
    case {'spline', 'bspline'} % standard spline has degree 3 - also 'bspline' accepted
        order = 4;
    otherwise
        error('carfm:invalidValue','Interpolation method must be either ''linear'' or ''spline''.')
end
order = repmat({order}, size(X)); % same order in all directions

% Use matlab spapi to create B-form
if isscalar(X)
    V = V(:)'; % row vector for 1D interpolant
end
% spline order of each dimension is automatically reduced to mach the 
% length of the corisponding grid vector
bform = spapi(order, X, V);

% Use casadi to create interpolant from the splien coefficients
% degree = order - 1
interp = casadi.Function.bspline(name, bform.knots, bform.coefs(:), bform.order(:)-1);

end