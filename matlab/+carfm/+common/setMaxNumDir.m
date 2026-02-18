function [nold] = setMaxNumDir(n)
%SETMAXNUMDIR Calls casadi.GlobalOptions.setMaxNumDir
% setMaxNumDir controls the memory-speed trade-off. Smaller value
% reduces the required memory, e.g. length of generated code and the
% related compilation time, but increase cpu time. Default is 64. 
% With value of 1 the cpu times are typically more than double.
% NOTE this option may break in future casadi versions.
% See also https://groups.google.com/g/casadi-users/c/JT73r_cYMkk/m/I2Z1sHbDDQAJ

nold = casadi.GlobalOptions.getMaxNumDir();
casadi.GlobalOptions.setMaxNumDir(n);

end

