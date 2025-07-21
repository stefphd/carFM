% File solve.m 
% Solve OPTra problem
% Copyright (C) 2024 Stefano Lovato

function sol = solve(problem)
    % Check arg is performed inside optraMEX
    % Call optraMEX
    sol = carfm.optra.optraMEX(problem);
end