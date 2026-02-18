% CasadiIPOPTCallback class
classdef CasadiIPOPTCallback < casadi.Callback
    properties
        nx
        nu
        nc
        nz
        ng
        t
        N
        printInt
        solver
        iter_callback
    end
    methods
        function self = CasadiIPOPTCallback(name, nx, nu, nc, nz, ng, t, printInt, iter_callback)
            self@casadi.Callback();
            self.nx = nx;
            self.nu = nu;
            self.nc = nc;
            self.nz = nz;
            self.ng = ng;
            self.t = t;
            self.N = numel(t);
            self.printInt = printInt;
            self.iter_callback = iter_callback;
            construct(self, name);
        end

        function set_solver(self,solver)
            self.solver = solver;
        end
        
        function n_in = get_n_in(self)
            n_in = 6;
        end
        
        function n_out = get_n_out(self)
            n_out = 1; 
        end
        
        function name_in = get_name_in(self, i)
            name_in = {'x', 'f', 'g', 'lam_x', 'lam_g', 'lam_p'}; % Input names
            name_in = name_in{i+1};
        end
        
        function name_out = get_name_out(self, i)
            name_out = {'ret'}; % Output name
            name_out = name_out{i+1};
        end
        
        function sparsity_in = get_sparsity_in(self, i)
            if i == 0 % x
                sparsity_in = casadi.Sparsity.dense(self.nz);
            elseif i == 1 % f
                sparsity_in = casadi.Sparsity.dense(1); 
            elseif i == 2 % g
                sparsity_in = casadi.Sparsity.dense(self.ng);
            elseif i == 3 % lam_x 
                sparsity_in = casadi.Sparsity.dense(self.nz);
            elseif i == 4 % lam_g 
                sparsity_in = casadi.Sparsity.dense(self.ng);
            else
                sparsity_in = casadi.Sparsity(0, 0); % default sparsity
            end
        end
        
        function out = eval(self, arg)
            iter = numel(self.solver.stats.iterations.obj) - 1;
            if (rem(iter, self.printInt) == 0)
                obj = self.solver.stats.iterations.obj(end);
                inf_pr = self.solver.stats.iterations.inf_pr(end);
                inf_du = self.solver.stats.iterations.inf_du(end);
                z = reshape(full(arg{1}), [self.nx+self.nu, self.N]);
                g = full(arg{3});
                g = reshape(g(1:(self.nx+self.nc)*(self.N-1)), [self.nx+self.nc, self.N-1]);
                sol.time = self.t;
                sol.state = z(1:self.nx,:);
                sol.control = z(1:self.nu,:);
                sol.dyn_constraint = g(1:self.nx,:);
                % call iter_callback
                self.iter_callback(iter, obj, inf_pr, inf_du, sol);
            end
            out = {0};
        end
    end
end