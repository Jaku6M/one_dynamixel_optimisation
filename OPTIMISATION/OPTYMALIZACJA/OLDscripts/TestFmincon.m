clear;
clc;
fun = @(x) objective_function(x); % Modified objective function

% Initial guess
x0 = [2, 2]; 

% Define linear inequality constraints: A*x <= b
A = [];
b = [];

% Define linear equality constraints: Aeq*x = beq
Aeq = [];
beq = [];

% Define lower and upper bounds for decision variables
lb = [-10, -10]; % Lower bounds
ub = [10, 10];   % Upper bounds

% Call fmincon
[x_opt,fval,exitflag,output,lambda,grad,hessian] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub);

% Display optimized results
disp('Optimized Decision Variables:');
disp(x_opt);
disp('Optimized Objective Value:');
disp(fval);
disp(output);
function val = objective_function(x)
    val = x(1)^2 + x(2)^2; % Example objective function (minimize x^2 + y^2)
    disp(['Value of fun at x = [', num2str(x(1)), ', ', num2str(x(2)), ']: ', num2str(val)]);
end
