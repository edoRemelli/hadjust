function [f, df] = cross_handle(a, da, b, db, arguments)

f = eval([' @(', arguments, ') cross(feval(a, ', arguments, '), feval(b, ', arguments, '))']);
df = eval([' @(', arguments, ') my_cross(feval(a, ', arguments, '), feval(db, ', arguments, ')) - my_cross(feval(b, ', arguments, '), feval(da, ', arguments, '))']);

% c1 = rand(D, 1);
% c2 = rand(D, 1);
% a = @(c1, c2) -2 * c1 + c2;
% b = @(c1, c2) 3 * c1 + c2;
% da = @(c1, c2) -2 * eye(D, D);
% db = @(c1, c2) 3 * eye(D, D);% 
% 
% O = @(c1, c2) cross(a(c1, c2), b(c1, c2));
% dO = @(c1, c2) my_cross(a(c1, c2), db(c1, c2)) - my_cross(b(c1, c2), da(c1, c2));
% O = @(c1) O(c1, c2);
% disp(my_gradient(O, c1));
% disp(dO(c1, c2));
