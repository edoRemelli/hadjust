function [f, df] = normalize_handle(a, da, arguments)


f = eval([' @(', arguments, ') feval(a, ', arguments, ') / sqrt(feval(a, ', arguments, ')'' * feval(a, ', arguments, '))' ]);
df = eval([' @(', arguments, ') (feval(da, ', arguments, ') * sqrt(feval(a, ', arguments, ')'' * feval(a, ', arguments, ')) - feval(a, ', arguments, ') * feval(a, ', arguments, ')'' * feval(da, ', arguments, ') / sqrt(feval(a, ', arguments, ')'' * feval(a, ', arguments, '))) /  (feval(a, ', arguments, ')'' * feval(a, ', arguments, ')) ']);