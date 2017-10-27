function [f, df]  = norm_handle(a, da, arguments)

f = eval([' @(', arguments, ') sqrt(feval(a, ', arguments, ')'' * feval(a, ', arguments, ')) ']);
df = eval([' @(', arguments, ') feval(a, ', arguments, ')'' * feval(da, ', arguments, ') / sqrt(feval(a, ', arguments, ')'' * feval(a, ', arguments, ')) ']);


