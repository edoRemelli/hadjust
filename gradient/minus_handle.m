function [f, df] = minus_handle(a, da, arguments)

f = eval(['@(', arguments, ') - feval(a, ', arguments, ')']);
df = eval([' @(', arguments, ') - feval(da, ', arguments, ')']);