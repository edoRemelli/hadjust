function [f, df] = product_handle(s, ds, a, da, arguments)

f  = eval(['@(', arguments, ') feval(s, ', arguments, ') * feval(a, ', arguments, ')']);
df = eval(['@(', arguments, ')feval(a, ', arguments, ') * feval(ds, ', arguments, ') + feval(s, ', arguments, ') * feval(da, ', arguments, ')']);