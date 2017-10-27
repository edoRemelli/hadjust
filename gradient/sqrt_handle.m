function [f, df]  = sqrt_handle(s, ds, arguments)

f = eval(['@(', arguments, ') sqrt(feval(s, ', arguments, '))']);
df = eval(['@(', arguments, ') feval(ds, ', arguments, ') / 2 / sqrt(feval(s, ', arguments, '))']);

