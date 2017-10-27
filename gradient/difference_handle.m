function [f, df]  = difference_handle(a, da, b, db, arguments)

f = eval(['@(', arguments, ') feval(a, ', arguments, ') - feval(b, ', arguments, ')']);
df = eval([' @(', arguments, ') feval(da, ', arguments, ') - feval(db, ', arguments, ')']);