function [f, df]  = ratio_handle(a, da, b, db, arguments)

f = eval ([' @(', arguments, ') feval(a, ', arguments, ') / feval(b, ', arguments, ')' ]);
df = eval ([' @(', arguments, ') (feval(b, ', arguments, ') * feval(da, ', arguments, ') - feval(a, ', arguments, ') * feval(db, ', arguments, ')) / feval(b, ', arguments, ')^2' ]);