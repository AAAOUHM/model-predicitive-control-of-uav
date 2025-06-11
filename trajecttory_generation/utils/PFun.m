function f = PFun(w,p)
P = p.c4 + p.c3*(w) + p.c2*(w*w) +p.c7*(w*w*w) + p.c8*(w*w*w*w);

f = P;