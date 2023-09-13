function err = CalcVWerr(Cref,Cnow)
%Cnow 目标连杆——uLINK(to);
%Cref 目标的连杆位姿
perr = Cref.p - Cnow.p;% p_ref - p
Rerr = Cnow.R^ (-1) * Cref.R; %R(T) R_ref since R_transpose = R_inverse
werr = Cnow.R * rot2omega(Rerr);
err = [perr; werr];