function T = dhTrans(th,alph,a,d)
ct=cos(th); st=sin(th); ca=cos(alph); sa=sin(alph);

T=[ct  -st*ca   st*sa  a*ct;
   st   ct*ca  -ct*sa  a*st;
   0     sa       ca    d;
   0      0        0    1];

end