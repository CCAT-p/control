function x = vel2wheel(fwdVel, angVel)


A = [0.5, 0.5; -0.5, 0.5];
b = [2*0.08*angVel; 2*fwdVel];
x = A*b/0.0143;


end