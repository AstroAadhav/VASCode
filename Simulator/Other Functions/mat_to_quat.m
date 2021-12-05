function [q]= mat_to_quat(M)
mt=M';
f=M-mt;
d=sqrt(f(3,2)^2+f(1,3)^2+f(2,1)^2);
nhat=[f(3,2)/d f(1,3)/d f(2,1)/d];
t=trace(M);
th=acos((t-1)/2);
q0=cos(th/2);
q1=nhat(1)*sin(th/2);
q2=nhat(2)*sin(th/2);
q3=nhat(3)*sin(th/2);
q=[q0 q1 q2 q3];
end