function [th1,th2,d3] = invKinArm(xe,ye,ze,d1)
th1=pi()+atan2(ye,xe);

r=sqrt((xe^2)+(ye^2));
s=ze-d1;

th2=atan2(s,r)+(pi()/2);
d3=sqrt((r^2)+(s^2));
end