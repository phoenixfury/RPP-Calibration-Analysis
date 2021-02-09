function [q] = IK_RPP(p_global, link_lengths)
%IK_RPP Summary of this function goes here

%% Extracting the link lengths and local points

L1 = link_lengths(1);
L2 = link_lengths(2);

x = p_global(1);
y = p_global(2);
z = p_global(3);
%% Inverse kinematics

q1 = atan2(y, x);

q2 = z - L1;

d = sqrt(x^2 + y^2);

q3 =  d - L2;

q = [q1, q2, q3];

end

