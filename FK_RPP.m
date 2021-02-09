function [T, T1_2, T2_3] = FK_RPP(Tbase, q, theta, link_length)
%FK_RPP Summary of this function goes here
%% Extracting the angles and thetas

q1 = q(1);
q2 = q(2);
q3 = q(3);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);

L1 = link_length(1);
L2 = link_length(2);

%% Transformations

T1_2 = Tbase * Rz(q1) * Rz(theta1) * Tz(L1);

T2_3 = T1_2 * Tz(q2) * Tz(theta2) * Tx(L2);

T = T2_3 * Tx(q3) * Tx(theta3);

end

