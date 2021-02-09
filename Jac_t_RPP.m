function [Jt, J1, J2, J3] = Jac_t_RPP(q, theta, link_lengths)
%JAC_RPP Summary of this function goes here

%% Extracting the angles and thetas
q1 = q(1);
q2 = q(2);
q3 = q(3);

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);

L1 = link_lengths(1);
L2 = link_lengths(2);

Tbase = eye(4);
%% Getting the forward kinematics

T = FK_RPP(Tbase, q, theta, link_lengths);

T(1:3, 4) = 0;

%% Getting the jacobians
Td = Tbase * Rz(q1) * Rzd(theta1) * Tz(L1) * Tz(q2) * Tz(theta2) * Tx(L2) * Tx(q3) * Tx(theta3) / T ;

J1 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Tbase * Rz(q1) * Rz(theta1) * Tz(L1) * Tz(q2) * Tzd(theta2) * Tx(L2) * Tx(q3) * Tx(theta3) / T ;

J2 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Tbase * Rz(q1) * Rz(theta1) * Tz(L1) * Tz(q2) * Tz(theta2) * Tx(L2) * Tx(q3) * Txd(theta3) / T ;

J3 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Jt = [J1, J2, J3];
end

