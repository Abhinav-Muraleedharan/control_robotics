% Blin: Linearization of quadrotor continuous-time model dF/du|_op.
%
% Control for Robotics
% AER1517 Spring 2022
% Assignment 2
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistant: 
% SiQi Zhou
% siqi.zhou@robotics.utias.utoronto.ca
% Lukas Brunke
% lukas.brunke@robotics.utias.utoronto.ca
% Adam Hall
% adam.hall@robotics.utias.utoronto.ca
%
% This script is adapted from the course on Optimal & Learning Control for
% Autonomous Robots at the Swiss Federal Institute of Technology in Zurich
% (ETH Zurich). Course Instructor: Jonas Buchli. Course Webpage:
% http://www.adrlab.org/doku.php/adrl:education:lecture:fs2015
%
% --
% Revision history
% [14.05.25]    first version

function B = Blin(in1,in2,in3)
%BLIN
%    B = BLIN(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 6.0.
%    25-May-2014 19:06:46

Thxxyy = in3(2,:);
Thzz = in3(3,:);
mQ = in3(1,:);
qph = in1(4,:);
qps = in1(6,:);
qth = in1(5,:);
t2 = 1.0./mQ;
t3 = cos(qth);
t4 = 1.0./Thxxyy;
t5 = 1.0./t3;
t6 = sin(qps);
t7 = cos(qps);
t8 = sin(qth);
B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t2.*t8,-t2.*t3.*sin(qph),t2.*t3.*cos(qph),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t4.*t5.*t7,t4.*t6,-t4.*t5.*t7.*t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t4.*t5.*t6,t4.*t7,t4.*t5.*t6.*t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0./Thzz],[12, 4]);
