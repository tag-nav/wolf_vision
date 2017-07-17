clear all;
close all;

data = load('../../../../KFO_cfix3D_odom.dat'); % 5sZX, 15sAll

v_p = data(:,1);
P1 = data(:,2:4);
Q1 = data(:,5:8);
Ab1 = data(:,9:11);
Wb1 = data(:,12:14);
Cp1 = data(:, 15:17);
Cq1 = data(:, 18:20);
Cab1 = data(:, 21:23);
Cwb1 = data(:, 24:26);
P2 = data(:,27:29);
Q2 = data(:,30:33);
Ab2 = data(:,34:36);
Wb2 = data(:,37:39);
Cp2 = data(:, 40:42);
Cq2 = data(:, 43:45);
Cab2 = data(:, 46:48);
Cwb2 = data(:, 49:51);

RangeP1_inf = P1 - 2*Cp1;
RangeP1_sup = P1 + 2*Cp1;
RangeAb1_inf = Ab1 - 2*Cab1;
RangeAb1_sup = Ab1 + 2*Cab1;
RangeWb1_inf = Wb1 - 2*Cwb1;
RangeWb1_sup = Wb1 + 2*Cwb1;
RangeQ1_inf = Q1(:,1:3) - 2*Cq1;
RangeQ1_sup = Q1(:,1:3) + 2*Cq1;

RangeP2_inf = P2 - 2*Cp2;
RangeP2_sup = P2 + 2*Cp2;
RangeAb2_inf = Ab2 - 2*Cab2;
RangeAb2_sup = Ab2 + 2*Cab2;
RangeWb2_inf = Wb2 - 2*Cwb2;
RangeWb2_sup = Wb2 + 2*Cwb2;

figure('Name','Varying sigma_p (Constraint)','NumberTitle','off');
subplot(2,2,1);
plot(v_p, P2(:,1), 'b');
hold on;
plot(v_p, P2(:,2), 'g');
plot(v_p, P2(:,3), 'r');
plot(v_p, RangeP2_inf(:,1), '--b');
plot(v_p, RangeP2_inf(:,2), '--g');
plot(v_p, RangeP2_inf(:,3), '--r');
plot(v_p, RangeP2_sup(:,1), '--b');
plot(v_p, RangeP2_sup(:,2), '--g');
plot(v_p, RangeP2_sup(:,3), '--r');
xlabel('sigma of p in Odometry Constraint');
ylabel('Estimated P (last) ');
legend('Px', 'Py', 'Pz');
title('last position wrt sigma_p (Constraint)');

subplot(2,2,2);
plot(v_p, Q1(:,1), 'b');
hold on;
plot(v_p, Q1(:,2), 'g');
plot(v_p, Q1(:,3), 'r');
plot(v_p, Q1(:,4), 'm');
plot(v_p, RangeQ1_inf(:,1), '--b');
plot(v_p, RangeQ1_inf(:,2), '--g');
plot(v_p, RangeQ1_inf(:,3), '--r');
plot(v_p, RangeQ1_sup(:,1), '--b');
plot(v_p, RangeQ1_sup(:,2), '--g');
plot(v_p, RangeQ1_sup(:,3), '--r');
xlabel('sigma p in Odometry Constraint');
ylabel('Quaternion estimation');
legend('Qx', 'Qy', 'Qz', 'Qw');
title('estimated Origin Q wrt sigma_p (Constraint)');

subplot(2,2,3);
plot(v_p, Ab1(:,1), 'b');
hold on;
plot(v_p, Ab1(:,2), 'g');
plot(v_p, Ab1(:,3), 'r');
plot(v_p, RangeAb1_inf(:,1), '--b');
plot(v_p, RangeAb1_inf(:,2), '--g');
plot(v_p, RangeAb1_inf(:,3), '--r');
plot(v_p, RangeAb1_sup(:,1), '--b');
plot(v_p, RangeAb1_sup(:,2), '--g');
plot(v_p, RangeAb1_sup(:,3), '--r');
xlabel('sigma of p in Odometry Constraint');
ylabel('Estimated Ab ');
legend('Abx', 'Aby', 'Abz');
title('ab Origin wrt sigma_p (Constraint)');

subplot(2,2,4);
plot(v_p, Wb1(:,1), 'b');
hold on;
plot(v_p, Wb1(:,2), 'g');
plot(v_p, Wb1(:,3), 'r');
plot(v_p, RangeWb1_inf(:,1), '--b');
plot(v_p, RangeWb1_inf(:,2), '--g');
plot(v_p, RangeWb1_inf(:,3), '--r');
plot(v_p, RangeWb1_sup(:,1), '--b');
plot(v_p, RangeWb1_sup(:,2), '--g');
plot(v_p, RangeWb1_sup(:,3), '--r');
xlabel('sigma of p in Odometry Constraint');
ylabel('Estimated Wb ');
legend('Wbx', 'Wby', 'Wbz');
title('wb Origin wrt sigma_p (Constraint)');


% 
% figure('Name','Varying sigma_p (Constraint)','NumberTitle','off');
% subplot(2,4,1);
% plot(v_p, Cab1(:,1), 'b');
% hold on;
% plot(v_p, Cab1(:,2), 'g');
% plot(v_p, Cab1(:,3), 'r');
% xlabel('sigma of p in Odometry Constraint');
% ylabel('sigma');
% legend('Abx', 'Aby', 'Abz');
% title('sigma(ab) wrt sigma_p (Constraint)');
% 
% subplot(2,4,5);
% plot(v_p, Ab1(:,1), 'b');
% hold on;
% plot(v_p, Ab1(:,2), 'g');
% plot(v_p, Ab1(:,3), 'r');
% xlabel('sigma of p in Odometry Constraint');
% ylabel('sigma');
% legend('Abx', 'Aby', 'Abz');
% title('estimated ab wrt sigma_p (Constraint)');
% 
% subplot(2,4,2);
% plot(v_p, Cwb1(:,1), 'b');
% hold on;
% plot(v_p, Cwb1(:,2), 'g');
% plot(v_p, Cwb1(:,3), 'r');
% xlabel('sigma of p in Odometry Constraint');
% ylabel('sigma');
% legend('Wbx', 'Wby', 'Wbz');
% title('sigma(wb) wrt sigma_p (Constraint)');
% 
% subplot(2,4,6);
% plot(v_p, Wb1(:,1), 'b');
% hold on;
% plot(v_p, Wb1(:,2), 'g');
% plot(v_p, Wb1(:,3), 'r');
% xlabel('sigma of p in Odometry Constraint');
% ylabel('sigma');
% legend('Abx', 'Aby', 'Abz');
% title('estimated wb wrt sigma_p (Constraint)');
% 
% subplot(2,4,3);
% plot(v_p, Cp1(:,1), 'b');
% hold on;
% plot(v_p, Cp1(:,2), 'g');
% plot(v_p, Cp1(:,3), 'r');
% xlabel('sigma of p in Odometry Constraint');
% ylabel('sigma');
% legend('CPx', 'CPy', 'CPz');
% title('sigma of estimated P wrt sigma_p (Constraint)');
% 
% subplot(2,4,7);
% plot(v_p, P1(:,1), 'b');
% hold on;
% plot(v_p, P1(:,2), 'g');
% plot(v_p, P1(:,3), 'r');
% xlabel('sigma p in Odometry Constraint');
% ylabel('position (m)');
% legend('Px', 'Py', 'Pz');
% title('estimated P wrt sigma_p (Constraint)');
% 
% subplot(2,4,4);
% plot(v_p, Cq1(:,1), 'b');
% hold on;
% plot(v_p, Cq1(:,2), 'g');
% plot(v_p, Cq1(:,3), 'r');
% xlabel('sigma of o in Odometry Constraint');
% ylabel('sigma');
% legend('CQx', 'CQy', 'CQz');
% title('sigma of estimated Quaternion wrt sigma_p (Constraint)');
% 
% subplot(2,4,8);
% plot(v_p, Q1(:,1), 'b');
% hold on;
% plot(v_p, Q1(:,2), 'g');
% plot(v_p, Q1(:,3), 'r');
% plot(v_p, Q1(:,4), 'm');
% xlabel('sigma p in Odometry Constraint');
% ylabel('Quaternion estimation');
% legend('Qx', 'Qy', 'Qz', 'Qw');
% title('estimated Q wrt sigma_p (Constraint)');
