clc;clear;


addpath("~/DesignLab/20170699/EE405_MotorControl/build")
csv_title = "recorded_data_chirp"; % Enter the name of the CSV file containing the data you want to check here.

data = load(csv_title + ".csv");

elapsed_time = data(:,1);
control_loop_time = data(:,2);
one_step_time = data(:,3);
current_q = data(:,4);
current_qdot= data(:,5);
target_q = data(:,6);
target_qdot = data(:,7);
target_torque = data(:,8);
numdiff_currectQdot = data(:,9);
filtered_numdiff_currentQdot = data(:,10);

figure(1)
plot(elapsed_time, control_loop_time, 'k.',"MarkerSize",9);
xlabel('elapsed time [sec]')
ylabel('control loop time [sec]')
grid minor
set(gca,'FontSize',13)

%%%%%%%%%%%%
figure(2)
subplot(2,1,1)
plot(elapsed_time, current_q , ...
     'LineWidth', 2.3)
xlabel('time [sec]')
ylabel('current q [rad]')
grid minor
set(gca,'FontSize',13)

subplot(2,1,2)
plot(elapsed_time, current_qdot , ...
     'LineWidth', 2.3)
xlabel('time [sec]')
ylabel('current qdot [rad/s]')
grid minor
set(gca,'FontSize',13)

%%%%%%%%%%%%%

figure(3)
plot(elapsed_time, target_q , ...
     elapsed_time, current_q , '--', 'LineWidth', 2.3)
xlabel('time [sec]')
ylabel('current q [rad]')
legend(["target q", "current q"])
grid minor
set(gca,'FontSize',13)


figure(4)
plot(elapsed_time, target_qdot , ...
     elapsed_time, current_qdot , '--' , 'LineWidth', 2.3)
xlabel('time [sec]')
ylabel('current qdot [rad/s]')
legend(["target qdot", "current qdot"])
grid minor
set(gca,'FontSize',13)

figure(5)
plot(elapsed_time, target_torque , 'LineWidth', 2.3)
xlabel('time [sec]')
ylabel('target torque [Nm]')
legend(["target torque"])
grid minor
set(gca,'FontSize',13)

figure(6)
plot(elapsed_time, numdiff_currectQdot, '.-', ...
     elapsed_time, filtered_numdiff_currentQdot, '--', ...
     elapsed_time, current_qdot, 'LineWidth',3)
xlabel('time [sec]')
ylabel('qdot [rad/s]')
legend(["numerical diff", "filtered numerical diff", "ROBOTIS's qdot"])
grid minor
set(gca,'FontSize',13)

objective_error_max = 0.1 * ones(length(current_q),1);
objective_error_min = -0.1 * ones(length(current_q),1);
figure(7)
plot(elapsed_time, target_q - current_q, ...
     elapsed_time, objective_error_max, 'k--',...
     elapsed_time, objective_error_min, 'k--',...
     'LineWidth', 2.0)
xlabel('time [sec]')
ylabel('error [rad]')
legend(["error"], 'Location','best')
grid minor
set(gca,'FontSize',13)