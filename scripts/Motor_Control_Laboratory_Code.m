clear;
clc;
figs = findall(0, 'Type', 'figure');
for k = 1:length(figs)
    clf(figs(k));
end

%% Variable Definitions %%
K_a = 0.41; % A/V
K_m = 0.11; % N*m/A
B = 0.001; % N*m*s/rad
J_tot = 0.00034; % N*m*s^2/rad
i_max = 4.1; % A
theta_0 = pi; % rad

K_da = K_a*K_m / B;
tau_m = J_tot / B;

OS = 0.1;
PM_pl = 59; % deg
zeta = sqrt(((log(OS))^2) / ((log(OS))^2 + pi^2));

%% Plant Definition %%
% Plant: Tm(s) = 45.1 / (s(0.34s + 1))
G = tf(K_da, [tau_m 1 0]);
% Closed-loop uncompensated
T_m = feedback(G, 1);

%% Proportional Controller %%
K_p = ((2*zeta)^2  * K_da * tau_m)^(-1);
C_p = zpk([], [], K_p);
G_p = series(G, C_p);

% Closed-loop with proportional controller
T_p = feedback(G_p, 1);
[GM_p, PM_p, w_gc_p, w_pc_p] = margin(T_p);

% Upper bound proportional controller gain
K_pmax = (i_max) / (K_a * theta_0);
K_pmax_buffer = 0.9*K_pmax;

%% Phase-Lead Compensator Design %%
% Values derived analytically
w_gc = sqrt((-1 + sqrt(1 - 4*(tau_m^2)*(-K_da^2))) / (2*(tau_m^2)));
PM_Tm = 180 + (-90 - atand(tau_m * w_gc));
phi_max = PM_pl - PM_Tm + 5;
beta = (1 - sind(phi_max)) / (1 + sind(phi_max));
K_c = (i_max*beta) / (K_a*theta_0); % from current limit
w_max = sqrt((-1 + sqrt(1 - 4*(tau_m^2)*(-((K_da*K_c)^2) / beta))) / (2*(tau_m^2))); % rad/s
tau = 1 / (w_max * sqrt(beta));

% Build compensator
C_lead = zpk([-1/tau], [-1/(beta*tau)], K_c/beta);
G_c = series(G, C_lead);

% Closed-loop with lead compensator
T_c = feedback(G_c, 1);
[GM_c, PM_c, w_gc_c, w_pc_c] = margin(T_c);

%% Display Final Values %%
disp('--- Proportional Controller ---')
fprintf('K_p = %.4f\n', K_p);
fprintf('Phase Margin = %.2f deg at %.2f rad/s\n', PM_p, w_gc_p);

disp(' ')
disp('--- Phase-Lead Compensator ---')
fprintf('phi_max = %.2f deg\n', phi_max);
fprintf('beta = %.4f\n', beta);
fprintf('K_c = %.4f\n', K_c);
fprintf('tau = %.3f s\n', tau);
fprintf('Zero = %.3f rad/s\n', 1/tau);
fprintf('Pole = %.3f rad/s\n', 1/(beta*tau));
fprintf('Gain = %.3f\n', K_c/beta);
fprintf('Phase Margin = %.2f deg at %.2f rad/s\n', PM_c, w_gc_c);

%% Bode Plots %%
figure(1);
margin(G);
hold on;
margin(G_p);
hold off;
title('Proportional Controller Open-Loop Bode Plot');
legend('Uncompensated', 'Proportional', 'Location', 'southwest');

figure(2);
margin(G);
hold on;
margin(G_c);
hold off;
title('Phase-Lead Controller Open-Loop Bode Plot');
legend('Uncompensated', 'Phase-Lead', 'Location', 'southwest');

%% Step Responses %%
figure(3);
step(theta_0*T_m, theta_0*T_p, theta_0*T_c, 5);
legend('Uncompensated', 'Proportional', 'Phase-Lead');
title('Angle Response to Step Input Comparison');
ylabel("Angle (rad)");

figure(4);
step(theta_0*feedback(1, G), theta_0*feedback(C_p, G), theta_0*feedback(C_lead, G), 5);
legend('Uncompensated', 'Proportional', 'Phase-Lead');
title('D/A Voltage Response to Step Input Comparison');
ylabel("Voltage (V)");

%% Display Final Step Response Metrics %%
P_Ang_Info = stepinfo(T_p);
C_Ang_Info = stepinfo(theta_0*T_c);
P_Volt_Info = stepinfo(theta_0*feedback(C_p, G));
C_Volt_Info = stepinfo(theta_0*feedback(C_lead, G));

disp(' ')
disp('--- Proportional Controller: Angle Response Design Criteria ---')
fprintf("Percent Overshoot = %.2f \n", P_Ang_Info.Overshoot)
fprintf("Settling Time = %.2f s \n", P_Ang_Info.SettlingTime)

disp(' ')
disp('--- Phase-Lead Compensator: Angle Response Design Criteria ---')
fprintf("Percent Overshoot = %.2f \n", C_Ang_Info.Overshoot)
fprintf("Settling Time = %.2f s \n", C_Ang_Info.SettlingTime)

disp(' ')
disp('--- Proportional Controller: Maximum Current ---')
fprintf("Maximum Current = %.2f \n", K_a*P_Volt_Info.Peak)
fprintf("Maximum Location = %.2f \n", K_a*P_Volt_Info.PeakTime)

disp(' ')
disp('--- Phase-Lead Compensator: Maximum Current ---')
fprintf("Maximum Current = %.2f \n", K_a*C_Volt_Info.Peak)
fprintf("Maximum Location = %.2f \n", K_a*C_Volt_Info.PeakTime)