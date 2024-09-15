clear
clc

%% Main Simulation
result = [];

for k0 = 5:5:150
    for m0 = 0.01:0.005:0.15
        for geo_ratio = 1.0:1.0

        k = k0;                             % Preload of the clutch spring
        m = m0;                             % Shoe mass (kg)

        disp(['k: ', num2str(k), ' m: ', num2str(m), ' geo_ratio: ', num2str(geo_ratio)])


        deltaS_o = 4.5;                     % Preload spring length (mm)       
        n = 3;                              % Number of shoes
        rho = 1.167;                        % air density (kg/m^3)
        Cd_A = 0.6;                         % Cd*A
        f_w = 0.02; 
        M = 113 + 27 + 60;                  % Total mass (kg): vehicle + motor + battery + rider
        theta_g = 0;                        % Slope angle (deg)
        g = 9.80665;                        % Gravitational acceleration (m/s^2)
        r_f = 12 * 0.0254 / 2 + 0.11 * 0.7; % Front wheel radius (m)
        r_r = 12 * 0.0254 / 2 + 0.12 * 0.7; % Rear wheel radius (m)
        I_f = 0.15;                         % Estimated front wheel inertia (kg-m^2)
        I_r = 0.138 + 0.062;                % Rear wheel inertia (kg-m^2)
        I_e = 3e-3;                         % Estimated crankshaft inertia (kg-m^2)
        I_m = 0.003;                        % Estimated motor rotor inertia (kg-m^2)

        % Two-speed transmission ratios
        n1 = 9;                             % First gear reduction ratio
        n2 = 5;                             % Second gear reduction ratio
        gear_mode = 1;                      % Initial gearbox mode (first gear)
        T_out = 0;                          % Output shaft torque

        % Centrifugal clutch data
        I1o = 3.413e-003;                   % Inertia of clutch driver (kg-m^2)
        I2o = 5.574e-003;                   % Inertia of clutch cover (kg-m^2)

        % Calculate equivalent inertia
        I1 = I1o + I_m + (I_r / r_r^2 + I_f / r_f^2 + M) * r_r^2 / n2^2;
        I2 = I1;

        omega1 = 0;                         % Initial speed of the driver
        omega2 = 0;                         % Initial speed of the driven
        dt_o = 0.0001;                      % Time step (s)
        dt = dt_o;                          % Time step (s)
        t_max = 100;                        % Total time (s)
        theta1 = 0;                         % Initial driver angle (rad)
        theta2 = 180 * pi / 180;            % Initial driven angle (rad)
        t = 0;                              % Initial time (s)
        i = 1;                              % Loop starting value
        var1 = 0;                           % Control variable
        var2 = 0;                           % Control variable
        var3 = 0;       
        status = 0;                         % Clutch engagement status
        check = 0;      
        Temp = 25;                          % Initial temperature

        ansmatrixCW = zeros(15, t_max / dt_o);
        
        % Execute while loop
        while t <= t_max
            if omega1 / n1 >= omega2 / n2 - 0.0001 % First gear
                gear_mode = 1;
            else % Second gear
                gear_mode = 2;
            end

            t_shift = 150;

            if t < t_shift || t > 70
                % Acceleration phase
                T_motor = [35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 35 34.1 32.93 31.83 30.8 29.84 28.94 28.09 27.28 26.53 25.81 25.13 24.49 23.87 23.29 22.74 22.21 21.7 21.22 20.76 20.32 19.89 19.49 19.1 18.72 18.36 18.02 17.68 17.36 17.05 16.75 16.46 16.19 15.92 15.65 15.4 15.16 14.92 14.69 14.47 14.25 14.04 13.84 13.64 13.45 13.26 13.08 12.9 12.73 12.56 12.4 12.24 12.09 11.94 11.79 11.65 11.51 11.37 11.23 11.1 10.98 10.85 10.73 10.61 10.49 10.38 10.27 10.16 10.05];
                Tm = interp1([0:100:9500] * (2 * pi) / 60, T_motor, omega1, 'linear');
                if gear_mode == 1 % First gear
                    velocity = r_r * omega1 / n1;
                    R = 0.5 * rho * Cd_A * (velocity)^2 + f_w * M * g * cosd(theta_g) + M * g * sind(theta_g);
                    Tr = R * r_r;
                    T1 = -(Tm - Tr / n1);
                    T2 = (Tm - Tr / n1) * n1 / n2;
                    omega_out = omega1 / n1;
                else % Second gear
                    velocity = r_r * omega2 / n2;
                    R = 0.5 * rho * Cd_A * (velocity)^2 + f_w * M * g * cosd(theta_g) + M * g * sind(theta_g);
                    Tr = R * r_r;
                    T1 = -Tm;
                    T2 = -Tr / n2;
                    omega_out = omega2 / n2;
                end    
            else
                % Braking phase
                Tm = 0;
                if gear_mode == 1 % First gear
                    velocity = r_r * omega1 / n1;
                    R = 0.5 * rho * Cd_A * (velocity)^2 + f_w * M * g * cosd(theta_g) + M * g * sind(theta_g);
                    Tr = R * r_r;
                    T1 = -(Tm - Tr / n1);
                    T2 = (Tm - Tr / n1) * n1 / n2;
                    omega_out = omega1 / n1;
                else % Second gear
                    velocity = r_r * omega2 / n2;
                    R = 0.5 * rho * Cd_A * (velocity)^2 + f_w * M * g * cosd(theta_g) + M * g * sind(theta_g);
                    Tr = R * r_r;
                    T1 = -Tm;
                    T2 = -Tr / n2;
                    omega_out = omega2 / n2;
                end
            end
            
            if abs(omega1 - omega2) <= 0.01
                friction = 2;
                if T2 >= -T1 % Self-decelerating
                    mode = 2;
                    if T2 >= 0
                        status = 2; % Fully engaged
                    else
                        status = -2; % Fully engaged (reverse)
                    end
                    Tmax = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
                    alpha = (T2 - T1) / (I1 + I2);
                    T0 = T2 - I2 * alpha;
                    if T0 > Tmax
                        friction = 1;
                        if T2 >= 0
                            status = 1; % Torque too high
                        else
                            status = -1;
                        end
                        [T, ~] = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
                    else
                        T = T0;
                    end
                else % Self-accelerating
                    mode = 1;
                    if T2 >= 0
                        status = -2;
                    else
                        status = 2;
                    end
                    [Tmax, ~] = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
                    Tmax = -Tmax;
                    if Tmax > 0
                        error('Lock-up occurred');
                    end
                    alpha = (T2 - T1) / (I1 + I2);
                    T0 = T2 - I2 * alpha;
                    if T0 < Tmax
                        friction = 1;
                        if T2 >= 0
                            status = -1;
                        else
                            status = 1;
                        end
                        [T, ~] = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
                        T = -T;
                    else
                        T = T0;
                    end
                end
            else % During engagement
                friction = 1; % Dynamic friction
                if omega2 > omega1
                    mode = 2;
                    status = 1; % Engaging
                    [T, ~] = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
                else
                    mode = 1;
                    status = -1;
                    [T, ~] = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
                    T = -T;
                    if T > 0
                        error('Lock-up occurred');
                    end
                end
            end
            
            % Calculate angular acceleration
            if gear_mode == 1 && T == 0
                alpha1 = (Tm - (Tr / n1)) / (I1 + I2 * (n2 / n1)^2);
                alpha2 = alpha1 * n2 / n1;
            elseif gear_mode == 1 && T < 0
                alpha1 = (Tm - (Tr / n1) + (1 - (n2 / n1)) * T) / (I1 + I2 * (n2 / n1)^2);
                alpha2 = alpha1 * n2 / n1;
                T12 = (Tr / n2) + T + I2 * alpha2;
                if T12 <= 0
                    gear_mode = 2;
                    alpha1 = (Tm + T) / I1;
                    alpha2 = (-T - (Tr / n2)) / I2;
                end
            elseif gear_mode == 2
                alpha1 = (Tm + T) / I1;
                alpha2 = (-T - (Tr / n2)) / I2;
            end

            % Avoid omega < 0
            if omega1 + alpha1 * dt < 0
                alpha1 = -omega1 / dt;
            end
            if omega2 + alpha2 * dt < 0
                alpha2 = -omega2 / dt;
            end

            % Record data
            [T_Record, ~] = centrifugal_clutch(mode, 2, omega1, k, n, m, geo_ratio);
            [~, omega0] = centrifugal_clutch(mode, friction, omega2, k, n, m, geo_ratio);
            ansmatrixCW(:, i) = [theta1; theta2; omega1; omega2; alpha1; alpha2; T1; T2; T; status; T_Record; t; gear_mode; omega_out; velocity];

            % Check omega change validity
            if i > 1 && (ansmatrixCW(3, i-1) - ansmatrixCW(4, i-1)) * (ansmatrixCW(3, i) - ansmatrixCW(4, i)) < 0 &&...
                    abs(ansmatrixCW(3, i-1) - ansmatrixCW(4, i-1)) > 0.01 &&...
                    abs(ansmatrixCW(3, i) - ansmatrixCW(4, i)) > 0.01
                dt = dt / 10;
                i = i - 1;
                theta1 = ansmatrixCW(1, i);
                theta2 = ansmatrixCW(2, i);
                omega1 = ansmatrixCW(3, i);
                omega2 = ansmatrixCW(4, i);
                alpha1 = ansmatrixCW(5, i);
                alpha2 = ansmatrixCW(6, i);
            else
                dt = dt_o;
            end

            % Update angles, speed, and time
            dtheta1 = omega1 * dt + 0.5 * alpha1 * dt^2;
            dtheta2 = omega2 * dt + 0.5 * alpha2 * dt^2;
            theta1 = theta1 + dtheta1;
            theta2 = theta2 + dtheta2;
            omega2 = omega2 + alpha2 * dt;
            omega1 = omega1 + alpha1 * dt;
            t = t + dt;
            i = i + 1;
        end
        
        % Adjust matrix size
        ansmatrixCW(:, (find(ansmatrixCW(14, :), 1, 'last') + 1):end) = [];
        
        % Determine engagement speed
        index = find(abs(ansmatrixCW(3, :) - ansmatrixCW(4, :)) < 1);
        if isempty(index)
            omega_engaged = 0;
        else
            kk = find(index > 100000);
            if isempty(kk)
                omega_engaged = 0;
            else
                index = index(kk(1));
                omega_engaged = ansmatrixCW(3, index) / pi() * 30;
            end
        end

        result = [k * deltaS_o, m, geo_ratio, omega0, omega_engaged; result];


        end
    end
end


%% Save simulation result
save("result_cofig_b.mat", "result", '-mat');

data = load('result_cofig_b.mat');
dataMatrix = data.result;
T = array2table(dataMatrix);
writetable(T, 'result_cofig_b.xlsx');


%% Plot simulation result
x = result(:, 1);
y = result(:, 2);
z = result(:, 4);
w = result(:, 5);

% Filter out points where w=0
non_zero_indices = w ~= 0;
x_filtered = x(non_zero_indices);
y_filtered = y(non_zero_indices);
z_filtered = z(non_zero_indices);
w_filtered = w(non_zero_indices);

% Keep original variables for indexing purposes
result_filtered = result(non_zero_indices, :);

% Target engagement speed and find the closest match
purpose_omega = 6000; % Target engagement speed in rpm
delta_omega = abs(z_filtered - purpose_omega);
index_o = find(delta_omega == min(delta_omega));
para = [result_filtered(index_o, 1), result_filtered(index_o, 2), result_filtered(index_o, 3)];

% Define the grid for interpolation (adjust step size for smoother plot)
[X, Y] = meshgrid(min(x_filtered):5 * 4.5:max(x_filtered), min(y_filtered):0.005:max(y_filtered));

% Interpolate Z values over the grid
Z = griddata(x_filtered, y_filtered, z_filtered, X, Y, 'v4');

% Interpolate W values over the grid for the second plot
W = griddata(x_filtered, y_filtered, w_filtered, X, Y, 'v4');

% Plot the surface
figure(1)
surf(X, Y, W);
shading interp;
colormap(gray);
colorbar;

% Improve aesthetics for publication-quality figures
title('Config. B Full Engagement Rotating Speed', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('F_{preload} (N)', 'FontSize', 12, 'FontName', 'Times New Roman');
ylabel('m_{shoe} (g)', 'FontSize', 12, 'FontName', 'Times New Roman');
zlabel('\omega (rpm)', 'FontSize', 12, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 10, 'LineWidth', 1, 'FontName', 'Times New Roman');
grid on;
box on;