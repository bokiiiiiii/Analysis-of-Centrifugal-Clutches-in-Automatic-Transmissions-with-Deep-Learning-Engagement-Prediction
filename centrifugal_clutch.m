function [T, omega0] = centrifugal_clutch(mode, friction, omega1, k, n, m, geo_ratio)

    % Centrifugal Clutch Data
    a = 46 * geo_ratio; % Distance from pin to clutch center (mm)
    b = 23 * geo_ratio; % Pad width (mm)
    c = 39.516 * geo_ratio; % Perpendicular distance from centrifugal force to pin (mm)
    r_cm = 46.696 * geo_ratio; % Distance from pad's center of mass to the center (mm)
    r = 145/2 * geo_ratio; % Clutch outer shell radius (mm)
    r_s1 = 50.655 * geo_ratio; % Perpendicular distance from spring force to pin (mm)
    r_s2 = 0.887 * geo_ratio; % Perpendicular distance from spring force to pin (mm)
    thetad_c1 = 33; % Initial position of pad (degrees)
    thetad_c2 = 93; % Final position of pad (degrees)
    theta_c1 = thetad_c1 * pi / 180; % Convert to radians
    theta_c2 = thetad_c2 * pi / 180; % Convert to radians
    mu_s = 0.2; % Static friction coefficient
    mu_d = 0.1; % Dynamic friction coefficient
    deltaS_o = 4.5 * geo_ratio; % Initial spring elongation (mm)
    F_spr_o = k * deltaS_o; % Initial spring force (N)
    F_rbr = 9.064 * geo_ratio; % Friction force of rubber block (N)
    d = 55 * geo_ratio; % Perpendicular distance from friction force to pin (mm)
    x_max = 4.5 * geo_ratio; % Maximum distance from pad to outer shell (mm)
    x = 0.5 * geo_ratio; % Pad clearance to shell (mm)

    % Spring deformation deltaS (mm)
    e = sqrt(a ^ 2 + (r - x_max) ^ 2 - 2 * a * (r - x_max) * cos(93 * pi / 180));
    phi = acos((a ^ 2 + e ^ 2 - (r - (x_max - x)) ^ 2) / (2 * a * e)) - asin((r - x_max) * sin(93 * pi / 180) / e);
    alpha = acos((12.5 ^ 2 + (sqrt(3) * a) ^ 2 - 71 ^ 2) / (2 * 12.5 * sqrt(3) * a));
    beta = acos((52 ^ 2 + 71 ^ 2 - 38 ^ 2) / (2 * 52 * 71));
    gamma = asin(12.5 * sin(alpha) / 71);
    deltaS = sqrt(52 ^ 2 + 12.5 ^ 2 + 3 * a ^ 2 + 2 * 52 * 12.5 * cos(alpha + beta + gamma) - 2 * sqrt(3) * a * (52 * cos(beta + gamma + phi) + 12.5 * cos(alpha - phi))) - 38;

    % Maximum pressure angle theta_a (rad)
    if theta_c2 >= pi / 2
        theta_a = pi / 2;
    else
        theta_a = theta_c2;
    end

    % Centrifugal force and spring force
    F_clu = m * omega1 .^ 2 * r_cm / 1000; % Centrifugal force on the pad (N)
    F_spr = F_spr_o + k * deltaS; % Spring force (N)

    % Initial engagement speed omega0 (rad/s)
    omega0 = sqrt((F_spr * (r_s1 + r_s2) + F_rbr * d) / (m * r_cm / 1000 * c));

    % Determine friction type
    if friction == 1
        mu = mu_d; % Dynamic friction
    elseif friction == 2
        mu = mu_s; % Static friction
    else
        error('Friction mode should be 1 (dynamic) or 2 (static)')
    end

    % Calculate maximum pressure area Pa (N/mm^2)
    if mode == 1 % Positive engagement force
        Pa = (F_clu * c - F_spr * (r_s1 + r_s2) - F_rbr * d) * sin(theta_a) / (b * r) / ...
            (a * (theta_c2 / 2 - theta_c1 / 2 + sin(2 * theta_c1) / 4 - sin(2 * theta_c2) / 4) - ...
            mu * (r * (cos(theta_c1) - cos(theta_c2)) - a * (cos(theta_c1) ^ 2 - cos(theta_c2) ^ 2) / 2));
    elseif mode == 2 % Negative engagement force
        Pa = (F_clu * c - F_spr * (r_s1 + r_s2) - F_rbr * d) * sin(theta_a) / (b * r) / ...
            (a * (theta_c2 / 2 - theta_c1 / 2 + sin(2 * theta_c1) / 4 - sin(2 * theta_c2) / 4) + ...
            mu * (r * (cos(theta_c1) - cos(theta_c2)) - a * (cos(theta_c1) ^ 2 - cos(theta_c2) ^ 2) / 2));
    else
        error('Clutch mode should be 1 or 2')
    end

    Pa(omega1 < omega0) = 0;

    % Calculate transmitted torque T
    T = n * mu * r ^ 2 * b * Pa / sin(theta_a) * (cos(theta_c1) - cos(theta_c2)) / 1000; % Transmitted torque (N-m)

end
