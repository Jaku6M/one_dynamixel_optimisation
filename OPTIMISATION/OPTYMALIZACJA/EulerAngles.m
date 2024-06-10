clc;
clear;
% Przekonwertowanie kątów na radiany
thetaX_deg = -90; % Kąt w stopniach
thetaY_deg = 90; % Kąt w stopniach
thetaZ_deg = 45;
thetaX_rad = deg2rad(thetaX_deg); % Kąt w radianach
thetaY_rad = deg2rad(thetaY_deg); % Kąt w radianach
thetaZ_rad = deg2rad(thetaZ_deg);
% Tworzenie macierzy obrotu wokół osi Y 
Ry = [cos(thetaY_rad) 0 sin(thetaY_rad);
      0 1 0;
      -sin(thetaY_rad) 0 cos(thetaY_rad)];

% Tworzenie macierzy obrotu wokół osi X
Rx = [1 0 0;
      0 cos(thetaX_rad) -sin(thetaX_rad);
      0 sin(thetaX_rad) cos(thetaX_rad)];

% Tworzenie macierzy obrotu wokół osi Z 
Rz = [cos(thetaZ_rad) -sin(thetaZ_rad) 0;
      sin(thetaZ_rad) cos(thetaZ_rad) 0;
      0 0 1];

% Mnożenie macierzy 
R = Rx * Rz;

% Wyświetlenie wynikowej macierzy
disp('Wynikowa macierz obrotu R:');
disp(R);

% Obliczanie kątów Eulera (roll, pitch, yaw) z macierzy obrotu R
% zakładają kolejność Yaw-Pitch-Roll (Z-Y-X)
% phi = atan2(R(2,1), R(1,1)); % Roll
% theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2)); % Pitch
% psi = atan2(R(3,2), R(3,3)); % Yaw

% Obliczanie kątów Eulera (roll, pitch, yaw) z macierzy obrotu R
% zakładają kolejność Roll-Pitch-Yaw (X-Y-Z)
psi = atan2(R(2,1), R(1,1)); % Yaw
theta = atan2(sqrt(R(3,1)^2 + R(3,2)^2), R(3,3)); % Pitch
phi = atan2(-R(3,2), R(3,1)); % Roll


% Wyświetlenie kątów Eulera
disp('Kąty Eulera (roll, pitch, yaw):');
disp([phi, theta, psi]);