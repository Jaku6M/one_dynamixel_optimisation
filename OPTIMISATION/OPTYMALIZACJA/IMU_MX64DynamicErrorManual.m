clear;
clc;
%LOAD CSV FILE FROM REAL IMU
RealFile = '/home/jaku6m/Desktop/Plots/RealIMU_MX64Plots/trajectory7.csv';
RealValues = readmatrix(RealFile);

%Trim the data that is near to zero
    % WORKS FOR TRAJECTORY 7
    % RealValues=trimdata(RealValues,length(RealValues)-20,Side="leading");
    % Real_Y_A = RealValues(:, 5);
    % first_nonzero=find(Real_Y_A > 0.7 | Real_Y_A < -1.0,1)-2;
    % WORKS FOR TRAJECTORY 10
    % Real_Y_A = RealValues(:, 5);
    % first_nonzero=find(Real_Y_A > 0.5 | Real_Y_A < -0.5,1)-2;
    % WORKS FOR TRAJECTORY 11
    % Real_Y_A = RealValues(:, 5);
    % first_nonzero=find(Real_Y_A > 1 | Real_Y_A < -1,1)-15;
    RealValues=trimdata(RealValues,length(RealValues)-first_nonzero,Side="leading");
%Adjust the starting time to zero
    DeltaTime=RealValues(1,7);
    RealValues(:,7)=RealValues(:,7)-DeltaTime;

% Extract columns into separate variables
Real_time_ms = RealValues(:, 7);       % Time in milliseconds
Real_X_G = RealValues(:, 1)*9.81;  % acceleration in G values multiplied to achieve m/s^2
Real_Y_G = RealValues(:, 2)*9.81;  % acceleration in G values multiplied to achieve m/s^2
Real_Z_G = RealValues(:, 3)*9.81;  % acceleration in G values multiplied to achieve m/s^2
Real_X_A = RealValues(:, 4); %velocity in degrees/s
Real_Y_A = RealValues(:, 5); %velocity in degrees/s
Real_Z_A = RealValues(:, 6); %velocity in degrees/s


%%FILTRATION
% Set window size for moving average filter
window_size = 10; % Adjust as needed based on the amount of noise
% Apply moving average filter to all axes
filteredAcceleration_X = movmean(Real_X_G, window_size);
filteredAcceleration_Y = movmean(Real_Y_G, window_size);
filteredAcceleration_Z = movmean(Real_Z_G, window_size);
filteredAngularVelocity_X = movmean(Real_X_A, window_size);
filteredAngularVelocity_Y = movmean(Real_Y_A, window_size);
filteredAngularVelocity_Z = movmean(Real_Z_A, window_size);
Real_X_G = filteredAcceleration_X;
Real_Y_G = filteredAcceleration_Y;
Real_Z_G = filteredAcceleration_Z;
Real_X_A = filteredAngularVelocity_X;
Real_Y_A = filteredAngularVelocity_Y;
Real_Z_A = filteredAngularVelocity_Z;


RealValues=[Real_X_G,Real_Y_G, Real_Z_G, Real_X_A, Real_Y_A, Real_Z_A, Real_time_ms];

%LOAD CSV FILE FROM MOVEMENT OF THE GAZEBO SERVO
Gazebo_X_G = "/home/jaku6m/Desktop/Plots/GazeboIMU_MX64Plots/Trajectory7/Plot Name-_gazebo_default_one_dynamixel_simulation_link2_MPU6050sensor_imu:p=_linear_acceleration_x.csv";
Gazebo_Y_G = "/home/jaku6m/Desktop/Plots/GazeboIMU_MX64Plots/Trajectory7/Plot Name-_gazebo_default_one_dynamixel_simulation_link2_MPU6050sensor_imu:p=_linear_acceleration_y.csv";
Gazebo_Z_G = "/home/jaku6m/Desktop/Plots/GazeboIMU_MX64Plots/Trajectory7/Plot Name-_gazebo_default_one_dynamixel_simulation_link2_MPU6050sensor_imu:p=_linear_acceleration_z.csv";
Gazebo_X_A = "/home/jaku6m/Desktop/Plots/GazeboIMU_MX64Plots/Trajectory7/Plot Name-_gazebo_default_one_dynamixel_simulation_link2_MPU6050sensor_imu:p=_angular_velocity_x.csv";
Gazebo_Y_A = "/home/jaku6m/Desktop/Plots/GazeboIMU_MX64Plots/Trajectory7/Plot Name-_gazebo_default_one_dynamixel_simulation_link2_MPU6050sensor_imu:p=_angular_velocity_y.csv";
Gazebo_Z_A = "/home/jaku6m/Desktop/Plots/GazeboIMU_MX64Plots/Trajectory7/Plot Name-_gazebo_default_one_dynamixel_simulation_link2_MPU6050sensor_imu:p=_angular_velocity_z.csv";
% Read Gazebo data from CSV files
Gazebo_XG_data = readmatrix(Gazebo_X_G);  
Gazebo_YG_data = readmatrix(Gazebo_Y_G);  
Gazebo_ZG_data = readmatrix(Gazebo_Z_G);  
Gazebo_XA_data = readmatrix(Gazebo_X_A);  
Gazebo_YA_data = readmatrix(Gazebo_Y_A); 
Gazebo_ZA_data = readmatrix(Gazebo_Z_A);
%Convert data to vectors:
Gazebo_time_ms = Gazebo_XG_data(:, 1)*1000;  
Gazebo_XGval = Gazebo_XG_data(:,2);
Gazebo_YGval = Gazebo_YG_data(:,2);
Gazebo_ZGval = Gazebo_ZG_data(:,2);
Gazebo_XAval = Gazebo_XA_data(:,2);
Gazebo_YAval = Gazebo_YA_data(:,2);
Gazebo_ZAval = Gazebo_ZA_data(:,2);
% Pass the files to one array:
GazeboValues=[Gazebo_time_ms,Gazebo_XGval,Gazebo_YGval,Gazebo_ZGval,Gazebo_XAval,Gazebo_YAval,Gazebo_ZAval];

%Trim the data that is equal to zero
    %find the first non zero velocity value:
    % WORKS FOR TRAJECTORY 11
    Gazebo_first_nonzero=find(Gazebo_YAval > 0.01 | Gazebo_YAval < -0.01,1)-2;
    GazeboValues=trimdata(GazeboValues,length(GazeboValues)-Gazebo_first_nonzero,Side="leading");
    %Adjust the starting time to zero
    GazeboDeltaTime=GazeboValues(1,1);
    GazeboValues(:,1)=GazeboValues(:,1)-GazeboDeltaTime;
    for j = 1:length(GazeboValues)
        GazeboValues(j,1) = (j-1)*50;
    end
%Update Vectors of GAZEBO 
Gazebo_time_ms = GazeboValues(:, 1);  
Gazebo_XGval = GazeboValues(:, 2); 
Gazebo_YGval = GazeboValues(:, 3); 
Gazebo_ZGval = GazeboValues(:, 4); 
Gazebo_XAval = GazeboValues(:, 5); 
Gazebo_YAval = GazeboValues(:, 6); 
Gazebo_ZAval = GazeboValues(:, 7); 

%COMPARE TIME VECTORS FOR GAZEBO AND REAL AND THROW AWAY UNNECESSARY
%MEASUREMENTS FROM GAZEBO MEASUREMENTS
    %what index should be deleted?
    whatrowstodelete=1+setdiff(GazeboValues(:, 1),RealValues(:, 7));
    GazeboValues(whatrowstodelete,:) = [];
    %Update Vectors of GAZEBO 
    Gazebo_time_ms = GazeboValues(:, 1);  
    Gazebo_XGval = GazeboValues(:, 2); 
    Gazebo_YGval = GazeboValues(:, 3); 
    Gazebo_ZGval = GazeboValues(:, 4); 
    Gazebo_XAval = GazeboValues(:, 5)*57.2957795; %conversion from rad/s to degres/second 
    Gazebo_YAval = GazeboValues(:, 6)*57.2957795; %conversion from rad/s to degres/second 
    Gazebo_ZAval = GazeboValues(:, 7)*57.2957795; %conversion from rad/s to degres/second 

%If there are Measurements in reality that doesnt exist in gazebo throw
%them away
RealValues=trimdata(RealValues,length(GazeboValues));
%Actualize values
Real_time_ms = RealValues(:, 7);       % Time in milliseconds
Real_X_G = RealValues(:, 1);  % acceleration in  m/s^2
Real_Y_G = RealValues(:, 2);  % acceleration in  m/s^2
Real_Z_G = RealValues(:, 3);  % acceleration in  m/s^2
Real_X_A = RealValues(:, 4); %velocity in degrees/s
Real_Y_A = RealValues(:, 5); %velocity in degrees/s
Real_Z_A = RealValues(:, 6); %velocity in degrees/s

            % % GAZEBO PLOT VALUES:
            % % Plots for gazebo values only
            % figure;
            % plot(Gazebo_time_ms, Gazebo_XGval, 'DisplayName', 'Gazebo X acceleration')
            % hold on
            % plot(Gazebo_time_ms, Gazebo_YGval, 'DisplayName', 'Gazebo Y acceleration')
            % hold on
            % plot(Gazebo_time_ms, Gazebo_ZGval, 'DisplayName', 'Gazebo Z acceleration')
            % hold on
            % legend
            % 
            % % Plot for values with 'A'
            % figure;
            % plot(Gazebo_time_ms, Gazebo_XAval, 'DisplayName', 'Gazebo X Velocity', 'LineWidth', 2, 'LineStyle', '--')
            % hold on
            % plot(Gazebo_time_ms, Gazebo_YAval, 'DisplayName', 'Gazebo Y Velocity')
            % hold on
            % plot(Gazebo_time_ms, Gazebo_ZAval, 'DisplayName', 'Gazebo Z Velocity')
            % hold on
            % legend

            % % REAL PLOT VALUES:
            % % Plot for values with 'G'
            % figure;
            % plot(Real_time_ms, Real_X_G, 'DisplayName', 'Real X acceleration')
            % hold on
            % plot(Real_time_ms, Real_Y_G, 'DisplayName', 'Real Y acceleration')
            % hold on
            % plot(Real_time_ms, Real_Z_G, 'DisplayName', 'Real Z acceleration')
            % hold on
            % legend
            % 
            % % Plot for values with 'A'
            % figure;
            % plot(Real_time_ms, Real_X_A, 'DisplayName', 'Velocity X axis')
            % hold on
            % plot(Real_time_ms, Real_Y_A, 'DisplayName', 'Velocity Y axis')
            % hold on
            % plot(Real_time_ms, Real_Z_A, 'DisplayName', 'Velocity Z axis')
            % hold on
            % legend


%Count the mean error squared
XG_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
YG_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
ZG_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
XA_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
YA_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
ZA_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
for i = 1:length(RealValues)
    XG_errorsquared(i)= (Gazebo_XGval(i)-Real_X_G(i)).^2;
    YG_errorsquared(i)=(Gazebo_YGval(i)-Real_Y_G(i)).^2;
    ZG_errorsquared(i)=(Gazebo_ZGval(i)-Real_Z_G(i)).^2;
    XA_errorsquared(i)=(Gazebo_XAval(i)-Real_X_A(i)).^2;
    YA_errorsquared(i)=(Gazebo_YAval(i)-Real_Y_A(i)).^2;
    ZA_errorsquared(i)=(Gazebo_ZAval(i)-Real_Z_A(i)).^2;
end

SUM_XG_errorsquared=sum(XG_errorsquared);
SUM_YG_errorsquared=sum(YG_errorsquared);
SUM_ZG_errorsquared=sum(ZG_errorsquared);
SUM_XA_errorsquared=sum(XA_errorsquared);
SUM_YA_errorsquared=sum(YA_errorsquared);
SUM_ZA_errorsquared=sum(ZA_errorsquared);
% plot(Gazebo_time_ms,position_errorsquared,'g', 'DisplayName','position error')
% hold on
% plot(Gazebo_time_ms,velocity_errorsquared,'b', 'DisplayName','velocity error')
% Wyświetlenie wartości w konsoli
disp(['Suma błędów kwadratu dla przyspieszenia X: ', num2str(SUM_XG_errorsquared)]);
disp(['Suma błędów kwadratu dla przyspieszenia Y: ', num2str(SUM_YG_errorsquared)]);
disp(['Suma błędów kwadratu dla przyspieszenia Z: ', num2str(SUM_ZG_errorsquared)]);
disp(['Suma błędów kwadratu dla prędkości X: ', num2str(SUM_XA_errorsquared)]);
disp(['Suma błędów kwadratu dla prędkości Y: ', num2str(SUM_YA_errorsquared)]);
disp(['Suma błędów kwadratu dla prędkości Z: ', num2str(SUM_ZA_errorsquared)]);

            % Real values with 'G' and Gazebo values with 'G'
            figure('Position', [100, 100, 1000, 800]);
            plot(Real_time_ms, Real_X_G, 'DisplayName', 'Real X acceleration')
            hold on
            plot(Real_time_ms, Real_Y_G, 'DisplayName', 'Real Y acceleration')
            hold on
            plot(Real_time_ms, Real_Z_G, 'DisplayName', 'Real Z acceleration')
            hold on
            plot(Gazebo_time_ms, Gazebo_XGval, 'DisplayName', 'Gazebo X acceleration')
            hold on
            plot(Gazebo_time_ms, Gazebo_YGval, 'DisplayName', 'Gazebo Y acceleration')
            hold on
            plot(Gazebo_time_ms, Gazebo_ZGval, 'DisplayName', 'Gazebo Z acceleration')
            hold on
            xlabel("Czas [ms]")
            ylabel("Przyspieszenie [m/s^2]")
            title("Przebiegi przyspieszenia IMU rzeczywiste i symulowane")
            legend
            % Adnotation
            errorText = sprintf(['Błędy dynamiczne:\n\n', ...
                                 'X: %.4f\n', ...
                                 'Y: %.4f\n', ...
                                 'Z: %.4f\n'], ...
                                 SUM_XG_errorsquared, SUM_YG_errorsquared, SUM_ZG_errorsquared);
            annotation('textbox',[0.905 .5 .1 .2],'String',errorText,'EdgeColor','none')
            % Real values with 'A'
            figure('Position', [100, 100, 1000, 800]);
            plot(Real_time_ms, Real_X_A, 'DisplayName', 'Velocity X axis')
            hold on
            plot(Real_time_ms, Real_Y_A, 'DisplayName', 'Velocity Y axis')
            hold on
            plot(Real_time_ms, Real_Z_A, 'DisplayName', 'Velocity Z axis')
            hold on
            % Gazebo values with 'A'
            plot(Gazebo_time_ms, Gazebo_XAval, 'DisplayName', 'Gazebo X Velocity', 'LineWidth', 2, 'LineStyle', '--')
            hold on
            plot(Gazebo_time_ms, Gazebo_YAval, 'DisplayName', 'Gazebo Y Velocity')
            hold on
            plot(Gazebo_time_ms, Gazebo_ZAval, 'DisplayName', 'Gazebo Z Velocity')
            hold on
            xlabel("Czas [ms]")
            ylabel("Prędkość [stopnie/s]")
            title("Przebiegi prędkości IMU rzeczywiste i symulowane")
            legend
            errorText = sprintf(['Błędy dynamiczne:\n\n', ...
                                 'X: %.4f\n', ...
                                 'Y: %.4f\n', ...
                                 'Z: %.4f\n'], ...
                                 SUM_XA_errorsquared, SUM_YA_errorsquared, SUM_ZA_errorsquared);
            annotation('textbox',[0.905 .5 .1 .2],'String',errorText,'EdgeColor','none')

