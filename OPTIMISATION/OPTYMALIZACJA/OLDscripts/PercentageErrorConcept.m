clear;
clc;
%LOAD CSV FILE FROM MOVEMENT OF THE REAL SERVO
RealFile = '/home/jaku6m/Desktop/Plots/RealMX64Plots/Trajectory2.csv';
RealValues = readmatrix(RealFile);

%Trim the data that is equal to zero
    %find the first non zero velocity value:
    Real_velocity_ms = RealValues(:, 3);
    first_nonzero=find(Real_velocity_ms,1);
RealValues=trimdata(RealValues,length(RealValues)-first_nonzero,Side="leading");
%Adjust the starting time to zero
DeltaTime=RealValues(1,1);
RealValues(:,1)=RealValues(:,1)-DeltaTime;

% Extract columns into separate variables
Real_time_ms = RealValues(:, 1);       % Time in milliseconds
Real_position_rad = RealValues(:, 2);  % Position in radians
Real_velocity_ms = RealValues(:, 3);   % Velocity in m/s
Real_current_mA = RealValues(:, 4);    % Current in mA
% plot(Real_time_ms,Real_position_rad,'DisplayName','Real position')
% hold on
% plot(Real_time_ms,Real_velocity_ms,'DisplayName','Real Velocity')
% Real_current_mA = RealValues(:, 4)/1000; %current transformed to A for better visibility on chart
% hold on
% plot(Real_time_ms,Real_current_mA)




%LOAD CSV FILE FROM MOVEMENT OF THE GAZEBO SERVO
Gazebo_position_File = "/home/jaku6m/Desktop/Plots/GazeboMX64Plots/Trajectory2/Plot Name-one_dynamixel_simulation_joint1:0_position.csv";
Gazebo_velocity_File = "/home/jaku6m/Desktop/Plots/GazeboMX64Plots/Trajectory2/Plot Name-one_dynamixel_simulation_joint1:0_velocity.csv";
% Read time and position from the position CSV file
Gazebo_position_data = readmatrix(Gazebo_position_File);  % Read entire matrix from position CSVGazeboValues
% Read velocity from the velocity CSV file
Gazebo_velocity_data = readmatrix(Gazebo_velocity_File);  % Read entire matrix from velocity CSV
% Pass the files to one array:
    %Trim inconsistent lengths of csv files
    min_length = min(length(Gazebo_position_data), length(Gazebo_velocity_data)); 
    max_length = max(length(Gazebo_position_data), length(Gazebo_velocity_data));
    if length(Gazebo_velocity_data) == max_length
        Gazebo_velocity_data = trimdata(Gazebo_velocity_data,min_length,Side="leading");
    else
        Gazebo_position_data = trimdata(Gazebo_position_data,min_length,Side="leading");
    end
Gazebo_time_ms = Gazebo_position_data(:, 1)*1000;                  % Extract time from first column and convert to ms
Gazebo_position_rad = Gazebo_position_data(:, 2);                 % Extract position from second column
Gazebo_velocity_ms = Gazebo_velocity_data(:, 2);                  % Extract velocity from second column
GazeboValues=[Gazebo_time_ms,Gazebo_position_rad,Gazebo_velocity_ms];

%Trim the data that is equal to zero
    %find the first non zero velocity value:
    Gazebo_first_nonzero=find(Gazebo_velocity_ms > 0.04 | Gazebo_velocity_ms < -0.04,1);
    GazeboValues=trimdata(GazeboValues,length(GazeboValues)-Gazebo_first_nonzero,Side="leading");
    %Adjust the starting time to zero
    GazeboDeltaTime=GazeboValues(1,1);
    GazeboValues(:,1)=ceil(GazeboValues(:,1)-GazeboDeltaTime);

%Update Vectors of GAZEBO time pos and vel:
Gazebo_time_ms = GazeboValues(:, 1);                     % Extract time from first column and convert to ms
Gazebo_position_rad = GazeboValues(:, 2);                 % Extract position from second column
Gazebo_velocity_ms = GazeboValues(:, 3);                  % Extract velocity from third column


%COMPARE TIME VECTORS FOR GAZEBO AND REAL AND THROW AWAY UNNECESSARY
%MEASUREMENTS FROM GAZEBO MEASUREMENTS
    %what index should be deleted?
    whatrowstodelete=1+setdiff(Gazebo_time_ms,Real_time_ms);
    GazeboValues(whatrowstodelete,:) = [];
    %Update Vectors of GAZEBO time pos and vel:
    Gazebo_time_ms = GazeboValues(:, 1);                     % Extract time from first column and convert to ms
    Gazebo_position_rad = GazeboValues(:, 2);                 % Extract position from second column
    Gazebo_velocity_ms = GazeboValues(:, 3);                  % Extract velocity from third column

%If there are Measurements in reality that doesnt exist in gazebo throw
%them away
RealValues=trimdata(RealValues,length(GazeboValues));
%Actualize values
Real_time_ms = RealValues(:, 1);       % Time in milliseconds
Real_position_rad = RealValues(:, 2);  % Position in radians
Real_velocity_ms = RealValues(:, 3);   % Velocity in m/s
Real_current_mA = RealValues(:, 4);    % Current in mA

    %PLOTS
    plot(Real_time_ms,Real_position_rad,'DisplayName','Real position')
    hold on
    plot(Real_time_ms,Real_velocity_ms,'DisplayName','Real Velocity')
    hold on
    plot(Gazebo_time_ms,Gazebo_position_rad,'DisplayName', 'Gazebo Position')
    hold on
    plot(Gazebo_time_ms,Gazebo_velocity_ms,'DisplayName','Gazebo Velocity')
    xlabel("Czas [ms]")
    ylabel("Prędkość [m/s]/Pozycja [rad]")
    title("Porównanie pozycji i prędkości z symulacji do przebiegów rzeczywistego serwomechanizmu")
    legend


%Count the mean error squared
position_errorsquared=0;
velocity_errorsquared=0;
PositionPercentError = zeros(length(GazeboValues), 1);  %  array to store position error percentages
VelocityPercentError = zeros(length(GazeboValues), 1);  %  array to store velocity error percentages

%MUSZE CIEBIE STESTOWAC
Gazebo_position_rad=Real_position_rad+0.1;
Gazebo_velocity_ms=Real_velocity_ms+0.1;

for i = 1:length(RealValues)
   position_errorsquared=(Gazebo_position_rad(i)-Real_position_rad(i)).^2+position_errorsquared;
   velocity_errorsquared=(Gazebo_velocity_ms(i)-Real_velocity_ms(i)).^2+ velocity_errorsquared;
   PositionPercentError(i)=abs((Gazebo_position_rad(i)-Real_position_rad(i))/Real_position_rad(i));
   VelocityPercentError(i)=abs((Gazebo_velocity_ms(i)-Real_velocity_ms(i))/Real_velocity_ms(i));
end
% Average errors over all samples (rows)
PositionAverageError = mean(PositionPercentError,"omitnan");  % Ignore NaN values in average calculation
VelocityAverageError = mean(VelocityPercentError,"omitnan");  % Ignore NaN values in average calculation

disp(['Błąd procentowy pozycji: ', num2str(PositionAverageError)]);
disp(['Błąd procentowy prędkości: ', num2str(VelocityAverageError)]);
plot(Gazebo_time_ms,PositionPercentError)
hold on
plot(Gazebo_time_ms,VelocityPercentError)
disp(['Błąd średniokwadratowy pozycji: ', num2str(position_errorsquared)]);
disp(['Błąd średniokwadratowy prędkości: ', num2str(velocity_errorsquared)]);
%Add annotation to the chart    
errorText = sprintf(['Błąd procentowy pozycji: %.4f\n\n', ...
                     'Błąd procentowy prędkości: %.4f\n\n', ...
                     'Błąd średniokwadratowy pozycji: %.4f\n\n', ...
                     'Błąd średniokwadratowy prędkości: %.4f\n'], ...
                     PositionAverageError, VelocityAverageError, position_errorsquared, velocity_errorsquared);
annotation('textbox',[0.905 .5 .1 .2],'String',errorText,'EdgeColor','none')