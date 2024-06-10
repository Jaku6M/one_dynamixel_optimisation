function [sum_position_errorsquared, sum_velocity_errorsquared] = DynamicErrorAutomatic(RealFile, Gazebo_raw_File, params)
    %LOAD CSV FILE FROM MOVEMENT OF THE REAL SERVO
    % RealFile = '/home/jaku6m/Desktop/Plots/RealMX28Plots/Trajectory2_9secTrajectorySTART0.csv';
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

    % UNCOMMENT IF YOU WANT TO PLOT REAL VALUES
    % plot(Real_time_ms,Real_position_rad,'DisplayName','Real position')
    % hold on
    % plot(Real_time_ms,Real_velocity_ms,'DisplayName','Real Velocity')
    % Real_current_mA = RealValues(:, 4)/1000; %current transformed to A for better visibility on chart
    % hold on
    % plot(Real_time_ms,Real_current_mA)
    
   
    
    
    %LOAD CSV FILE FROM MOVEMENT OF THE GAZEBO SERVO
    % Gazebo_raw_File = "/home/jaku6m/Desktop/OPTYMALIZACJA/OptcsvGazeboFiles/feedback_data.csv";
    % Read time and position from the position CSV file
    Gazebo_raw_data = readmatrix(Gazebo_raw_File);  % Read entire matrix from CSVGazeboValues
    
    Gazebo_time_ms = Gazebo_raw_data(:, 1)*1000+Gazebo_raw_data(:, 2)/1000000;                  % Extract time from first column and convert to ms
    Gazebo_position_rad = Gazebo_raw_data(:, 5);                 % Extract position from fifth column
    Gazebo_velocity_ms = Gazebo_raw_data(:, 6);                  % Extract velocity from sixth column
    GazeboValues=[Gazebo_time_ms,Gazebo_position_rad,Gazebo_velocity_ms];
    
    %Trim the data that is equal to zero
        %find the first non zero velocity value (NOT WORKING CONSISTENTLY):
        % Gazebo_first_nonzero=find(Gazebo_velocity_ms > 0.05 | Gazebo_velocity_ms < -0.05,1);
        
        %WORKS FOR TRAJECTORY2:
        %Find number of element in vector of gazebo positions that is first to be the closest
        %one in terms of value to the real position vector values
        % Gazebo_first_nonzero=find(abs(Gazebo_position_rad-Real_position_rad(1)) < 0.001,1);

        %WORKS FOR TRAJECTORY3,5,8:
        Gazebo_first_nonzero=find(Gazebo_velocity_ms > 0.05 | Gazebo_velocity_ms < -0.05,1);
    %sometimes the discrepancies are so big that gazebo doesnt have element
    %that is similar to real vector, if this occurs set error on big value
  if isempty(Gazebo_first_nonzero)  
        sum_position_errorsquared=1000;
        sum_velocity_errorsquared=1000;
  else
        GazeboValues=trimdata(GazeboValues,length(GazeboValues)-Gazebo_first_nonzero,Side="leading");
        %Adjust the starting time to zero
        GazeboDeltaTime=GazeboValues(1,1);
        GazeboValues(:,1)=GazeboValues(:,1)-GazeboDeltaTime;
        for j = 1:length(GazeboValues)
            GazeboValues(j,1) = j-1;
        end
    
    %Update Vectors of GAZEBO time pos and vel:
    Gazebo_time_ms = GazeboValues(:, 1);                     % Extract time from first column and convert to ms
    Gazebo_position_rad = GazeboValues(:, 2);                 % Extract position from second column
    Gazebo_velocity_ms = GazeboValues(:, 3);                  % Extract velocity from third column
    
    
    %COMPARE TIME VECTORS FOR GAZEBO AND REAL AND THROW AWAY UNNECESSARY
    %MEASUREMENTS FROM GAZEBO MEASUREMENTS
        %what index should be deleted?
        whatrowstodelete=1+setdiff(GazeboValues(:, 1),RealValues(:, 1));
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
        title("Rzeczywiste i symulowane przebiegi")
        legend
    
    
    %Count the mean error squared
    position_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
    velocity_errorsquared=zeros(length(GazeboValues), 1);  %  array to store position error percentages
    for i = 1:length(RealValues)
       position_errorsquared(i)=(Gazebo_position_rad(i)-Real_position_rad(i)).^2;
       velocity_errorsquared(i)=(Gazebo_velocity_ms(i)-Real_velocity_ms(i)).^2;
    end
    
    sum_position_errorsquared=sum(position_errorsquared);
    sum_velocity_errorsquared=sum(velocity_errorsquared);
    
    %Error plots
    % plot(Gazebo_time_ms,position_errorsquared,'g', 'DisplayName','position error')
    % hold on
    % plot(Gazebo_time_ms,velocity_errorsquared,'b', 'DisplayName','velocity error')


    % disp(['Błąd średniokwadratowy pozycji: ', num2str(sum_position_errorsquared)]);
    % disp(['Błąd średniokwadratowy prędkości: ', num2str(sum_velocity_errorsquared)]);
    %Add annotation to the chart    
    errorText = sprintf(['Błąd dynamiczny pozycji: %.4f\n\n', ...
                         'Błąd dynamiczny prędkości: %.4f\n'], ...
                         sum_position_errorsquared, sum_velocity_errorsquared);
    annotation('textbox',[0.905 .5 .1 .2],'String',errorText,'EdgeColor','none')
    % Convert params to a string
    paramsString = sprintf('Parametry: %.4f, %.4f, %.4f %.4f, %.4f, %.4f, %.4f %.4f, %.4f', params);
    % Create annotation on the lower side of the chart
    annotation('textbox', [0.1, 0.1, 0.9, 0.1], 'String', paramsString, 'EdgeColor', 'none');

    % Save the current figure
    persistent counter; % Declare counter as a persistent variable
    if isempty(counter)
        counter = 1; % Initialize counter if it's empty
    else
        counter = counter + 1; % Increment counter
    end
    
    %Save every figure with its name and number
    current_fig = gcf(); % Get handle to the current figure
    save_dir = '/home/jaku6m/Desktop/OPTYMALIZACJA/Trajectory3OptimizationChartsMX64AR/';
    save_name = sprintf('%splot_%d.png', save_dir, counter);
    saveas(current_fig, save_name);
    % Close the figure after saving
    pause(5);
    close(current_fig);
  end
end