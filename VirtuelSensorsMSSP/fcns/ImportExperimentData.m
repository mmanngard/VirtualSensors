function Data = ImportExperimentData(name,nameMotor,Model,lp)
%IMPORTCSVDATA(name,Fs,lowpass), where name is the name of the csv file, Fs
%the sampling frequency in Hz and lowpass logical indicator wether to
%lowpass filter encoder data or not.

    T = readtable(name);
    
    
    Time = table2array(T(:,1));
    Fs = round(1/diff(Time(1:2)))
    Angles = table2array(T(:,2:6))/360*2*pi; % angles in rad
    Torques = table2array(T(:,7:8));
    
%     lp = lp/Fs;
    
    %lowpass encoder data
    if lp~=0
%         AngularVelocities = lowpass(diff(Angles)./diff(Time),lp);
%         Torques = lowpass(Torques,lp);
        AngularVelocities = lowpass(diff(Angles)./diff(Time),lp,Fs,'ImpulseResponse','iir','Steepness',0.98);
        Torques = lowpass(Torques,lp,Fs,'ImpulseResponse','iir','Steepness',0.98);
    else
%         AngularVelocities = diff(Angles)/(1/Fs);
        AngularVelocities = diff(Angles)./diff(Time);
    end
    AngularVelocities = [AngularVelocities;AngularVelocities(end,:)];
    
    
    %% load motor data
    
    if ~isempty(nameMotor)
        T = readtable(nameMotor);
        TimeInputs = table2array(T(:,1));
        Setpoints = table2array(T(:,[2 4])); 
        Inputs = table2array(T(:,[3 5]));
            if lp~=0
                Inputs = lowpass(Inputs,lp,Fs,'ImpulseResponse','iir','Steepness',0.98);
%                 Inputs = lowpass(Inputs,lp);
            end
    end
    
    %% resample data for uniform sampling
    %input torques and the rest of measurements are not on the same clock.
    Tstart = max(Time(1),TimeInputs(1));
    Tend = min(Time(end),TimeInputs(end));
    
    TimeUniform = Tstart:1/Fs:Tend;
    
    AngularVelocities = interp1(Time,AngularVelocities,TimeUniform);
    Torques = interp1(Time,Torques,TimeUniform);
    
    Setpoints = [1 8].*interp1(TimeInputs,Setpoints,TimeUniform);
    Inputs = [1 8].*interp1(TimeInputs,Inputs,TimeUniform);
    
%     AngularVelocities = interp1(Time,AngularVelocities,TimeInputs); %velocities to input timestamps to input
%     Torques = interp1(Time,Torques,TimeInputs); %velocities to input timestamps to input
    
%     TimeUniform = [0:1/Fs:TimeInputs(end)-TimeInputs(1)]';

%     AngularVelocities = interp1(TimeInputs-TimeInputs(1),AngularVelocities,TimeUniform);
%     Torques = interp1(TimeInputs-TimeInputs(1),Torques,TimeUniform);
%     Setpoints = [1 8].*interp1(TimeInputs-TimeInputs(1),Setpoints,TimeUniform);
%     Inputs = interp1(TimeInputs-TimeInputs(1),Inputs,TimeUniform);
    
%     
    %%
    Data.Fs = Fs;
    Data.Time = TimeUniform - TimeUniform(1);
    Data.TorqueMotor = Inputs(:,1);
    Data.TorquePropeller = Inputs(:,2);
    
    Data.Torques = NaN(length(Torques),Model.nMasses-1); Data.Torques(:,[9 19]) = Torques;
    Data.AngularSpeeds = NaN(length(AngularVelocities),Model.nMasses-1);  Data.AngularSpeeds(:,[7 8 14 15 22]) = AngularVelocities; %corrected sensor locations 11/02/2020
    Data.Setpoints = Setpoints;
    
    Data.AngularSpeedMeasurements = Data.AngularSpeeds(:,Model.RPMSensorLocations)';
    Data.TorqueMeasurements = Data.Torques(:,Model.TorqueSensorLocations)';
    

    %%
    Data.AngularSpeedMeasurements(isnan(Data.AngularSpeedMeasurements)) = 0;
    Data.TorqueMeasurements(isnan(Data.TorqueMeasurements)) = 0;

    %manual correction for syncing errors
    Data.AngularSpeedMeasurements(2,:) = [Data.AngularSpeedMeasurements(2,2:end) Data.AngularSpeedMeasurements(2,end)*ones(1,1) ];
    
%     %% resample data for uniform sampling
%     %input torques and the rest of measurements are not on the same clock.
%     
%       
%     
%     AngularVelocities = interp1(Time,AngularVelocities,TimeInputs); %velocities to input timestamps to input
%     Torques = interp1(Time,Torques,TimeInputs); %velocities to input timestamps to input
%     
%     TimeUniform = [0:1/Fs:TimeInputs(end)-TimeInputs(1)]';
%     AngularVelocities = interp1(TimeInputs-TimeInputs(1),AngularVelocities,TimeUniform);
%     Torques = interp1(TimeInputs-TimeInputs(1),Torques,TimeUniform);
%     Setpoints = [1 8].*interp1(TimeInputs-TimeInputs(1),Setpoints,TimeUniform);
%     Inputs = interp1(TimeInputs-TimeInputs(1),Inputs,TimeUniform);
%     
% %     
%     %%
%     Data.Fs = Fs;
%     Data.Time = TimeUniform;
%     Data.TorqueMotor = Inputs(:,1);
%     Data.TorquePropeller = 8*Inputs(:,2);
%     Data.Torques = NaN(length(Torques),Model.nMasses-1); Data.Torques(:,[9 19]) = Torques;
%     Data.AngularSpeeds = NaN(length(AngularVelocities),Model.nMasses-1);  Data.AngularSpeeds(:,[7 8 14 16 22]) = AngularVelocities;
%     Data.Setpoints = Setpoints;
%     
%     Data.AngularSpeedMeasurements = Data.AngularSpeeds(:,Model.RPMSensorLocations)';
%     Data.TorqueMeasurements = Data.Torques(:,Model.TorqueSensorLocations)';
%     
% 
%     %%
%     Data.AngularSpeedMeasurements(isnan(Data.AngularSpeedMeasurements)) = 0;
%     Data.TorqueMeasurements(isnan(Data.TorqueMeasurements)) = 0;
    
    

    
    
    
    
end

