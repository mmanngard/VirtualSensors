clear all
close all

%% PROVIDE PROJECT ID
ProjectID = 'TestBench';

disp(' ')
disp('------------------------')
disp(ProjectID)
addpath([pwd '\fcns'])
addpath([pwd '\models\' ProjectID])
addpath([pwd '\data\IceExcitations12102020\'])

%% Create model
Model = loadProjectData(ProjectID); %load project data
Model.damp = [0.0030 0 0 0 0 0 0 0 0 0 0 0.0031 0 0 0 0 0.0031 0 0 0 0 0.24]; %identified damping parameters, d12=d17.

%% #################### INPUT PARAMETERS ####################
% Sensor parameters
Model.RPMSensorLocations = [7 8];       %rpm sensor locations (index conted from driving motor)
Model.TorqueSensorLocations = [9];      %torque sensor locations (index conted from driving motor)
Model.AngularDevSensorLocations = [];
Model.nSensors = length(Model.RPMSensorLocations) + length(Model.TorqueSensorLocations) + length(Model.AngularDevSensorLocations);   %number of puls/rev

% KALMAN FILTER DESIGN PARAMETERS
Model.Lag = 10;
Model.R   = diag([0.05 0.1 0.2]);       %measurement covariance R = E{v*v'}
Model.Q = 0.01*eye(Model.nStates);

%%#################### LOAD DATA ####################
clear Data
lp = 500;
Data = ImportExperimentData('Ice_excitation1.csv','Ice_excitation1_motor.csv',Model,lp);
Data.AngularDevMeasurements = [];

disp('Data loaded')

%% #################### Clear redundant variables ####################
clearvars -except Model KalmanFilter Data par

%% #################### Create driveline model ####################
Model.sys_sym = ss_sym(Model);
Model.c(7) = Model.c(7); 
G = ss_ini(Model);                          %continuous-time model
Gd = discreteSystem(G,1/Model.Fs,'zoh');    %discrete-time model

%% #################### FILTER DESIGN ####################
clear Pz P P_u P_t P_z P_v
%#######    INPUT MODEL    ######
% cf. Sec 3.2., Manngard et al. (2022). Torque estimation in marine propulsion systems, MSSP.
%------------------------------------------
% RANDOM WALK + WHITE NOISE MODEL 
% u(k) = u1(k) + u2(k)
% u1(k) = a*u1(k-1) + e1(k)
% u2(k) = e2(k);
%
% ------------- INPUT MODEL DESIGN CASES ---------
% (1) a = 1, sigma_1^2 = 1, sigma_2^2 = 0.1
% (2) a = 1, sigma_1^2 = 1, sigma_2^2 = (10/3)^2
% (3) a = 0, sigma_1^2 = 0, sigma_2^2 = (10/3)^2
%
inputDesignCase = 1; %<- SELECT
%-------------------------------------------------
[Ge,sigma_m,mu_m] = inputDesignCases(inputDesignCase, Model, Gd);

% Kalman filter design
[KF,Pz,K] = KalmanFilterDesign(Ge);

%% ######## Estimation ########
y = [Data.AngularSpeedMeasurements;Data.TorqueMeasurements];
u = mu_m*ones(1,length(y));
Y = [u;y];

KF2 = ss( KF.A, [Ge.Bm KF.B], KF.C, zeros( size(KF.C,1), size(KF.B,2)+1 ) , 1 );
zhat = lsim(KF2,Y);

Data.TorqueMotorEstimate = u' + zhat(:,1);
Data.TorquePropellerEstimate = zhat(:,2);
Data.TorqueEstimates = zhat(:,3:G.nMasses+1);
Data.AngularSpeedEstimates = zhat(:,G.nMasses+2:end);

%% ######## Naive estimates (dynamics ignored) ########
gearLossCorrection = mean(Data.Torques(1:2000,9))/mean(Data.Torques(1:2000,19));
Data.TorqueNaiveEstimate = Data.TorqueMeasurements/gearLossCorrection;

%% ######## PLOT RESULTS ########
% close all
Fs = Model.Fs;
Torque = [9 19];
nTorque = length(Torque);

RPM = [7 8 14 15 22]; %corrected 02/11/2020
nRPM = length(RPM);

xlimits = [6.5 8];

% ------ PLOT INPUTS ------
ylimits={[1 6],[-5 16]};
figure(1),
subplot(2,1,1), hold on
plot(Data.Time+10/1000-xlimits(1),Data.TorqueMotor(:,1),'-o','color',[0.7 0.7 0.7],'DisplayName','Data','linewidth',1,'markersize',1, 'MarkerFaceColor',[0.7 0.7 0.7])
plot(Data.Time(1:end-Model.Lag(end)+1)-xlimits(1), Data.TorqueMotorEstimate(Model.Lag(end):end),'color',[0.8500, 0.3250, 0.0980],'linewidth',1,'DisplayName','AKF')
xlim(xlimits-xlimits(1))
ylim(ylimits{1})
xlabel('Time (s)','interpreter','latex','FontSize',8)
ylabel('Torque (Nm)','interpreter','latex','FontSize',8)
legend('orientation','horizontal','interpreter','latex','Location','northoutside')
set(gca,'FontSize',7,'TickLabelInterpreter','latex')

TorquePropellerDamping = Model.damp(end)*Data.AngularSpeedEstimates(:,end);

subplot(2,1,2), hold on
plot(Data.Time-xlimits(1) + 0*10/1000,Data.TorquePropeller(:,1),'-o','color',[0.7 0.7 0.7],'DisplayName','Data','linewidth',1,'markersize',1, 'MarkerFaceColor',[0.7 0.7 0.7])
plot(Data.Time(1:end-Model.Lag(end)+1)-xlimits(1),Data.TorquePropellerEstimate(Model.Lag(end):end),'color',[0.8500, 0.3250, 0.0980],'linewidth',1,'DisplayName','AKF')
xlim(xlimits-xlimits(1))
ylim(ylimits{2})
xlabel('Time (s)','interpreter','latex','FontSize',8)
ylabel('Torque (Nm)','interpreter','latex','FontSize',8)
% title('Loading motor')
legend('orientation','horizontal','interpreter','latex','Location','northoutside')
set(gca,'FontSize',7,'TickLabelInterpreter','latex')

fig_u = gcf;
fig_u.Units = 'centimeters';
fig_u.Position = [0 2 9 9];
fig_u.Clipping = 'off';

% ------ PLOT velocities ------
ylims = {[3130 3350]/9.55,[3130 3350]/9.55,[1040 1120]/9.55,[1040 1120]/9.55,[260 280]/9.55};
figure(3),
for k=1:nRPM
h(k)=subplot(nRPM,1,k); hold on

t = Data.Time-xlimits(1);
x = Data.AngularSpeeds(:,RPM(k));
that = Data.Time(1:end-Model.Lag(end)+1)-xlimits(1);
xhat = Data.AngularSpeedEstimates(Model.Lag(end):end,RPM(k));

plot(t,x,'bo-','DisplayName','Data','markersize',1, 'MarkerFaceColor','b')
plot(that,xhat,'r','DisplayName',['Estimate, $i=$ ' num2str(RPM(k))])
xlim(xlimits-xlimits(1))
ylim(ylims{k})
legend('orientation','horizontal','interpreter','latex','FontSize',7)
xlabel('Time (s)','interpreter','latex','FontSize',7)
ylabel('Angular Velocity (rad/s)','interpreter','latex','FontSize',7)
% title(['Encoder ' num2str(k)])
set(gca,'FontSize',7,'TickLabelInterpreter','latex')

RMSE(k) = rms(x(xlimits(1)*Fs:xlimits(2)*Fs) - xhat(xlimits(1)*Fs:xlimits(2)*Fs));
end
%center last odd subplot
pos = get(h,'Position');
new = mean(cellfun(@(v)v(1),pos(1:2)));
set(h(5),'Position',[new,pos{end}(2:end)])
hold on
fig_rpm = gcf;
fig_rpm.Units = 'centimeters';
fig_rpm.Position = [30 2 9 20];
fig_rpm.Clipping = 'off';

% ######## PLOT TORQUES ############
ylims = {[0 5],[0 35]};
figure(2),
for k=1:nTorque
subplot((nTorque),1,k), hold on    
    
t = Data.Time-xlimits(1);
x = Data.Torques(:,Torque(k));
that = Data.Time(1:end-Model.Lag(end)+1)-xlimits(1);
xhat = Data.TorqueEstimates(Model.Lag(end):end,Torque(k));

RMSE(k) = rms(x(xlimits(1)*Fs:xlimits(2)*Fs) - xhat(xlimits(1)*Fs:xlimits(2)*Fs));
RMSE_naive(k) = rms(x(xlimits(1)*Fs:xlimits(2)*Fs) - Data.TorqueNaiveEstimate(xlimits(1)*Fs:xlimits(2)*Fs)');
plot(t,x,'-o','color',[0.7 0.7 0.7],'DisplayName','Data','linewidth',1,'markersize',1, 'MarkerFaceColor',[0.7 0.7 0.7])
plot(that,xhat,'color',[0.8500, 0.3250, 0.0980],'linewidth',1,'DisplayName','AKF')

xlim(xlimits-xlimits(1))
ylim(ylims{k})
legend('orientation','horizontal','interpreter','latex','Location','northoutside')
xlabel('Time (s)','interpreter','latex','FontSize',8)
ylabel('Torque (Nm)','interpreter','latex','FontSize',8)
set(gca,'FontSize',8,'TickLabelInterpreter','latex')
grid off
end
fig_t = gcf;
fig_t.Units = 'centimeters';
fig_t.Position = [0 2 9 9];
fig_t.Clipping = 'off';

%--------- Plot setpoint ---------
figure(7), hold on
plot(Data.Time-xlimits(1)+7/1000, Data.Setpoints(:,2),'k','DisplayName','Setpoint')
xlim(xlimits-xlimits(1))
ylim([-1 12])
set(gca,'FontSize',7,'TickLabelInterpreter','latex')
grid off
legend('orientation','horizontal','interpreter','latex')
xlabel('Time (s)','interpreter','latex','FontSize',7)
ylabel('Torque (Nm)','interpreter','latex','FontSize',7)
fig_sp = gcf;
fig_sp.Units = 'centimeters';
fig_sp.Position = [0 0 9 5];
fig_sp.Clipping = 'off';
