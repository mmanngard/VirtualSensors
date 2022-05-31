%General data US205S small scale FP

%% Motor and general variables

n=3221;                         % Nominal rotational speed
P=1150;                         % Nominal power [W]
T=P/(n/9.55);                   % Nominal torque [Nm]
% Old values which apparently are needed for Ice excitations. 
% Ice excitations have not been used; hence these are not changed.
z1=17;                          % Pinion tooth number
z2=42;                          % Wheel tooth number
z3=14;                          % Pinion tooth number
z4=38;                          % Wheel tooth number
Z=4;                            % Number of blades
To=0.2;                         % Over torque factor
Todelta=0.5;                    % rate of increase of electric engine moment 
                                % (1 = 100 %/s, 2 = 200 %/s)
Tice=100000;                    % Ice torque [Nm]

RPMdelta = 0.00;

prop_blades = 4;
x4 = 0.06/4;
x8 = 0.02/4;


Nmill=12;                       % Number of ice impacts during milling sequence
n_idle=0;                       % Engine idling speed 

W = 131.9469;

Imotor = 0.00068;

% Shaft material properties (steel s355)
Shaft_shear_modulus = 81*10^9;  % [Pa]
Shaft_density = 7850;           % [kg/m^3]

%% Coupling and shaft between motor and flexible coupling

Ic_mf = 1.1420e-04;             % Coupling's inertia [kgm^2]
Kc_mf = 1.9039e+05;             % Coupling's torsinal stiffness [Nm/rad]
C = 0.0056;                     % Unidimensional damping factor for shafts

Ds_mf = 0.016;                  % Diameter of shaft [m]
Ls_mf = 0.075;                  % Length of shaft [m]
S = 0.0056;                     % Unidimensional damping factor for shafts
% Calculate inertia and stiffness of shaft
[Ks_mf, Is_mf] = Shaft_inertia_stiffness(Ds_mf,Ls_mf,Shaft_shear_modulus,Shaft_density);

%% Flexible coupling and first shaft

D = 0.15;                       % Diameter of Flexible coupling's extra mass [m]
L = 0.02;                       % Length of Flexible coupling's extra mass [m]
% Calculate inertia and stiffness of extra mass
[Kem, Iem] = Shaft_inertia_stiffness(D,L,Shaft_shear_modulus,Shaft_density);

%EKL/010/C/16/16 and EKZ/010/C
Ifc_1 = 0.003*10^(-3);          % Flexible coupling's hub inertia [kgm^2]
Ifc_2 = 0.003*10^(-3);          % Flexible coupling's hub inertia [kgm^2]
Ifcm = 0.002*10^(-3);           % Flexible coupling's middle piece inertia [kgm^2]
Kfc = 90;                       % Flecxible coupling's torsinal stiffness [Nm/rad]
Cfc = 0.35;                     % Unidimensional damping factor for elastomer C

Ds1 = 0.006;                    % Diameter of shaft 1 [m]
Ls1 = 0.342; %0.3;              % Length of shaft 1 [m]
S1 = S;                         % Unidimensional damping factor for shafts
% Calculate inertia and stiffness of shaft 1
[Ks1, Is1] = Shaft_inertia_stiffness(Ds1,Ls1,Shaft_shear_modulus,Shaft_density);

%% Couplings and Torque Transducer 1

% BKE/20XX/15/16
C1 = C;                         % Unidimensional damping factor for couplings
Kc1 = 41.9*10^3;                % Torsinal stifness of coupling 1 [Nm/rad]
Ic1 = 0.05*10^(-3);             % Inertia of coupling 1 [kgm^2]

%DRBK-20
M1 = S1;                        % Unidimensional damping factor for Torque Transducer
Km1 = 5.4*10^3;                 % Torsinal stifness of torque transducer 1 [Nm/rad]
Im1 = 0.000013;                 % Inertia of torque transducer 1 [kgm^2]

% BKE/20XX/15/19
C2 = C1;                        % Unidimensional damping factor for couplings
Kc2 = Kc1;                      % Torsinal stifness of coupling 2 [Nm/rad]
Ic2 = Ic1;                       % Inertia of coupling 2 [kgm^2]

S2 = S1;                        % Unidimensional damping factor for shafts
Ds2 = 0.019;                    % Diameter of shaft 2 [m]
Ls2 = 0.085;                    % Length of shaft 2 [m]
% Calculate inertia and stiffness of shaft 2
[Ks2, Is2] = Shaft_inertia_stiffness(Ds2,Ls2,Shaft_shear_modulus,Shaft_density);

%% Upper gearbox (WPLE120-003-SSSA3AF-T24)

gug = 3;                        % Upper gearbox gear ratio
ug = C1;                        % Unidimensional damping factor for upper gearbox
Kug = 43.3*10^3;                % Torsinal stifness of upper gearbox [Nm/rad]
Iug = 0.3204*10^(-3);           % Inertia of upper gearbox [kgm^2]

I_upperpinion = Iug/(1+(1/gug)^2); %assume upper wheel and pinion have the same inertia
I_upperwheel = I_upperpinion;


%% Shafts and rigid couplings

% BKL/030/25/25
C3 = C1;                        % Unidimensional damping factor for coupling 3
Kc3 = 31*10^3;                  % Torsinal stifness of  coupling 3 [Nm/rad]
Ic3 = 0.12*10^(-3);             % Inertia of coupling 3 [kgm^2]

S3 = S1;                        % Unidimensional damping factor for shafts
Ds3 = 0.016;                    % Diameter of shaft 3 [m]
Ls3 = 0.456; %0.4;              % Length of shaft 3 [m]
% Calculate inertia and stiffness of shaft 3
[Ks3, Is3] = Shaft_inertia_stiffness(Ds3,Ls3,Shaft_shear_modulus,Shaft_density);

%BKL/030/25/19
C4 = C1;                        % Unidimensional damping factor for coupling 4
Kc4 = Kc3;                      % Torsinal stifness of  coupling 4 [Nm/rad]
Ic4 = Ic3;                      % Inertia of coupling 4 [kgm^2]

S4 = S1;                        % Unidimensional damping factor for shafts
Ds4 = 0.019;                    % Diameter of shaft 4 [m]
Ls4 = 0.085;                    % Length of shaft 4 [m]
% Calculate inertia and stiffness of shaft 4
[Ks4, Is4] = Shaft_inertia_stiffness(Ds4,Ls4,Shaft_shear_modulus,Shaft_density);

%% Lower gearbox (WPLE120-004-SSSA3AF-T24)

glg = 4;                        % Lower gearbox gear ratio
lg = C1;                        % Unidimensional damping factor for upper gearbox
Klg = 44.3*10^3;                % Torsinal stifness of upper gearbox [Nm/rad]
Ilg = 0.265*10^(-3);            % Inertia of upper gearbox [kgm^2]

I_lowerpinion = Ilg/(1+(1/glg)^2); %assume upper wheel and pinion have the same inertia
I_lowerwheel = I_lowerpinion;

%% Coupling, Torque Transducer 2 and propeller

% BKE/200XX/24/25
C5 = C1;                        % Unidimensional damping factor for couplings
Kc5 = 138*10^3;                 % Torsinal stifness of coupling 5 [Nm/rad]
Ic5 = 0.18*10^(-3);             % Inertia of coupling 5 [kgm^2]

% DRBK-50
M2 = M1;                        % Unidimensional damping factor for Torque Transducer 2
Km2 = 20*10^3;                  % Torsinal stifness of torque transducer 2 [Nm/rad]
Im2 = 0.00004;                  % Inertia of torque transducer 2 [kgm^2]

% BKE/200XX/24/25
C6 = C1;                        % Unidimensional damping factor for couplings
Kc6 = Kc5;                      % Torsinal stifness of coupling 6 [Nm/rad]
Ic6 = Ic5;                      % Inertia of coupling 6 [kgm^2]

S5 = S1;                        % Unidimensional damping factor for shafts
Ds5 = 0.025;                    % Diameter of shaft 5 [m]
Ls5 = 0.250;                    % Length of shaft 5 [m]
% Calculate inertia and stiffness of shaft 5
[Ks5, Is5] = Shaft_inertia_stiffness(Ds4,Ls4,Shaft_shear_modulus,Shaft_density);

Dprop = 0.15;
Lprop = 0.02;
% Calculate inertia and stiffness of propeller mass
[Kprop, Iprop0] = Shaft_inertia_stiffness(Dprop,Lprop,Shaft_shear_modulus,Shaft_density);             

I_GL = 0.000132;
I_ML = 0.00065;
gml = 8;

Iprop = gml^2*I_ML + I_GL + Iprop0; %propeller innertia: propeller + equivalent torque for transmission and loading motor.

% Iprop = Iprop0 + 0.05;           % Propeller inertia: propeller + loadmotor ERROR HERE?



%%
Fs = 1000; %sampling frequency Hz

GearRatio1 = 1/gug;
GearRatio2 = 1/glg;

% Labels = {'Driving motor + coupling', 'shaft','fc hub','fc middle piece','fc hubs + shaft','fc middle piece','fc hub + shaft','shaft+coupling','torque transducer','torque transducer','coupling','shaft','shaft+gear','coupling','shaft','shaft + coupling','shaft','shaft + gear','coupling','torque transducer','torque transducer','coupling','shaft','shaft + loading motor'};
Labels={'Driving motor, coupling','Shaft','Flexible coupling hub','Flexible coupling middle piece','Flexible coupling hubs, shaft','Flexible coupling middle piece','Flexible coupling hub, shaft','Shaft, coupling','Torque transducer','Torque transducer, coupling','Shaft','Shaft, planetary gear','Coupling','Shaft','Shaft, coupling','Shaft','Shaft, planetary gear','Coupling','Torque transducer','Torque transducer, coupling','Shaft','Shaft, mass, gear, loading motor'};

%corrected 02/11/2020
MassmomentI(1) = Imotor+Ic_mf;
MassmomentI(2) = Is_mf;
MassmomentI(3) = Ifc_1;
MassmomentI(4) = Ifcm;
MassmomentI(5) = Ifc_2+Iem+Ifc_1;
MassmomentI(6) = Ifcm;
MassmomentI(7) = Ifc_2+Is1/2;
MassmomentI(8) = Is1/2+Ic1;
MassmomentI(9) = Im1/2;
MassmomentI(10) = Im1/2+Ic2;
MassmomentI(11) = Is2/2;
MassmomentI(12) = Is2/2+Iug;
MassmomentI(13) = Ic3;
MassmomentI(14) = Is3/2;
MassmomentI(15) = Is3/2+Ic4;
MassmomentI(16) = Is4/2;
MassmomentI(17) = Is4/2+Ilg;
MassmomentI(18) = Ic5;
MassmomentI(19) = Im2/2;
MassmomentI(20) = Im2/2+Ic6;
MassmomentI(21) = Is5/2;
MassmomentI(22) = Is5/2+Iprop;

% MassmomentI = [(Imotor+Ic_mf) (Is_mf) (Ifc_1) (Ifcm) (Ifc_2+Iem+Ifc_1) (Ifcm) (Ifc_2+Is1/2) (Is1/2+Ic1) (Im1/2) (Im1/2+Ic2) (Is2/2) (Is2/2+Iug) (Ic3) (Is3/2) (Is3/2+Ic4) (Is4/2) (Is4/2+Ilg) (Ic5) (Im2/2) (Im2/2+Ic6) (Is5/2) (Is5/2+Iprop)];
% MassmomentI = [Imotor+Ic_mf Is_mf+Ifc_1 Ifcm Ifc_2+Iem+Ifc_1 Ifcm Ifc_2 Is1/2 Is1/2+Ic1 Im1/2 Im1/2+Ic2 Is2/2 Is2/2+Iug Ic3 Is3/2 Is3/2+Ic4 Is4/2 Is4/2+Ilg Ic5 Im2/2 Im2/2+Ic6 Is5/2 Is5/2+Iprop];
DampingC = [(C*Kc_mf/W) (S*Ks_mf/W) (Cfc*Kfc/W) (Cfc*Kfc/W) (Cfc*Kfc/W) (Cfc*Kfc/W) (S1*Ks1/W) (C1*Kc1/W) (M1*Km1/W) (C2*Kc2/W) (S2*Ks2/W) (ug*Kug/W) (C3*Kc3/W) (S3*Ks3/W) (C4*Kc4/W) (S4*Ks4/W) (lg*Klg/W) (C5*Kc5/W) (M2*Km2/W) (C6*Kc6/W) (S5*Ks5/W)];
StiffnessK = [Kc_mf Ks_mf Kfc Kfc Kfc Kfc Ks1 Kc1 Km1 Kc2 Ks2 Kug Kc3 Ks3 Kc4 Ks4 Klg Kc5 Km2 Kc6 Ks5];


% MassmomentI = [Ic_mf+Imotor Is_mf Ifc_1 Ifcm Ifc_2+Iem+Ifc_1 Ifcm Ifc_2+Is1/2 Is1/2+Ic1 Im1 Ic2 Is2/2 Is2/2+Iug Ic3 Is3/2 Ic4+Is3/2 Is4/2 Is4/2+Ilg Ic5 Im2 Ic6 Is5/2 Is5/2+Iprop];
% StiffnessK = [Kc_mf Ks_mf Kfc Kfc Kfc Kfc Ks1 Kc1 Km1 Kc2 Ks2 Kug Kc3 Ks3 Kc4 Ks4 Klg Kc5 Km2 Kc6 Ks5];
% DampingC = [C*Kc_mf/W S*Ks_mf/W Cfc*Kfc/W Cfc*Kfc/W Cfc*Kfc/W Cfc*Kfc/W S1*Ks1/W C1*Kc1/W M1*Km1/W C2*Kc2/W S2*Ks2/W ug*Kug/W C3*Kc3/W S3*Ks3/W C4*Kc4/W S4*Ks4/W lg*Klg/W C5*Kc5/W M2*Km2/W C6*Kc6/W S5*Ks5/W];
GearRatios = [1 1 1 1 1 1 1 1 1 1 1 GearRatio1 1 1 1 1 GearRatio2 1 1 1 1 1]';
GearCorrection = prod(GearRatios);
RelativeGearRatios = [1;cumprod(GearRatios(1:end-1))];

% Labels = {'Motor','Shaft 1','Ifc_1','Ifcm','Ifc_2+Iem+Ifc_1','I_fcm','Ifcm','Ifc2+Is1/2','Is1/2+Ic1','Im1','Ic2','Is2/2','Is2/2','Upper gear','Ic3','Is3/2','Iv4+Is3/2','Is4/2','Lower gear','Ic5','Im2','Ic6','Is5/2','Propeller'};

Comments = 'TestBench parameters 18/08/2020';

TorqueMax = T;

Speed2TorqueFactor = -T*(glg*gug)/(n*(1/(glg*gug)))^2;

nmax = n;





















