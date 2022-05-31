function ssSym = ss_sym(Model)
%DRIVELINE2SYMSS Creates a symbolic state-space model based on driveline
%parameters

nMasses = Model.nMasses;
RPMSensorLocations = Model.RPMSensorLocations;
TorqueSensorLocations = Model.TorqueSensorLocations;

nSensors = length(RPMSensorLocations) + length(TorqueSensorLocations);

C = sym('c',[1 nMasses-1]);
K = sym('k',[1 nMasses-1]);
I = sym('I',[1 nMasses]);
n = sym('n',[1 nMasses]);     %gear ratios
damp = sym('damp',[1 nMasses]);


Acmat = [diag( (n(1:end-1).*C)./I(2:end),-1) + diag((-([0,C] + n.^2.*[C,0]))./I,0) + diag( (n(1:end-1).*C)./I(1:end-1) ,1)];
Akmat = [diag(K./I(2:end),-1) + diag(-(n.*[K,0])./I,0)];

omega = sym('omega',[1 nMasses]);
theta = sym('theta',[1 nMasses]);

x = [omega, n(1:end-1).*theta(2:end)-theta(1:end-1)]; %is this an eeror??? should it be n_1*theta_1-theta_2
A = [Acmat Akmat(:,1:end-1);
     diag(n,0)-diag(ones(nMasses-1,1),1) zeros(nMasses,nMasses-1)];  A=A(1:end-1,:);
 
B = sym(zeros(2*nMasses-1,2)); B(1,1) = 1/I(1); B(nMasses,2) = -1/I(nMasses);


%transform mat
cmat = sym(zeros(length(C),length(C)+1));
kmat = sym(zeros(length(K)));
for k=1:length(C)
    cmat(k,k:k+1) = n(k)*[n(k)*C(k) -C(k)]; %check extra n(k). Outputs torque between lower gear pinion and the next mass.
    kmat(k,k) = K(k);
end
T=[0*cmat kmat];


% it is assumed that only the torsue caused by angular deflections is
% measured.
Tdef = [zeros(size(cmat)) kmat]; %defflection torque
if nSensors>0
    Cmat = sym(zeros(nSensors,2*nMasses-1));
    for k=1:length(RPMSensorLocations)
        Cmat(k,RPMSensorLocations(k))=1;
    end
    Cmat(length(RPMSensorLocations)+1:nSensors,:) = Tdef(TorqueSensorLocations,:); %measure only deflection torque
else
    Cmat = [];
end
D = sym(zeros(length(RPMSensorLocations)+length(TorqueSensorLocations),2));


%% Damping between motor and ground
    A(1:nMasses,1:nMasses) = A(1:nMasses,1:nMasses) - diag(damp./I);
% 
%     Cm = sym('cm',[1 2]);
%     A(1,1) = A(1,1) - Cm(1)/I(1);
%     A(nMasses,nMasses) = A(nMasses,nMasses) - Cm(2)/I(nMasses);

%%
ssSym.A = sym(A);
ssSym.B = sym(B);
ssSym.C = sym(Cmat);
ssSym.D = sym(D);
ssSym.T = sym(T);
ssSym.x = sym(x);



end

