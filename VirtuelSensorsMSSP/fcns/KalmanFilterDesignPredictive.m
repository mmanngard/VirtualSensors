function [KF,Pz,K] = KalmanFilterDesignPredictive(Ge)
%KALMANFILTERDESIGN Compute a Kalman filter.
nStates = Ge.nStates;
nMasses = Ge.nMasses;
nInputs = Ge.nInputs;
nOutputs = Ge.nOutputs;
nz = size(Ge.T,1);

Ae = Ge.A;
Ce = Ge.C;
Te = Ge.T;

Qe = Ge.Q;
R = Ge.R;

%% Riccati equation
P = dare(Ae',Ce',Qe,R);

K = P*Ce'/(R+Ce*P*Ce');
Pz = Ge.T*(eye(length(K*Ce))-K*Ce)*P*Ge.T';

% filter implementation
% KF = ss( (eye(length(P))-K*Ce)*Ae, K, Te, zeros(nz,nOutputs), 1 );
KF = ss( (Ae - K*Ce), K, Te, zeros(nz,nOutputs), 1 ); %predictive


end

