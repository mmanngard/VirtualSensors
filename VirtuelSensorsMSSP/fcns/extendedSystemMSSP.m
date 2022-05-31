function Ge = extendedSystemMSSP(Gd,W,Model,Lag)
%EXTENDEDSYSTEM Comput extended system from pre-filter and system model.
W = ss(W);

nG = length(Gd.A); %number of states
mG = size(Gd.C,1); %number outputs
nW = length(W.A); %number of pre-filter states
mW = size(W.C,2); %number of pre-filter outputs
nT = size(Gd.T,1); %number of states t obe estimates

nMasses = Gd.nMasses;

%%
A = Gd.A;
B = Gd.B;
C = Gd.C;
D = Gd.D;
T = Gd.T;
% F = eye(nG);        %state noise matrix
% F = [diag(1./Model.I)           zeros(nMasses, nMasses-1);
%      zeros(nMasses-1,nMasses)   zeros(nMasses-1)]'; %process noise affecting speeds;
F = [eye(nMasses)           zeros(nMasses, nMasses-1);
     zeros(nMasses-1,nMasses)   zeros(nMasses-1)]'; %process noise affecting speeds;
% F = [diag(1./Model.I)           zeros(nMasses, nMasses-1);
%      zeros(nMasses-1,nMasses)   zeros(nMasses-1)]'; %process noise affecting speeds;

% F = [0*B zeros(nG,nG-2)]; %input noise F=B

Q = F*Gd.Q*F';

R = Gd.R;
S = Gd.S;

Aw = W.A;
Bw = W.B;
Cw = W.C;
Dw = W.D;



%extended system
Ge.A = [Aw zeros(nW,nG);B*Cw A];
Ge.F = [Bw zeros(nW,nG);B*Dw F];
Ge.C = [zeros(mG,mW) C];
Ge.Bm = [zeros(nW,1);Gd.B(:,1)];
% Ge.T = [Cw zeros(nW,nG);zeros(nT,nW) T];

Ge.T = [Cw                        zeros(2,nG);
        zeros(nT,nW)              T
        zeros(nMasses,nW)         [eye(nMasses)   zeros(nMasses,nMasses-1)] ];

Ge.Q = [Bw*Bw' Bw*Dw'*B';B*Dw*Bw' B*Dw*Dw'*B'+F*Q*F'];

% Ge.Q = Ge.F*blkdiag(eye(2),Gd.Q)*Ge.F';
Ge.S = [eye(nW,mG);S];
Ge.R = R;

%%
Ge.nMasses = Gd.nMasses;
Ge.nStates = length(Ge.A);
Ge.nOutputs = Gd.nOutputs;
Ge.nInputs = Gd.nInputs; %inputs of original system



%% ############     FIXED LAG SMOOTHING    ##############
nStates = Ge.nStates;
nOutputs = Ge.nOutputs;
nz = size(Ge.T,1);

ne = size(Ge.F,2);


if Lag>1
    Gfls.A = [Ge.A                   zeros(nStates,(Lag-1)*nz);
              Ge.T                   zeros(nz,(Lag-1)*nz);
              zeros((Lag-2)*nz,nStates)   eye((Lag-2)*nz)   zeros((Lag-2)*nz,nz)];
    Gfls.Bm = [Ge.Bm;zeros((Lag-1)*nz,1)];
    Gfls.F = [Ge.F;zeros((Lag-1)*nz,ne)];
    Gfls.C = [Ge.C zeros(nOutputs,(Lag-1)*nz)];
    Gfls.T = [zeros(nz,nStates) zeros(nz,(Lag-2)*nz) eye(nz)];

    Gfls.Q = blkdiag(Ge.Q,zeros((Lag-1)*nz));
    Gfls.S = Ge.S;
    Gfls.R = Ge.R;

    Gfls.nMasses = Ge.nMasses;
    Gfls.nStates = length(Gfls.A);
    Gfls.nOutputs = Ge.nOutputs;
    Gfls.nInputs = Ge.nInputs;
    
    
%     Gfls.Q = Gfls.F*Ge.Q*Gfls.F'; %!
    %
    Ge = Gfls;
    
end
    
end

