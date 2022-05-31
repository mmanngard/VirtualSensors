function Ge = extendedSystemConstantSpectraMSSP(Gd,S,Lag)
%EXTENDEDSYSTEM Comput extended system from pre-filter and system model.

nG = length(Gd.A); %number of states
mG = 2; %number of inputs
pG = size(Gd.C,1); %number outputs
% nW = length(W.A); %number of pre-filter states
% mW = size(W.C,2); %number of pre-filter outputs
nT = size(Gd.T,1); %number of states to be estimates

nMasses = Gd.nMasses;

%%
A = Gd.A;
B = Gd.B;
C = Gd.C;
D = Gd.D;
T = Gd.T;

F = [eye(nMasses)               zeros(nMasses, nMasses-1);
     zeros(nMasses-1,nMasses)   zeros(nMasses-1)]'; %process noise affecting speeds;

% F = eye(nG);        %state noise matrix
% F = [B zeros(nG,nG-2)]; %input noise F=B

Q = F*Gd.Q*F';

R = Gd.R;

%extended system
Ge.A = [zeros(mG,mG) zeros(mG,nG);B A];
Ge.F = [S^(1/2) zeros(mG,nG);0*B*S^(1/2) F];
Ge.C = [zeros(pG,mG) C];
Ge.Bm = [zeros(2,1);Gd.B(:,1)];
Ge.T = [eye(mG)                   zeros(mG,nG);
        zeros(nT,mG)              T
        zeros(nMasses,mG)         [eye(nMasses)   zeros(nMasses,nMasses-1)] ];

Ge.Q = Ge.F*blkdiag(eye(2),Gd.Q)*Ge.F';
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
    Gfls.R = Ge.R;

    Gfls.nMasses = Ge.nMasses;
    Gfls.nStates = length(Gfls.A);
    Gfls.nOutputs = Ge.nOutputs;
    Gfls.nInputs = Ge.nInputs;
  
    Ge = Gfls;
    
end
    
    
end

