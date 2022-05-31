function Gd = discreteSystem(G,Ts,method)
Gss = ss(G.A,G.B,G.C,G.D);
Gdss = c2d(Gss,Ts,method); %discrete time model

Gd.A = Gdss.A;
Gd.B = Gdss.B;
Gd.C = Gdss.C;
Gd.D = Gdss.D;

Gd.T = G.T;

Gd.Q = G.Q;
Gd.R = G.R;
Gd.S = G.S;

    %%
    Gd.nMasses = G.nMasses;
    Gd.nStates = G.nStates;
    Gd.nOutputs = G.nOutputs;
    Gd.nInputs = G.nInputs;

end

