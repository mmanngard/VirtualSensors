function G = ss_ini(Model)
    nMasses = Model.nMasses;


    I = sym('I',[1 nMasses]);
    C = sym('c',[1 nMasses-1]);
    K = sym('k',[1 nMasses-1]);
    n = sym('n',[1 nMasses]);     %gear ratios
    damp = sym('damp',[1 nMasses]); %motor damping, not used anymore

    G.A = eval(subs(Model.sys_sym.A,[I C K n damp],[Model.I Model.c Model.k Model.n Model.damp]));
    G.B = eval(subs(Model.sys_sym.B,[I C K n damp],[Model.I Model.c Model.k Model.n Model.damp]));
    G.C = eval(subs(Model.sys_sym.C,[I C K n damp],[Model.I Model.c Model.k Model.n Model.damp]));
    G.D = eval(subs(Model.sys_sym.D,[I C K n damp],[Model.I Model.c Model.k Model.n Model.damp]));
    G.T = eval(subs(Model.sys_sym.T,[I C K n damp],[Model.I Model.c Model.k Model.n Model.damp]));
    
    
    nOutputs = size(G.C,1);
    nStates = length(G.A);
    
    G.Q = Model.Q;
    G.R = Model.R;
    G.S = zeros(nStates,nOutputs);
    
    %%
    G.nMasses = Model.nMasses;
    G.nStates = Model.nStates;
    G.nOutputs = Model.nSensors;
    G.nInputs = Model.nInputs;
end

