function [Ge,sigma_m,mu_m] = inputDesignCases(inputDesignCase, Model, Gd)
% cf. Sec 3.2., Manngard et al. (2022). Torque estimation in marine propulsion systems, MSSP.
sigma_m = 2/3; %worst-case variance
mu_m = 2.7;    %mean

switch inputDesignCase
    case 1
    % design case (i)
    var1 = [1];         %var(e2): u2(k) = u2(k-1) + e2(k)
    var2 = [0.1];       %var(e1): u1(k) = e1(i)
    a = [1];

    R0 = (1+a^2)*var2 + var1;
    R1 = -a*var2;
    c = R0/2/R1 + sqrt( (R0/2/R1)^2 -1 );
    sigma_e = sqrt(R1/c); 
    Wp = ss([a c*sigma_e;0 0],[sigma_e;1],[1 0],0,1/Model.Fs);
    W = ss( [0 zeros(1,2);zeros(2,1) Wp.A], [sigma_m 0;zeros(2,1) Wp.B], [1 zeros(1,2);0 Wp.C], zeros(2,2), 1/Model.Fs );

    %augmented system
    Ge = extendedSystemMSSP(Gd,W,Model,Model.Lag);

    case 2
    % design case (ii)
    var1 = [1];         %var(e2): u2(k) = u2(k-1) + e2(k)
    var2 = [(10/3)^2];  %var(e1): u1(k) = e1(i)
    a = [1];

    R0 = (1+a^2)*var2 + var1;
    R1 = -a*var2;
    c = R0/2/R1 + sqrt( (R0/2/R1)^2 -1 );
    sigma_e = sqrt(R1/c); 
    Wp = ss([a c*sigma_e;0 0],[sigma_e;1],[1 0],0,1/Model.Fs);
    W = ss( [0 zeros(1,2);zeros(2,1) Wp.A], [sigma_m 0;zeros(2,1) Wp.B], [1 zeros(1,2);0 Wp.C], zeros(2,2), 1/Model.Fs );

    %augmented system
    Ge = extendedSystemMSSP(Gd,W,Model,Model.Lag);

    case 3
    % design case (iii)
    var1 = [0];         %var(e2): u2(k) = u2(k-1) + e2(k)
    var2 = [(10/3)^2];  %var(e1): u1(k) = e1(i)
    a = [0];
    S = [sigma_m^2 0;0 var2];
    Ge = extendedSystemConstantSpectraMSSP(Gd,S,Model.Lag);
end

end

