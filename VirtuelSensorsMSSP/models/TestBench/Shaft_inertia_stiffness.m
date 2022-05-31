function [torsinal_stiffness,inertia] = Shaft_inertia_stiffness(diameter,length,shear_modulus,density)

G = shear_modulus;           % Shear modulus
roo = density;               % density   
D = diameter;                % Diameter of shaft
L = length;                  % Length of shaft


% Laskuja

J = (pi*D^4)/32;            % torsional constant for round shaft
TS = (G*J)/L;               % torsional stiffness

m = (L*pi*(D/2)^2)*roo;      % weigth
I = 0.5*m*(D/2)^2;          % inertia


torsinal_stiffness = TS;
inertia = I;
end

