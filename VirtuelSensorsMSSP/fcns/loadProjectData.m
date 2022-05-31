function Model = loadProjectData(ProjectID)
    run([ProjectID 'Data.m'])     % load driveline parameters from data file (check that data is formatted properly)  
%     load('cfdData.mat')
    
    Model.nMasses = length(MassmomentI);
    Model.nStates = 2*length(MassmomentI)-1;
    Model.nInputs = 2;

    Model.I = MassmomentI;
    Model.c = DampingC;
    Model.k = StiffnessK;
    Model.n = GearRatios';
    Model.RelativeGearRatios = RelativeGearRatios;
    Model.Fs = Fs;
    Model.Labels = Labels;
    Model.Comments = Comments;
    Model.ProjectID = ProjectID;
    Model.GearCorrection = GearCorrection;
    Model.NominalSpeed = nmax/60; %nominal speed [Hz];
    Model.NominalTorque = P/(nmax/9.55)*n^2/nmax^2; %nominal speed in [Nm];
end

