function B = Body(varargin)
% Define a Body Structure containing following fields:
%     - Name:       Assigned name to identify body
%     - Type:       Identify rigid or flexible body
%     - Mass:       Assigned mass matric of the body about BCF (both inertia and mass)
%     - Transform:  Screw associated with transformation from start to end                       (RIGID ONLY)
%     - Stiff:      Stiffness Matrix of the body about BCF (axial, shear, torsional & bending)   (FLEXIBLE ONLY)
%     - Damp:       Dampiing Matrix of the body about BCF (axial, shear, torsional & bending)    (FLEXIBLE ONLY)
%     - CoM:        Screw associated with transformation from start to Center of Mass            (RIGID ONLY)
%     - F_0:        Twist associated with free strain of the continuum under no applied loadings (FLEXIBLE ONLY)
%     - N:          Number of elements used to discretize the continuum                          (FLEXIBLE ONLY)
%     - L:          Reference length of continuum under no applied loadings                      (FLEXIBLE ONLY)
%
% Written by BD Bhai

    if strcmpi(varargin{2},'RIGID')
        NAME        = varargin{1};
        TYPE        = 'RIGID';
        MASS        = varargin{3};
        TRANSFORM   = varargin{4};
        STIFF       = null(1);
        DAMP        = null(1);
        CoM         = varargin{5};
        F_0         = null(1);
        N           = null(1);
        L           = null(1);
        
    elseif strcmpi(varargin{2},'FLEXIBLE')
        NAME        = varargin{1};
        TYPE        = 'FLEXIBLE';
        MASS        = varargin{3};
        TRANSFORM   = zeros(6,1);
        STIFF       = varargin{4};
        DAMP        = varargin{5};
        CoM         = zeros(6,1);
        F_0         = varargin{6};
        N           = varargin{7};
        L           = varargin{8};
        
    else
        error('Really? Its RIGID or FLEXIBLE, how hard is it to do that?')
    end

    B.Name      = NAME;
    B.Type      = TYPE;
    B.Mass      = MASS;
    B.Transform = TRANSFORM;
    B.Stiff     = STIFF;
    B.Damp      = DAMP;
    B.CoM       = CoM;
    B.F_0       = F_0;
    B.N         = N;
    B.L         = L;
    
end