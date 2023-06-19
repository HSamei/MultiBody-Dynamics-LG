function J = Joint(varargin)
% Define a Body Structure containing following fields:
%     - Name:           Assigned name to identify joint
%     - Type:           Identify rigid or flexible joint
%     - Twist:          Assigned twist axis defining the joint
%     - Position:       Assigned magnitude of twist defining the joint
%     - Vel:            Assigned time rate of change of Position
%     - Accel:          Assigned time rate of change of Vel(ocity)
%     - Limit:          Assigned limits to joint values
%     - HomePos:        Assigned reference home value of joint
%     - Parent:         Parent Body of this joint
%     - Child:          Child Body of this joint

    if strcmpi(varargin{2},'RIGID')
        NAME      = varargin{1};
        TYPE      = 'RIGID';
        TWIST     = varargin{3};
        POSITION  = varargin{4};
        VELOCITY  = varargin{5};
        ACCEL     = varargin{6};
        LIMIT     = varargin{7};
        HOMEPOS   = varargin{8};
        PARENT    = varargin{9};
        CHILD     = varargin{10};
        
    elseif strcmpi(varargin{2},'FLEXIBLE')
        NAME      = varargin{1};
        TYPE      = 'FLEXIBLE';
        TWIST     = varargin{3};
        POSITION  = varargin{4};
        VELOCITY  = varargin{5};
        ACCEL     = varargin{6};
        LIMIT     = varargin{7};
        HOMEPOS   = varargin{8};
        PARENT    = varargin{9};
        CHILD     = varargin{10};
        
    else
        J = null(1);
        return;
        
    end

    J.Name      = NAME;
    J.Type      = TYPE;
    J.Twist     = TWIST;
    J.Position  = POSITION;
    J.Vel       = VELOCITY;
    J.Accel     = ACCEL;
    J.Limit     = LIMIT;
    J.HomePos   = HOMEPOS;
    J.Parent    = PARENT;
    J.Child     = CHILD;
    
end