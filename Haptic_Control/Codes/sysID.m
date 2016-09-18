% system Identification
function dataID = sysID()
    dataID.M = 176; % [kg], Stage and motor inertia
    dataID.B = 1051; % [kg/s], Stage damping
    dataID.m = 0.035; % [kg], End-effector mass
    dataID.b = 15.05; % [kg/s], End-effector damping
    dataID.k = 1300; % [N/m], Physical spring stiffness
    dataID.n = 41.8; % [-], Gear ratio
end