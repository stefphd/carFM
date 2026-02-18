function checkCarStruct(bike)
    % CHECKCARSTRUCT Check the contents of the bike data structure.
    % INPUT:
    % bike: the bike data structure
    
    fields_and_types = {
        'wheelbase' 'double'
        'frontTrackWidth' 'double'
        'rearTrackWidth' 'double'
        'frontTyreUnloadedRadius' 'double'
        'rearTyreUnloadedRadius' 'double'
        'mass' 'double'
        'cgH' 'double'
        'cgB' 'double'
        'rollInertia' 'double'
        'pitchInertia' 'double'
        'crossYawRollInertia' 'double'
        'frontUnsprungMass' 'double'
        'frontWheelSpinInertia' 'double'
        'rearUnsprungMass' 'double'
        'rearWheelSpinInertia' 'double'
        'transmissionInertia' 'double'
        'transmissionEfficiency' 'double'
        'brakeRatio' 'double'
        'aeroH' 'double'
        'aeroB' 'double'
        'frontTyreStiffness' 'double'
        'frontTyreDamping' 'double'
        'rearTyreStiffness' 'double'
        'rearTyreDamping' 'double'
        'engine' 'struct'
        'transmission' 'struct'
        'frontSuspension' 'struct'
        'rearSuspension' 'struct'
        'aero' 'struct'
        'frontTyre' 'struct'
        'rearTyre' 'struct'
    };
    required_fields = fields_and_types(:,1);
    type_fields = fields_and_types(:,2);
    car_fields = fieldnames(bike);
    for k = 1 : numel(required_fields)
        if ~any(strcmp(required_fields{k}, car_fields))
            eid = 'carfm:noSuchVariable';
            msg = ['Unrecognized field name "' required_fields{k} '".'];
            error(eid,msg)
        end
        if ~isa(bike.(required_fields{k}), type_fields{k}) 
            eid = 'carfm:incorrectType';
            msg = ['Incorrect type of field "' required_fields{k} '".'];
            error(eid,msg)
        end
        if ~isequal(size(bike.(required_fields{k})), [1 1])
            eid = 'carfm:notEqual';
            msg = ['Size of field "' required_fields{k} '" must be [1,1]'];
            error(eid,msg)
        end
    end
    % check specific parameters
    if ~any(strcmp('gearboxRatios', car_fields))
        eid = 'carfm:noSuchVariable';
        msg = 'Unrecognized field name "gearboxRatios".';
        error(eid,msg)
    end
    if ~isa(bike.gearboxRatios, 'double') 
        eid = 'carfm:incorrectType';
        msg = 'Incorrect type of field "gearboxRatios".';
        error(eid,msg)
    end
    if ~any(strcmp('gearboxSwitchingSpeeds', car_fields))
        eid = 'carfm:noSuchVariable';
        msg = 'Unrecognized field name "gearboxSwitchingSpeeds".';
        error(eid,msg)
    end
    if ~isa(bike.gearboxSwitchingSpeeds, 'double') 
        eid = 'carfm:incorrectType';
        msg = 'Incorrect type of field "gearboxSwitchingSpeeds".';
        error(eid,msg)
    end
    if ~isequal(size(bike.gearboxRatios), size(bike.gearboxSwitchingSpeeds))
        eid = 'carfm:notEqual';
        msg = 'Size of field "gearboxRatios" must be the same of field "gearboxSwitchingSpeeds"';
        error(eid,msg)
    end
    % check tyre, aero, suspension, engine model functions
    %frontTyre
    try
        [~, ~, ~, ~, ~, ~, ~] = bike.frontTyre.Forces(bike.frontTyre, 1, 1, 0, 1, 1, 0);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate frontTyre.Forces function.';
        err = MException(eid, msg);   
        throw(err);
    end
    %rearTyre
    try
        [~, ~, ~, ~, ~, ~, ~] = bike.rearTyre.Forces(bike.rearTyre, 1, 1, 0, 1, 1, 0); 
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate rearTyre.Forces function.';
        err = MException(eid, msg);   
        throw(err);
    end
    %aero
    try
        [~, ~, ~, ~, ~, ~] = bike.aero.Forces(bike.aero, 1, 0, 0, 0, 0, 0, 0);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate aero.Forces function.';
        err = MException(eid, msg);   
        throw(err);
    end
    %fronSupension
    try
        [~] = bike.frontSuspension.Force(bike.frontSuspension, 0, 0, 0, 0);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate frontSuspension.Force function.';
        err = MException(eid, msg);   
        throw(err);
    end
    try
        [~] = bike.frontSuspension.Kinematics(bike.frontSuspension, 0, 0);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate frontSuspension.Kinematics function.';
        err = MException(eid, msg);   
        throw(err);
    end
    %rearSuspension
    try
        [~] = bike.rearSuspension.Force(bike.rearSuspension, 0, 0, 0, 0);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate rearSuspension.Force function.';
        err = MException(eid, msg);   
        throw(err);
    end
    try
        [~] = bike.rearSuspension.Kinematics(bike.rearSuspension, 0, 0);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate rearSuspension.Kinematics function.';
        err = MException(eid, msg);   
        throw(err);
    end
    %engine
    try
        [~, ~]= bike.engine.Torques(bike.engine, 10);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate engine.Torques function.';
        err = MException(eid, msg);   
        throw(err);
    end
    %transmission
    try
        [~, ~]= bike.transmission.Torques(bike.transmission, 10,10,10,10,10);
    catch ME
        eid = 'carfm:unableEval';
        msg = 'Unable to evaluate transmission.Torques function.';
        err = MException(eid, msg);   
        throw(err);
    end
end