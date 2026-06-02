function sysCall_init()
    local h=sim.getObject('.',{noError=true})
    local config={}
    config.size={0.5,0.5,0.5}
    config.density=1
    config.static=true
    local pc=sim.packTable(config)
    local d=sim.readCustomStringData(h,'CONF')
    if d~=pc then    
        sim.writeCustomStringData(h,'CONF',pc)
        sim.setShapeBB(h,config.size)
        local mass= config.size[1]* config.size[2]* config.size[3]*config.density
        sim.setObjectFloatParam(h,sim.shapefloatparam_mass,mass)
        if config.static then
            sim.setObjectInt32Param(h,sim.shapeintparam_static,1)
        else
            sim.setObjectInt32Param(h,sim.shapeintparam_static,0)
        end
    end
    
end
