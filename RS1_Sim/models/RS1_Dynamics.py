#
# RebelSat-1 Dynamics
# 
# Description:
# 
#
# 
# 

import os
import numpy as np
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav, 
                                     coarseSunSensor, eclipse, radiationPressure)
from Basilisk.simulation import ephemerisConverter
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.topLevelModules import pyswice
from Basilisk.architecture import messaging

from Basilisk import __path__
bskPath = __path__[0]


class RS1DynamicModels():
    """
    RS-1 simulation class that sets up the spacecraft simulation configuration.

    """
    def __init__(self, SimBase, dynRate):
        # define empty class variables
        self.sun = None
        self.earth = None
        self.moon = None
        self.epochMsg = None

        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.solarRadPressureObject = radiationPressure.RadiationPressure()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()
        self.EarthEphemObject = ephemerisConverter.EphemerisConverter()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()

        # Assign initialized modules to tasks with priority values 
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.EarthEphemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, None, 108)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, None, 204)
        SimBase.AddModelToTask(self.taskName, self.solarRadPressureObject, None, 300)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, None, 300)
     
    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    # Spacecraft Model - Include more accurate model later 
    #   B Frame is located at CoM
    def SetSpacecraftHub(self):
        """
        Specify the spacecraft hub parameters.
        """
        self.scObject.ModelTag = "RS-1"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [1e-11, 1e-9, 2e-11,
                    -1e9, 1e-11, -7e-11,
                    -7e-11, -2e-11, 1e-9]
        self.scObject.hub.mHub = 1.519  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation
        """
        timeInitString = "2012 MAY 1 00:28:30.0"
        gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'moon'])
        gravBodies['earth'].isCentralBody = True
        self.sun = 0
        self.earth = 1
        self.moon = 2

        self.scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(self.gravFactory.gravBodies.values()))
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True)
        self.epochMsg = self.gravFactory.epochMsg

        self.gravFactory.spiceObject.zeroBase = 'Earth'

        self.EarthEphemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])

        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for c in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[c])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)
        
    # Add solar radiation pressure to satellite
    # TO DO: Check SunSpiceMsg Position Vector and check for eclipse
    def SetSolarRadiationPressure(self):
        """Set the solar radiation pressure on satellite."""
        self.solarRadPressureObject.ModelTag = "solarRadiationDisturbance"

        # Use cannonball which ignores attitude
        self.solarRadPressureObject.setUseCannonballModel()
        self.solarRadPressureObject.area = 0.03 # m^2 body surface area
        self.solarRadPressureObject.coefficientReflection = 1.2 
    
        #Eclipse
        self.sunEclipseMsgData = messaging.EclipseMsgPayload()
        self.sunEclipseMsgData.shadowFactor = 0.5
        self.sunEc1Msg = messaging.EclipseMsg().write(self.sunEclipseMsgData)
        self.solarRadPressureObject.sunEclipseInMsg.subscribeTo(self.sunEc1Msg) 
        
        # Time and planetary or spacecraft body info from JPL ephemeris library 
        self.sunSpiceMsg = messaging.SpicePlanetStateMsgPayload()
        self.sunSpiceMsg.PositionVector = [507128401.716, 22652490.9092, -14854379.6232] # m, true position of planet for the time
        self.sunMsg = messaging.SpicePlanetStateMsg().write(self.sunSpiceMsg)
        self.solarRadPressureObject.sunEphmInMsg.subscribeTo(self.sunMsg)

        self.scObject.addDynamicEffector(self.solarRadPressureObject)

    def SetExternalForceTorqueObject(self):
        """Set the external force and torque object."""
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetSimpleNavObject(self):
        """Set the navigation sensor object."""
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetCSSConstellation(self):
        """Set the 4 CSS sensors"""
        self.CSSConstellationObject.ModelTag = "cssConstellation"

        def setupCSS(cssDevice):
            cssDevice.fov = 80. * mc.D2R         # half-angle field of view value
            cssDevice.scaleFactor = 2.0
            cssDevice.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
            cssDevice.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
            cssDevice.sunEclipseInMsg.subscribeTo(self.eclipseObject.eclipseOutMsgs[0])
            cssDevice.this.disown()

        # setup CSS sensor normal vectors in body frame components
        nHat_B_List = [
            [1., 0., 0.],
            [0., 1., 0.],
            [-1., 0., 0.],
            [0., -1., 0.],
        ]
        numCSS = len(nHat_B_List)
        
        # TODO: once we receive exact location of sun sensors 
        # TODO: create another list r_B like above. these are the 4 positions of the CSS in body frame 
        # TODO: within the for loop below, also add the positions to cssList

        # store all
        cssList = []
        for nHat_B, i in zip(nHat_B_List, list(range(1,numCSS+1))):
            CSS = coarseSunSensor.CoarseSunSensor()
            setupCSS(CSS)
            CSS.ModelTag = "CSS" + str(i)
            CSS.nHat_B = np.array(nHat_B)
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarseSunSensor.CSSVector(cssList)

    # Global call to initialize every module
    def InitAllDynObjects(self):
        """
        Initialize all the dynamics objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies()
        self.SetSolarRadiationPressure()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetEclipseObject()
        self.SetCSSConstellation()