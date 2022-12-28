#
# RebelSat-1 Simulation
# 
# Description: Simulates RS-1 in LEO 
# 

# Import utilities
from Basilisk.simulation import simSynch
from Basilisk.utilities import orbitalMotion, macros, vizSupport
import numpy as np

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
from RS1_masters import RS1Sim, RS1Scenario
import RS1_Dynamics
import RS1_Fsw

# Import plotting files for your scenario
import RS1_Plotting as RS1_plt


# Create your own scenario child class
class scenario_RS1(RS1Sim, RS1Scenario):
    def __init__(self):
        super(scenario_RS1, self).__init__()
        self.name = 'scenario_RS1'

        # declare empty class variables
        self.sNavAttRec = None
        self.sNavTransRec = None

        self.set_DynModel(RS1_Dynamics)
        self.set_FswModel(RS1_Fsw)

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynModels.scObject)

    def configure_initial_conditions(self):
        DynModels = self.get_DynModel()

        # Configure Dynamics initial conditions
        oe = orbitalMotion.ClassicElements()
        oe.a = 6738. * 1000  # meters semi-major axis
        oe.e = 0.0005731 # eccentricity
        oe.i = 51.6 * macros.D2R # inclination of orbital plane, use ISS
        oe.Omega = 116.0054 * macros.D2R # right ascension of ascending node
        oe.omega = 347.8 * macros.D2R # arguemnt of periapsis of the orbit
        oe.f = 85.3 * macros.D2R # true anomaly of orbit
        mu = DynModels.gravFactory.gravBodies['earth'].mu
        rN, vN = orbitalMotion.elem2rv(mu, oe)
        orbitalMotion.rv2elem(mu, rN, vN)
        DynModels.scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
        DynModels.scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B, initial attitude of B frame represented as an MRP
        DynModels.scObject.hub.omega_BN_BInit = [[0], [0], [0]]  # rad/s - omega_BN_B, initial angular velocity of B frame represented in B frame

    def log_outputs(self):
        # Dynamics process outputs
        DynModels = self.get_DynModel()
        self.sNavAttRec = DynModels.simpleNavObject.attOutMsg.recorder()
        self.sNavTransRec = DynModels.simpleNavObject.transOutMsg.recorder()
        self.AddModelToTask(DynModels.taskName, self.sNavAttRec)
        self.AddModelToTask(DynModels.taskName, self.sNavTransRec)

        # log the outputs of the CSS and add it to task
        self.cssConstLog = DynModels.CSSConstellationObject.constellationOutMsg.recorder()
        self.AddModelToTask(DynModels.taskName, self.cssConstLog)

    def pull_outputs(self, showPlots):
        DynModels = self.get_DynModel()

        # Dynamics process outputs
        sigma_BN = self.sNavAttRec.sigma_BN
        r_BN_N = self.sNavTransRec.r_BN_N
        v_BN_N = self.sNavTransRec.v_BN_N

        dataCSSArray = []
        dataCSSArray = self.cssConstLog.CosValue[:, :len(DynModels.CSSConstellationObject.sensorList)]
        np.set_printoptions(precision=16)

        # Plot results
        RS1_plt.clear_all_plots()
        timeLineSet = self.sNavAttRec.times() * macros.NANO2MIN
        RS1_plt.plot_orbit(r_BN_N)
        RS1_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)
        RS1_plt.plot_CSS(self, dataCSSArray)

        figureList = {}
        if showPlots:
            RS1_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["orbit", "orientation"]
            figureList = RS1_plt.save_all_plots(fileName, figureNames)

        return figureList

def liveStream(RS1):
    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = 50.0 # how fast to accelerate stream
    RS1.AddModelToTask(RS1.DynModels.taskName, clockSync)

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(RS1, RS1.DynModels.taskName, 
        RS1.DynModels.scObject, liveStream=True)

def runScenario(scenario):

    # Run Vizard
    # liveStream(scenario)


    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure FSW mode
    scenario.modeRequest = 'standby'

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(200.)

    scenario.ConfigureStopTime(simulationTime)

    scenario.ExecuteSimulation()

def run(showPlots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_RS1()
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(showPlots)


    return figureList

if __name__ == "__main__":
    sys.settrace
    run(True)
