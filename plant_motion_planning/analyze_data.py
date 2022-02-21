from fileinput import close
import random
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data
import time
import cv2 
from skimage.metrics import structural_similarity as ssim
from plant_motion_planning.pybullet_tools.camera import PyBulletCamera
from plant_motion_planning.rand_gen_plants_programmatically_main import create_random_plant, create_plant_params
from plant_motion_planning.representation import TwoAngleRepresentation, TwoAngleRepresentation_mod, CharacterizePlant, \
    CharacterizePlant2


base_distance = [0.18466560803508475, 0.1484790263800085, 0.15216375245281547, 0.08766153261534754, 0.1038412519006305, 0.0813932005233064, 0.04882429729920144, 0.09682144080013384, 0.029575011060822783, 0.07884738452140383, 0.16852930695338356, 0.09998009658765562, 0.17539009034734798, 0.03886253323730945, 0.00686572950173385, 0.19997362426579687, 0.15574700080557421, 0.1088957329611085, 0.08329474451484725, 0.04188720946764005, 0.19029492928870848, 0.17670740026730597, 0.03813975804171349, 0.061343526114403385, 0.07646077100627884, 0.15764923860660043, 0.04725787290419105, 0.12940981264848384, 0.19245912733220683, 0.07600758118746441, 0.026702861269980893, 0.1460665772956234, 0.0881659422249096, 0.13517189511324404, 0.0990403156088775, 0.1149783929403356, 0.07267970675796348, 0.10600536631044295, 0.14818842538309715, 0.020976787872763705, 0.13149826058576464, 0.015807136792020095, 0.11807037144481895, 0.023775865526432127, 0.12765821515166934, 0.04713913505439823, 0.11132529588340756, 0.027213578917880496, 0.14103786021249518, 0.037569132746634676, 0.14727818672383378, 0.20819420414960327, 0.08713258745106527, 0.03473234999608153, 0.20088110565659614, 0.06213290248350174, 0.10342785514045234, 0.18593259405549137, 0.20461899111337933, 0.1355084903636472, 0.15677652472703943, 0.20097217783729782, 0.06583183359155453, 0.16748527387181564, 0.17443508358632837, 0.044689550748608274, 0.15799827431802196, 0.04284839552385065, 0.07747173326558938, 0.0513376271939983, 0.09676018421338611, 0.009652507724023174, 0.11427480548060266, 0.17245719385119307, 0.18109585344276632, 0.03229205280228372, 0.176112891403466, 0.14731688835131787, 0.08865919126178967, 0.10477802161112187, 0.14038204847506433, 0.07420040677122865, 0.03607094950114443, 0.07552689530089503, 0.14893388740118757, 0.04897885395236192, 0.022379144691787898, 0.13776985796935093, 0.08081088205389374, 0.12183828081546651, 0.056485849615609596, 0.09297061103267008, 0.05317248418053349, 0.20093433509607486, 0.13478946860327787, 0.12298280719749033, 0.202409690466961, 0.029928503189397982, 0.08237457150355602, 0.07754387260314947]
link1_ori_distr = [0.008577339427773722, 0.0007091669828853675, 0.010578496021164896, 0.008742307866396515, 0.012678115069303053, 0.006114771436226185, 0.006329214282321249, 0.006440886336451901, 0.0020479602548032494, 0.006259350303893574, 0.002262444410156328, 0.004195389186011811, 0.0013933905191964024, 0.01877320002984062, 0.019215913874336077, 0.00997520607589264, 0.02601595164951709, 0.0021351908488733695, 0.01877501801157322, 0.003550076960121218, 0.002887638078648891, 0.012992236133377522, 0.004327702240845777, 0.003835629351856218, 0.0017354185468192451, 0.003094622780782541, 0.014243515892607084, 0.0072225734704589595, 0.005215542325823108, 0.0028643950291569187, 0.004210644071064307, 0.01013501396716987, 0.006877498744934485, 0.011858070130383869, 0.011890308690011175, 0.012810816971416727, 0.004194860879093154, 0.0058447925275062484, 0.006647046419783198, 0.0074368613444038, 0.013966961080140283, 0.0022263940947427896, 0.0025106108030683583, 0.008353075955938483, 0.005962276789342247, 0.01740411123856589, 0.014737005459517283, 0.0038496558309519813, 0.008489724014486999, 0.0027519037861989393, 0.0014269929309619345, 0.010872522242647786, 0.0030277202458020014, 0.00031834861610136045, 0.036772230387592364, 0.005983613885008121, 0.003898620142262943, 0.005298947806642751, 0.0177021392276151, 0.005601270406792702, 0.011143450571909908, 0.01346212006881442, 0.016961214039081574, 0.0012938758930149952, 0.0064080513212387835, 0.007917710657050714, 0.0006818138250861416, 0.0033300455854862188, 0.01379738650493767, 0.0047219304792209815, 0.01082274487075563, 0.00966664839906417, 0.00642648750087782, 0.0063976097880502625, 0.0007724157710261581, 0.0021423734897548785, 0.007136548418729904, 0.008872645635021215, 0.006044355054234396, 0.002364582658623493, 0.009050813864295848, 0.001803298699653233, 0.009197452209991797, 0.005267663378257792, 0.019626641140691903, 0.00765576893640052, 0.0008999350699003816, 0.008864016260046403, 0.002755329194461087, 0.0113268335070138, 0.004941618519709978, 0.007780337340480337, 0.006984469781904146, 0.0030877117869554382, 0.00502228551372752, 0.004561522827254749, 0.018493400030875073, 0.00802993210641545, 0.003702941168992835, 0.014099996072960863]
link2_ori_distr = [0.00047492537970772464, 0.02991408733605061, 0.08055976050765723, 0.01967608238553875, 0.006607933157318113, 0.06599803048491848, 0.04540869762770239, 0.03049502971747997, 0.02126085270557132, 0.053613139801853116, 0.0025843550116930736, 0.020166324471235564, 0.10404310023839292, 0.014200440933387237, 0.014527804530065969, 0.04350219728964311, 0.003907173607551773, 0.03950237104230503, 0.00042860935243016485, 0.027076602181810516, 0.008684060497280877, 0.05826779671887894, 0.00026766814669187955, 0.07908871756576175, 0.007857802701736083, 0.04647847511926373, 0.02160150710309272, 0.03194178741440512, 0.01634362485687102, 0.02054418364711963, 0.003514574474220211, 0.025686773939297525, 0.04757048914917261, 0.00622176697608523, 0.008871934403731685, 0.020585476417711468, 0.03782839764370738, 0.02661888095429754, 0.024862994000720562, 0.018375445479987262, 0.04975503390854652, 0.04824780810447693, 0.030107936969631766, 0.011057927628863817, 0.029950436741536635, 0.020610528600214617, 0.003913761616418099, 0.03618327786083453, 0.01809465748901573, 0.013011262981502014, 0.04416483065388532, 0.044460702601682445, 0.0011685072309979283, 0.004659614260937306, 0.05651349023780439, 0.0014965135664948281, 0.009078138177574302, 0.029481018129121628, 0.004733869076560926, 0.015616213339951668, 0.02506239531152632, 0.014661133551967409, 0.02195145828566103, 0.032521567654760886, 0.011624890330107829, 0.026673512690351675, 0.018433750350700828, 0.005718835845784476, 0.01973088070304474, 0.06661940215315176, 0.004951923318564977, 0.010253385921293767, 0.06141533803458765, 0.008027880520696473, 0.04857109756901201, 0.005052237994952802, 0.030961841890741026, 0.0015774375631788518, 0.00230540641654553, 0.006929698490179104, 0.0020170268043112616, 0.01696414676245639, 0.025884270531323472, 0.03860995422975044, 0.07634513486204497, 0.02477204091152141, 0.06071744279629754, 0.03944152024792069, 0.03259486106511911, 0.0352364097292992, 0.00447346026837081, 0.01856731992253957, 0.00017785090707522766, 0.03694250073553951, 0.02296702016541652, 0.01405429492299115, 0.004575174616722522, 0.002178844704993854, 0.00337500266467039, 0.02321398443305911]


num_branches_per_stem = 0
total_num_vert_stems = 2
total_num_extensions = 1
stem_half_length = 0.1
stem_half_width = 0.1

eye = np.array([2.0,0.0,2.0])
lookat = np.array([0.0,0.0,0.0])
up = np.array([-1.0,0.0,0.0])
cam = PyBulletCamera(eye, lookat, camera_up=up)

exec_times = []
EMDs = []
base_proximity = []
link1_ori_dist = []
link2_ori_dist = []


if __name__ == "__main__":

    p.connect(p.GUI)#, options='--background_color_red=1.0 --background_color_green=0.0 --background_color_blue=0.0')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)
    for j in range(100):

        print("Generating plant"+str(j))
        base_points = {}
        main_stem_indices = []
        link_parent_indices = {}

        base_params, stems_params, main_stem_indices = create_plant_params(num_branches_per_stem, total_num_vert_stems,
                                                                        total_num_extensions, [0.0, 0.0])



        base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params
        link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
            linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = stems_params






        saved_base_pos = np.array(base_pos)
        print("base link2 ori", linkOrientations[2])
        saved_link2_ori = np.array(linkOrientations[2])
        print("base base ori", base_ori)
        saved_base_ori = np.array(base_ori)


        x = np.random.uniform(low=-0.2, high=0.2)
        y = np.random.uniform(low=-0.2, high=0.2)
        base_params, stems_params, main_stem_indices = create_plant_params(num_branches_per_stem, total_num_vert_stems,
                                                                        total_num_extensions, [x, y])



        base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params
        link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
            linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = stems_params






        basepos = np.array(base_pos)
        baseori = np.array(base_ori)
        link2ori = np.array(linkOrientations[2])

        base_proximity.append(np.linalg.norm(basepos-saved_base_pos,2))
        link1_ori_dist.append(1-np.abs(np.dot(baseori, saved_base_ori)))
        link2_ori_dist.append(1-np.abs(np.dot(link2ori, saved_link2_ori)))

    print("random")
    print (np.average(base_proximity))
    print (np.average(link1_ori_dist))
    print (np.average(link2_ori_dist))
    print("best")
    print (np.average(base_distance))
    print (np.average(link1_ori_distr))
    print (np.average(link2_ori_distr))

    ax = plt.hist(link2_ori_dist, 15)
    plt.title("Distribution of Link2 Orientation Distance between 100 randomly generated pairs")
    plt.xlabel("quaternion distance")
    plt.ylabel("count")
    plt.show()