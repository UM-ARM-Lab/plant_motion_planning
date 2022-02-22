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


base_distance2 = [0.005317504109468147, 0.005574963720411767, 0.0005643862101277, 0.009233834308107793, 0.012586930467853825, 0.0024670241739131737, 0.010162057405433796, 0.007713380070360244, 0.011704546936371865, 0.0008431503627739229, 0.0007868429167061652, 0.013030601695683881, 0.0038948880489911436, 0.003441252813518904, 0.012255529483645, 9.660458033533548e-05, 0.006305462938941103, 0.010308504457824677, 0.00421463442490376, 0.0018086146614902862, 0.007613465381061595, 0.01218114993171452, 0.00909339422495132, 0.0030816605770137354, 0.008315231671026692, 0.0014609518885568828, 0.009720343842267293, 0.013740139026947453, 0.003511223885766218, 0.0035405423302385663, 0.0009032908415755499, 0.016806045132411605, 0.009135456702015349, 0.005817505559628584, 0.017629158297247437, 0.005834174356660754, 0.0018802984293931901, 0.005355214433866131, 0.002971687415522707, 0.008161052993948831, 0.0015134437805901096, 0.006114992582453297, 0.004003414974045376, 0.00807485785516478, 0.005858482414427411, 0.004082294832548596, 0.0003408080240484279, 0.004896137247461765, 0.0015914052422002292, 0.004996921851372398, 0.0031780310111612503, 0.007607511671701754, 0.003652726190850622, 0.0023437698916062046, 0.011483622245229069, 0.0007944998206258456, 0.0028729845265772144, 0.012362019892747364, 0.0055461392530556895, 0.000299100400079258, 0.005986335490074508, 2.7079732682615265e-05, 0.011887332132034325, 0.0009786230393680657, 0.006111456646644364, 0.01571602443394443, 0.0032339197950549264, 0.0012430127948458783, 0.008742103551146914, 0.00012328090487398122, 0.0033271401843494933, 0.0036168750217818874, 0.003606584825056613, 0.015397116515709328, 0.014819261610382016, 0.0057368937688436275, 0.008086839615993272, 0.011672979714676325, 0.0015249239798346206, 0.01606006529651623, 0.001288166312728118, 0.0021426253629348233, 0.004163035712011792, 0.01430959852109576, 0.0019985714034657896, 0.0003291646259333085, 0.0020512119897032367, 0.0008989941345346475, 0.003904417940467659, 0.004326986783898007, 0.011229713235938042, 0.008643493100129719, 0.002496936213365407, 0.0033454236823060937, 0.0041863192563085135, 0.004910928731075091, 0.012460372300323794, 0.0009541529854129524, 0.007178705992039486, 0.008167973920177235]

link1_ori_distr2 = [0.015090722273792823, 0.002347172603035519, 0.003542097480332962, 0.0029050613516978796, 0.023522783904095013, 0.002215795144450139, 0.01099600047744509, 0.003160511086052531, 0.0005160155677867984, 0.0017657610011101443, 0.00044011011730116234, 0.00812565244740715, 0.008915287893917934, 0.006530487704111132, 0.006624326859123508, 0.0006530434965374976, 0.0011830457251027493, 0.011040931010595889, 0.00782268312026524, 0.005479740599623417, 0.016644997360524827, 0.002395260323943771, 0.003799969792975033, 0.0007457853749206489, 0.0034156096285610227, 0.006960391577904623, 0.01564666027554351, 0.002582178958174919, 0.02387523726397267, 0.00579297088946773, 0.00725501926626726, 0.0019627968742109703, 0.002944197379599167, 0.003668364591852602, 0.01755720806035299, 0.024564828559011875, 0.007230433144814308, 0.014833354145716737, 0.009862039714848336, 0.01502013842423322, 0.003110470468053461, 0.016848530697735287, 0.008053078560495863, 0.006307621384590734, 0.005215041355233319, 0.006247905773424822, 0.003984297296965589, 6.328431882385299e-05, 0.0021363058164124427, 0.0022269849698050015, 0.003579130340969794, 0.005759662386435771, 0.002227165937739639, 0.001532832638730608, 0.0057503946358911096, 0.0025274702260952298, 0.0011570242962384292, 0.00547037433697084, 0.006638336579356308, 0.013209198388426824, 0.003754407281857053, 0.003277060746854188, 0.002630535611229612, 0.012707540543805407, 0.0025775243206275222, 0.020122589185844864, 0.00576860354167974, 0.01721474223129782, 0.0011370314344633092, 0.0015120103404157037, 0.0002521470186468422, 0.007225050353133922, 0.004031175755507599, 0.015049190523004907, 0.005568980923618794, 0.009746203867260661, 0.010473663663020827, 0.012797377239029739, 0.008960480595675335, 0.021136719344876154, 0.01210020147482227, 0.000970876456075298, 0.00574352103632525, 0.005580971988150107, 0.00435574392806326, 0.004382422744757419, 0.014780959656960913, 0.0014372216083838874, 0.0006547654124708924, 0.0015244341035770548, 0.02216138399047962, 0.014938999105736261, 0.019793345201970403, 0.004927085678759768, 0.0060486030745396935, 0.0014551048684076084, 0.003164061653223582, 0.009965375605735849, 0.00563106592961482, 0.0017887544995992544]

link2_ori_distr2 = [0.038364741037647865, 0.04866354305535081, 0.009914442607556695, 0.03886599417374548, 0.021140171933419882, 0.00480793778976496, 0.031064807829518926, 0.04008740681418177, 0.058121106123519506, 0.011949150609083192, 0.03997980146159774, 0.012717317073113854, 0.03693616249831122, 0.016069225591351488, 0.00924346486234695, 0.014815852765698523, 0.057275542679334546, 0.0349950168476878, 0.008575630347853802, 0.04216153353876895, 0.03967383072191788, 0.009187569919314464, 0.044715120738476166, 0.0020415340293276074, 0.009704449758043077, 0.008319119132253805, 0.03127197409732718, 0.03370470898731792, 0.03726374556716239, 0.0471733480130192, 0.0062588670341553465, 0.004298278186862459, 0.04946108764751245, 0.030897203794218453, 0.013613254867533842, 0.025919312712661347, 0.04506160202741094, 0.005604698154965448, 0.020938236257809217, 0.048169277125566956, 0.018170860191777938, 0.005147605240180875, 0.024112657244446112, 0.02169633419008643, 0.027248986991388446, 0.038085095623275445, 0.052215155772762034, 0.01972249910118007, 0.013441274179424978, 8.996794760218751e-05, 0.001961536256690821, 0.02853325127827777, 0.011311664122045095, 0.03735325789048749, 0.02386215071132336, 0.026414984416766374, 0.00783624056557286, 0.004087672202727877, 0.028634382541685044, 0.0020808570493857648, 0.003320890452279457, 0.030821433613790084, 0.03482873950436838, 0.012857198024548211, 0.0057460427971826045, 0.06515212481763732, 0.007922795010038786, 0.003891557411589752, 0.04122246191206469, 0.007204767893824626, 0.043219352602553385, 0.0270386865712543, 0.01970111258560281, 0.0382123407096735, 0.01632189293468378, 0.006236284761289235, 0.014559600380953608, 0.05320179194314756, 0.00877292687078901, 0.015191364545885766, 0.024716348672969124, 0.007816955222868738, 0.05644699337488979, 0.00669498012987646, 0.05820046136084811, 0.05676707245269452, 0.0029301893065091233, 0.0013310954009405629, 0.002490556397588106, 0.031166976116324685, 0.030995785983566226, 0.01627674225324416, 0.0016410195469367395, 0.0206311203577223, 0.01968677617961656, 0.034205028819058625, 0.01775706424324941, 0.019096729652339328, 0.02411584880984352, 0.011524229646496575]



ax = plt.hist(link1_ori_distr2, 15)
plt.title("Distribution of Link2 Orientation Distance between 100 randomly generated pairs")
plt.xlabel("quaternion distance")
plt.ylabel("count")
plt.show()

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

    ax = plt.hist(link1_ori_dist, 15)
    plt.title("Distribution of Link2 Orientation Distance between 100 randomly generated pairs")
    plt.xlabel("quaternion distance")
    plt.ylabel("count")
    plt.show()