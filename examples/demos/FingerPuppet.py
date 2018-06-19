import Sofa
import numpy as np



class FingerPuppetController(Sofa.PythonScriptController):

    def __init__(self, node, argv):
        self.argv = argv
        self.createGraph(node)
        pass
        
    def createGraph(self,rootNode):
        self.started = False
        self.root = rootNode

        ## Builds the appropriate camera matrix to compute point reprojections
        self.root.createObject('CameraSettings', name="cam_points", imageSize="640 360", f="10", M="[640 0 320 0   0 640 180 0   0 0 1 0]")
        self.root.createObject('CameraSettings', name="cam", imageSize="1280 720", f="20", M="[1280 0 640 0   0 1280 360 0   0 0 1 0]")
        ## Sets a camera in the scene
        self.root.createObject('CalibratedCamera', name="cc", cam="@cam", drawGizmo="true", freeProj="false", freeCam="false")

        self.vision = self.root.createChild("Vision")
        ## Retrieve images from the webcam
        self.vision.createObject('VideoGrabber', name="grabber_camera", cam_index="0", resolution="1280 720", async="true")
        self.vision.createObject('VideoGrabber', name='description', videoFile='data/Instructions_Demo1.png')
        self.mask = self.vision.createObject('VideoGrabber', name="mask", videoFile="data/Mask_Demo1.png")

        ## Flip the image for better mvmt cognition
        self.vision.createObject('Flip', name="flipped", img="@grabber_camera.img_out", flipCode="1")
        ## Resize for speedup in detection
        self.vision.createObject('Resize', name="resized", img="@flipped.img_out", size="640 360")
        ## Add a mask around fingers to filter out noise
        self.copyto = self.vision.createObject('CopyTo', name="masked", img="@resized.img_out", mask="@mask.img_out", useMask="true")

        ## Detect blobs on fingers
        self.detector = self.vision.createObject('FeatureDetector', name="detector", img="@masked.img_out", detectorMode="DETECT_ONLY", mask="@mask.img_out", detectorType="BLOB"\
                                                 , BLOBthresholdStep="1", BLOBminThreshold="0", BLOBmaxThreshold="50"\
                                                 , BLOBfilterByArea="true", BLOBminArea="60"\
                                                 , BLOBfilterByConvexity="false", BLOBfilterByInertia="false"\
                                                 , BLOBfilterByCircularity="true", BLOBminCircularity="0.5")

        self.vision.createObject('PointVectorConverter', name="converter", template="cvKeypoint,Vec2d", points="@detector.keypoints_out")
        
        ## Track 2D points in image
        self.tracker = self.vision.createObject('OpticalFlow', name="tracker", img="@resized.img_out", points="@converter.points_out", start="false")
        ## Project tracked points in 3D:
        self.vision.createObject('ProjectPoints', cam="@/cam_points", depth="1", points2D="@tracker.points_out")

        
        ## Show the image description (until tracking starts)
        self.addDescription = self.vision.createObject('AddWeighted', name="addImage", img="@flipped.img_out", img2="@description.img_out")

        ## display the camera stream
        self.vision.createObject('FrameViewer', img="@addImage.img_out", corners="@/cam.3DCorners")

        ## display projected points:
        self.vision.createObject('PCViewer', name="3DPoints", size="10", points="@ProjectPoints.points3D")









        ## Standard Sofa SparseGrid + FEM + solvers etc.
        translation = "0 2.8 7"
        rotation = "0 200 180"
        self.root.createObject('CollisionPipeline', name="Pipeline")
        self.root.createObject('BruteForceDetection')
        self.root.createObject('NewProximityIntersection', alarmDistance="0.06", contactDistance="0.05")
        self.root.createObject('CollisionResponse')
        self.root.createObject('CollisionGroup')
        
        self.root.createObject('EulerImplicit')
        self.root.createObject('CGLinearSolver', iterations="200", tolerance="1e-9", threshold="1e-9")

        self.puppet = self.root.createChild('FingerPuppet')

        self.puppet.createObject('MeshObjLoader', name="loader", filename="data/FingerPuppet.obj", scale="0.1", translation=translation, rotation=rotation)
        self.puppet.createObject('SparseGrid', name="grid", vertices="@loader.position", input_triangles="@loader.triangles", n="10 10 7")
        self.puppet.createObject('MechanicalObject', name="gridDOFs")
        self.puppet.createObject('UniformMass', totalMass="100")
        self.puppet.createObject('HexahedronFEMForceField', name="FF", youngModulus="5000", poissonRatio="0.4")

        ## Visual model of the puppet
        self.visu = self.puppet.createChild('Visual')
        self.visu.createObject('OglModel', name="VisualModel", fileMesh="data/FingerPuppet.obj", scale="0.1", translation=translation, rotation=rotation)
        self.visu.createObject('BarycentricMapping', name="visual-mapping", input="@../gridDOFs", output="@VisualModel")


        self.fingers = self.puppet.createChild('Fingers')
        ## Mechanical Object containing the "slaved" positions (i.e. The points mapped to the model that we want to control) (clockwise, starting with the thumb)
        self.slave = self.fingers.createObject('MechanicalObject', name='slave', position='-0.4 0.88 6.8   -1.318 0.028 7.805    -0.4799 -0.6767 7.023    0.3255 -0.6761 7.043    1.265 -0.167 8.081', showObject="true", showObjectScale="10")


        ## The slave points projected on the line of sight -> master points
        self.markers = self.puppet.createChild('Observations')
        self.masterPoints = self.markers.createObject('OrthoProj', name="masterPoints", cam="@/cam", observations="@/Vision/ProjectPoints.points3D", slavePts="@/FingerPuppet/Fingers/slave.position", method="ORTHO")

        self.root.getRootContext().animate = True

        pass

    def onBeginAnimationStep(self, deltaTime):
        if self.started:
            self.slave.position = self.slaveMO.position
            self.slave.init()
            self.slave.reinit()
            self.slave.bwdInit()
        pass

    def onKeyPressed(self,k):
        if k == 'D':
            if len(self.tracker.points_out) != 5:
                return

	    print 'Starting Simulation: '+k
            
	    sys.stdout.flush()
            self.started = True
            masterMO = self.markers.createObject('MechanicalObject', name="master", position="@masterPoints.points_out", showObject="true", showObjectScale="10", showColor="1 0 0 1" )
            masterMO.init()
            masterMO.reinit()
            masterMO.bwdInit()


            self.msc = self.puppet.createChild('MasterSlaveControl')
            self.slaveMO = self.msc.createObject('MechanicalObject', name="MO", position="@/FingerPuppet/Fingers/slave.position", showObject="true", showObjectScale="10")
            self.slaveMO.init()
            self.slaveMO.reinit()
            self.slaveMO.bwdInit()
            
            ## Create springs between the "master" points (image features reprojected in 3D) and the "slave" points (pre-placed 3D markers mapped on the model)
            rssff = self.msc.createObject('RestShapeSpringsForceField', name="springs", stiffness="100000", external_rest_shape="@/FingerPuppet/Observations/master", springColor="0 1 0 1", drawSpring="true")
            rssff.init()
            rssff.reinit()
            rssff.bwdInit()
            bm = self.msc.createObject('BarycentricMapping', name="BM", input="@../gridDOFs", output="@./")        
            bm.init()
            bm.reinit()
            bm.bwdInit()


            ## mask all unnecessary stuff, start the feature tracking...
            self.addDescription.listening = "false"
            self.mask.listening = "false"

            self.detector.isActive = "false"
            self.copyto.isActive = "false"
            self.tracker.start = "true"
            self.addDescription.isActive = "false"
            self.root.gravity = '0 -9.8 0'

        pass

    
        
def createScene(node):
    node.createObject('RequiredPlugin', pluginName='DataAcquisition')
    node.createObject('RequiredPlugin', pluginName='ImageProcessing')
    
    node.gravity = '0 0 0'
    node.dt = '0.01'

    try :
        sys.argv[0]
    except :
        sys.argv = ['FingerPuppet']
    else :
        sys.argv = sys.argv

    pyController = FingerPuppetController(node,sys.argv)
    return

