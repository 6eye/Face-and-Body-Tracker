namespace Microsoft.Samples.Kinect.FaceBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Media3D;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Face;
    using OSC.NET;

    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        
        private KinectSensor sensor = null; 		// Currently used KinectSensor
        private BodyFrameSource bodySource = null; 	// Body frame source to get a BodyFrameReader
        private BodyFrameReader bodyReader = null; 	// Body frame reader to get body frames

        private HighDefinitionFaceFrameSource highDefinitionFaceFrameSource = null; // HighDefinitionFaceFrameSource to get a reader and a builder from.
																					// Also to set the currently tracked user id to get High Definition Face Frames of
        
        private HighDefinitionFaceFrameReader highDefinitionFaceFrameReader = null; // HighDefinitionFaceFrameReader to read HighDefinitionFaceFrame to get FaceAlignment

        private int bodyCount;			// Number of bodies tracked
        private Body[] bodies = null;	// Array for the bodies

		private Point3D[] instrumentXYZ = new Point3D[] {Point3D(0.22, -0.17, 1.80), Point3D(-0.26, -0.18, 1.77), Point3D(-0.80, -0.18, 1.81)}; // These are the (x, y, z) coordinates of the instruments from Kinect POV		      
		private Point3D audienceXYZ = new Point3D(4, 4, 4); 	// Audience position

        private FaceAlignment currentFaceAlignment = null; 		// FaceAlignment is the result of tracking a face, it has face animations location and orientation
        private FaceModel currentFaceModel = null;				// FaceModel is a result of capturing a face
        private Body currentTrackedBody = null;					// The currently tracked body
        private ulong currentTrackingId = 0;					
        private string currentBuilderStatus = string.Empty;		// Gets or sets the current tracked user id

        private List<Tuple<JointType, JointType>> bones;		// Definition of bones

        private CoordinateMapper coordinateMapper = null;		// Coordinate mapper to map one type of point to another
        private int displayWidth;								// Width of display (depth space)
        private int displayHeight;								// Height of display (depth space)
		
        private string remoteIP = "10.0.0.4";					// Open Sound Control (OSC) remote IP
        private int remotePort = 9000;							// OSC remote port
		private OSCTransmitter osc;								// OSC trasmitter

        public MainWindow()
        {
            this.sensor = KinectSensor.GetDefault();
            this.bodyCount = this.sensor.BodyFrameSource.BodyCount;
            this.coordinateMapper = this.sensor.CoordinateMapper;								// The coordinate mapper
            FrameDescription frameDescription = this.sensor.DepthFrameSource.FrameDescription;	// The depth (display) extents
            this.displayWidth = frameDescription.Width;											// Size of joint space
            this.displayHeight = frameDescription.Height;
            this.InitializeComponent();
            this.DataContext = this;
        }

        public event PropertyChangedEventHandler PropertyChanged; 	// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data

        private ulong CurrentTrackingId								// Gets or sets the current tracked user id
        {
            get
            {
                return this.currentTrackingId;
            }
            set
            {
                this.currentTrackingId = value;

            }
        }

        private static double VectorLength(CameraSpacePoint point)	// Returns the length of a vector from origin
        {
            var result = Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2);
            result = Math.Sqrt(result);

            return result;
        }

        private static Body FindClosestBody(BodyFrame bodyFrame)	// Finds the closest body from the sensor if any
        {
            Body result = null;
            double closestBodyDistance = double.MaxValue;
            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);
			
            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    var currentLocation = body.Joints[JointType.SpineBase].Position;
                    var currentDistance = VectorLength(currentLocation);
                    if (result == null || currentDistance < closestBodyDistance)
                    {
                        result = body;
                        closestBodyDistance = currentDistance;
                    }
                }
            }

            return result;
        }

        private static Body FindBodyWithTrackingId(BodyFrame bodyFrame, ulong trackingId) // Find if there is a body tracked with the given trackingId
        {
            Body result = null;
            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);
			
            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    if (body.TrackingId == trackingId)
                    {
                        result = body;
                        break;
                    }
                }
            }
			
            return result;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            osc = new OSCTransmitter(remoteIP, remotePort);		// Initializes the OSC transmitter
            this.InitializeHDFace();
        }

        private void InitializeHDFace() 						// Initialize Kinect object
        {
            this.sensor = KinectSensor.GetDefault();
            this.bodySource = this.sensor.BodyFrameSource;
            this.bodyReader = this.bodySource.OpenReader();
            this.bodyReader.FrameArrived += this.BodyReader_FrameArrived;

            this.highDefinitionFaceFrameSource = new HighDefinitionFaceFrameSource(this.sensor);
            this.highDefinitionFaceFrameSource.TrackingIdLost += this.HdFaceSource_TrackingIdLost;

            this.highDefinitionFaceFrameReader = this.highDefinitionFaceFrameSource.OpenReader();
            this.highDefinitionFaceFrameReader.FrameArrived += this.HdFaceReader_FrameArrived;

            this.currentFaceModel = new FaceModel();
            this.currentFaceAlignment = new FaceAlignment();

            this.sensor.Open();
        }

        private static void ExtractRotationInDegrees(Vector4 rotQuaternion, out int pitch, out int yaw, out int roll) // Convert face rotation quaternion to Euler angles in degrees
        {
            double x = rotQuaternion.X;
            double y = rotQuaternion.Y;
            double z = rotQuaternion.Z;
            double w = rotQuaternion.W;

            pitch = (int)Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
            yaw = (int)Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
            roll = (int)Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;
        }

		private static string getHandProximity(var handPos, double threshold)	// Returns the hand distance in relation to the three instruments and returns the closest instrument
																				// If the instrument are outside the threshold, returns "far"																				
		{
			float[] handDistance = new float[3];								// Array of hand distance to each instrument
			
			for (int j = 0; j < 3; j++)
			{
				float handDistance[j] = (float)Math.Sqrt(Math.Pow(handPos.X - instrumentXYZ[j].X, 2) + Math.Pow(handPos.Y - instrumentXYZ[j].Y, 2) + Math.Pow(handPos.Z - instrumentXYZ[j].Z, 2));
			}
			
			float handMinDistance = handDistance.Min();							// Find the closest instrument
									
			if (handMinDistance <= threshold) 									// Compare to the proximity threshold
			{
				for (int j = 0; j < 3; j++) 
				{
					if (handMinDistance == handDistance[j])
						{
							string handProximity = "instrument_" + j.ToString();// Hand is within the proximity threshold of instrument #j
						}
				}
			}
			
			else
			{
				string handProximity = "far";									// Hand is outside the proximity threshold of all instruments
			}	
			return handProximity;	
		}	

        void OSCFaceOrientation(string pointOfInterest)							// Sends the instrument towards which the performer is looking																			
        {
            OSCMessage msg = new OSCMessage("/faceOrientation");		
            msg.Append(pointOfInterest); 						// Face orientation, if facing towards an instrument
            osc.Send(msg);
        }

        void OSCFaceTrackState(int state)						// Sends the updated face tracking state (0 or 1)
        {
            OSCMessage msg = new OSCMessage("/faceTrackState");
            msg.Append(state);
            osc.Send(msg);
        }

        void OSCBodyTrackState(int state)						// Sends the updated body tracking state (0 or 1)
        {
            OSCMessage msg = new OSCMessage("/bodyTrackState");
            msg.Append(state);
			osc.Send(msg);
        }

		void OSCJointPos(var jointPos, string jointName) 		// Sends the Joint (X, Y, Z) coordinates
		{
			OSCMessage msg = new OSCMessage("/joint");
			msg.Append(jointName);
			msg.Append((float)jointPos.X);
			msg.Append((float)jointPos.Y);
			msg.Append((float)jointPos.Z);
			osc.Send(msg);
		}

		void OSCBodyOrientation(var lean, var neckOrientation)	// Sends  the body lean (X, Y) and head orientation (Pitch, Yaw, Roll)
		{
			OSCMessage msgOrientation = new OSCMessage("/bodyOrientation");
	
			msgOrientation.Append("lean");
			msgOrientation.Append((float)lean.X);													// Sideways lean (-1,1)
			msgOrientation.Append((float)lean.Y);													// Front-back lean (-1,1)

			int headPitch, headYaw, headRoll;
			ExtractRotationInDegrees(neckOrientation, out headPitch, out headYaw, out headRoll);	// Extract the head orientation							
			msgOrientation.Append("neckOrientation");
			msgOrientation.Append(headPitch);														// Head pitch in angles
			msgOrientation.Append(headYaw);															// Head yaw
			msgOrientation.Append(headRoll);														// Head roll
                        
			osc.Send(msgOrientation);
		}
		
		void OSCHandProximity(var[] handPos)
		{
			OSCMessage msgHandProximity = new OSCMessage("/handProximity"); // OSC message containing the hand proximity in relation to the instruments   
			for (int i = 0; i < 2; i++)
			{
				if (i == 0)
				{
					msgHandProximity.Append("handLeft");
				}
				else 
				{
					msgHandProximity.Append("handRight");	
				}
				string handProximity = getHandProximity(handPos[i]);
				msgHandProximity.Append(handProximity);
				osc.Send(msgHandProximity);		
			}
		}
		
        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;  
            var frameReference = e.FrameReference;

            using (var frame = frameReference.AcquireFrame())
            {
                if (frame == null)
                {
                    OSCBodyTrackState(0);
                    return;
                }

                if (frame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[frame.BodyCount];
                    }
                    
                    frame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }

                if (dataReceived)
                {   
                    Body selectedBody = FindClosestBody(frame);

                    if (selectedBody == null)
                    {
                        return;
                    }

                    this.currentTrackedBody = selectedBody;
                    this.CurrentTrackingId = selectedBody.TrackingId;
                    this.highDefinitionFaceFrameSource.TrackingId = this.CurrentTrackingId;

                    if (this.currentTrackedBody != null)
                    {
                        this.currentTrackedBody = FindBodyWithTrackingId(frame, this.CurrentTrackingId);
						OSCBodyTrackState(1);
				
						// Body Orientations: Torso lean and neck orientation						
						// Lean (X, Y) ranges [-1, 1]				
						var lean = this.currentTrackedBody.Lean;
						var neckOrientation = this.currentTrackedBody.JointOrientations[JointType.Neck].Orientation;
						OSCOrientation(lean, neckOrientation);
	
						// Hand to instrument proximities. Require both hands to be tracked
                        if (this.currentTrackedBody.Joints[JointType.HandTipLeft].TrackingState.ToString().Equals("Tracked") == true &&
							this.currentTrackedBody.Joints[JointType.HandTipRight].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var[] handPos = new var[2];
							handPos[0] = this.currentTrackedBody.Joints[JointType.HandTipLeft].Position;
							handPos[1] = this.currentTrackedBody.Joints[JointType.HandTipRight].Position;
							OSCHandProximity(handPos);
						}

						// Send the (X, Y, Z) coordinates of a selected list of Joints
                        if (this.currentTrackedBody.Joints[JointType.HandLeft].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var jointPos = this.currentTrackedBody.Joints[JointType.HandLeft].Position;
							OSCJointPos(jointPos, "handLeft");                       
                        }
                        if (this.currentTrackedBody.Joints[JointType.HandRight].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var jointPos = this.currentTrackedBody.Joints[JointType.HandRight].Position;
							OSCJointPos(jointPos, "handRight");  
                        }                     
                        if (this.currentTrackedBody.Joints[JointType.ElbowLeft].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var jointPos = this.currentTrackedBody.Joints[JointType.ElbowLeft].Position;
							OSCJointPos(jointPos, "elbowLeft");  
                        }
                        if (this.currentTrackedBody.Joints[JointType.ElbowRight].TrackingState.ToString().Equals("Tracked") == true)
                        {
							var jointPos = this.currentTrackedBody.Joints[JointType.ElbowRight].Position;
							OSCJointPos(jointPos, "elbowRight");  
                        }
                        if (this.currentTrackedBody.Joints[JointType.ShoulderLeft].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var jointPos = this.currentTrackedBody.Joints[JointType.ShoulderLeft].Position;
                            OSCJointPos(jointPos, "ShoulderLeft");  
                        }
                        if (this.currentTrackedBody.Joints[JointType.ShoulderRight].TrackingState.ToString().Equals("Tracked") == true)
                        {
							var jointPos = this.currentTrackedBody.Joints[JointType.ShoulderRight].Position;
							OSCJointPos(jointPos, "ShoulderRight");  
                        }
                        if (this.currentTrackedBody.Joints[JointType.Head].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var jointPos = this.currentTrackedBody.Joints[JointType.Head].Position;
							OSCJointPos(jointPos, "head");  
                        }
                        if (this.currentTrackedBody.Joints[JointType.SpineBase].TrackingState.ToString().Equals("Tracked") == true)
                        {
                            var jointPos = this.currentTrackedBody.Joints[JointType.SpineBase].Position;
                            OSCJointPos(jointPos, "spineBase");
                        }
                    }
                }
            }
        }

        private void HdFaceSource_TrackingIdLost(object sender, TrackingIdLostEventArgs e)	// This event is fired when a tracking is lost for a body tracked by HDFace Tracker
        {
            var lostTrackingID = e.TrackingId;
            if (this.CurrentTrackingId == lostTrackingID)
            {
                this.CurrentTrackingId = 0;
                this.currentTrackedBody = null;     
                this.highDefinitionFaceFrameSource.TrackingId = 0;

                OSCFaceTrackState(0);
            }
        }

		
		
		private static string inferPointOfInterest(var head, CameraSpacePoint noseTip) // Infers the point of interest based on head, nose and instrument positions
		{
			Point3D headPos = new Point3D(head.X, head.Y, head.Z);			// The (X, Y, Z) position of the center of the head
			Point3D nosePos = new Point3D(noseTip.X, noseTip.Y, noseTip.Z); // The (X, Y, Z) position of nose

			Vector3D headNoseVector = headPos - nosePos;					// The vector from the center of the head center to the nose
			Vector3D[] noseInstrumentVector = new Vector3D[3];				// The vector array from nose to the instruments
			double[] faceToInstrumentAngle = new double[3]					// The angular differences between headNoseVector and noseInstrumentVectors
			
			for (int i = 1; i < 3; i++)
			{
				noseInstrumentVector[i] = nosePos - instrumentXYZ[i];		// The vector from nose to instrument
				faceToInstrumentAngle[i] = Vector3D.AngleBetween(headNoseVector, noseInstrumentVector[i]); // The angular difference between nose and instrument
			}
			
			double min = faceToInstrumentAngle.Min();						// The smallest angular difference between nose vector and instrument positions
			string pointOfInterest = "elsewhere";							// Point of interest initialized to "elsewhere"
			double thresholdAngle = 40;										// Threshold angle for point of interest
			
			for (int i = 1; i < 3; i++)
			{
				if (thresholdAngle < faceToInstrumentAngle[i])
				{
					pointOfInterest = "instrument_" + i.ToString();			// The instrument that is within the treshold
				}
			}
			return pointOfInterest;
		}
		
        private void HdFaceReader_FrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)	// This event is fired when a new HDFace frame is ready for consumption
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame == null || !frame.IsFaceTracked)
                {
                    OSCFaceTrackState(0); 
                    return;
                }

                frame.GetAndRefreshFaceAlignmentResult(this.currentFaceAlignment);
                OSCFaceTrackState(1);

                if (this.currentTrackedBody.Joints[JointType.Head].TrackingState.ToString().Equals("Tracked") == true)
                {
					// Send an OSC message containing information which instrument the performer is looking
					
                    var head = this.currentTrackedBody.Joints[JointType.Head].Position;
                    var nose = this.currentFaceModel.CalculateVerticesForAlignment(this.currentFaceAlignment);
					CameraSpacePoint noseTip = nose[(int)HighDetailFacePoints.NoseTip];
					
					string pointOFInterest = inferPointOfInterest(head, noseTip);

					OSCFaceOrientation(pointOfInterest);                   
                 }

                if (this.currentFaceAlignment.Quality.ToString().Equals("High") == true)	
                {					
					// Send a single OSC message containing all the animation units
					OSCMessage msgAnimUnit = new OSCMessage("/animationUnits");

                    var jawOpen = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.JawOpen];
                    var jawSlideRight = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.JawSlideRight];
                    var leftEyeClosed = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LefteyeClosed];
                    var rightEyeClosed = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.RighteyeClosed];
                    var leftCheekPuff = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LeftcheekPuff];
                    var rightCheekPuff = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.RightcheekPuff];
                    var leftEyebrowLowerer = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LefteyebrowLowerer];
                    var rightEyebrowLowerer = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.RighteyebrowLowerer];
                    var lipCornerDepressorLeft = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipCornerDepressorLeft];
                    var lipCornerDepressorRight = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipCornerDepressorRight];
                    var lipCornerPullerLeft = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipCornerPullerLeft];
                    var lipCornerPullerRight = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipCornerPullerRight];
                    var lipPucker = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipPucker];
                    var lipStretcherLeft = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipStretcherLeft];
                    var lipStretcherRight = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LipStretcherRight];
                    var lowerlipDepressorLeft = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LowerlipDepressorLeft];
                    var lowerlipDepressorRight = this.currentFaceAlignment.AnimationUnits[FaceShapeAnimations.LowerlipDepressorRight];
								
                    msgAnimUnit.Append("jawOpen");
                    msgAnimUnit.Append(jawOpen);
                    msgAnimUnit.Append("jawSlideRight");
                    msgAnimUnit.Append(jawSlideRight);
                    msgAnimUnit.Append("leftEyeClosed"); 
                    msgAnimUnit.Append(leftEyeClosed);
                    msgAnimUnit.Append("rightEyeClosed"); 
                    msgAnimUnit.Append(rightEyeClosed);
                    msgAnimUnit.Append("leftCheekPuff");
                    msgAnimUnit.Append(leftCheekPuff);
                    msgAnimUnit.Append("rightCheekPuff"); 
                    msgAnimUnit.Append(rightCheekPuff);
                    msgAnimUnit.Append("leftEyebrowLowerer"); 
                    msgAnimUnit.Append(leftEyebrowLowerer);
                    msgAnimUnit.Append("rightEyebrowLowerer");
                    msgAnimUnit.Append(rightEyebrowLowerer);
                    msgAnimUnit.Append("lipCornerDepressorLeft"); 
                    msgAnimUnit.Append(lipCornerDepressorLeft);
                    msgAnimUnit.Append("lipCornerDepressorRight"); 
                    msgAnimUnit.Append(lipCornerDepressorRight);
                    msgAnimUnit.Append("lipCornerPullerLeft"); 
                    msgAnimUnit.Append(lipCornerPullerLeft);
                    msgAnimUnit.Append("lipCornerPullerRight"); 
                    msgAnimUnit.Append(lipCornerPullerRight);
                    msgAnimUnit.Append("lipPucker"); 
                    msgAnimUnit.Append(lipPucker);
                    msgAnimUnit.Append("lipStretcherLeft"); 
                    msgAnimUnit.Append(lipStretcherLeft);
                    msgAnimUnit.Append("lipStretcherRight"); 
                    msgAnimUnit.Append(lipStretcherRight);
                    msgAnimUnit.Append("lowerLipDepressorLeft"); 
                    msgAnimUnit.Append(lowerlipDepressorLeft);
                    msgAnimUnit.Append("lowerLipDepressorRight"); 
                    msgAnimUnit.Append(lowerlipDepressorRight);

                    osc.Send(msgAnimUnit);
                }
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e) 	// Execute start up tasks
        {
            if (this.bodyReader != null)
            {
                this.bodyReader.FrameArrived += this.BodyReader_FrameArrived;
            }
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)	// Execute shutdown tasks
        {
            if (this.bodyReader != null)
            {
                this.bodyReader.Dispose();
                this.bodyReader = null;
            }
            if (this.sensor != null)
            {
                this.sensor.Close();
                this.sensor = null;
            }
        }

    }
}
