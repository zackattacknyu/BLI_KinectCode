namespace BLIKinect
{
    using System;
    using System.ComponentModel;
    using System.Globalization;
    using System.IO;
    using System.Text;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Data;
    using System.Windows.Media;
    using System.Windows.Media.Media3D;
    using System.Windows.Media.Imaging;
    using System.Windows.Threading;
    using System.Collections.Generic;
    using Microsoft.Win32;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit;
    using Microsoft.Kinect.Toolkit.Fusion;
    using AviFile;
    using System.Drawing;

    /// <summary>
    /// A struct containing depth image pixels and frame timestamp
    /// </summary>
    internal struct DepthData
    {
        public DepthImagePixel[] DepthImagePixels;
        public long FrameTimestamp;
    }
    
    public class KinectReconstruction : IDisposable
    {
        #region Constants

        /// <summary>
        /// Max tracking error count, will reset the reconstruction if tracking errors
        /// reach the number
        /// </summary>
        private const int MaxTrackingErrors = 100;

        /// <summary>
        /// Time threshold to reset the reconstruction if tracking can't be restored within it.
        /// This value is valid if GPU is used
        /// </summary>
        private const int ResetOnTimeStampSkippedMillisecondsGPU = 1000;

        /// <summary>
        /// Time threshold to reset the reconstruction if tracking can't be restored within it.
        /// This value is valid if CPU is used.
        /// </summary>
        private const int ResetOnTimeStampSkippedMillisecondsCPU = 6000;

        /// <summary>
        /// If set true, will automatically reset the reconstruction when MaxTrackingErrors have occurred
        /// </summary>
        private const bool AutoResetReconstructionWhenLost = false;

        /// <summary>
        /// Event interval for FPS timer
        /// </summary>
        private const int FpsInterval = 5;

        /// <summary>
        /// The reconstruction volume processor type. This parameter sets whether AMP or CPU processing
        /// is used. Note that CPU processing will likely be too slow for real-time processing.
        /// </summary>
        private const ReconstructionProcessor ProcessorType = ReconstructionProcessor.Amp;

        /// <summary>
        /// Format of depth image to use
        /// </summary>
        private const DepthImageFormat ImageFormat = DepthImageFormat.Resolution640x480Fps30;

        /// <summary>
        /// Format of color image to use
        /// </summary>
        private const ColorImageFormat CImageFormat = ColorImageFormat.RgbResolution1280x960Fps12;

        private const ColorImageFormat IRImageFormat = ColorImageFormat.InfraredResolution640x480Fps30;

        /// <summary>
        /// Width of the color image
        /// </summary>
        private const int ColorImageWidth = 1280;

        /// <summary>
        /// Height of the color image
        /// </summary>
        private const int ColorImageHeight = 960;

        #endregion

        #region Fields

        /// <summary>
        /// Track whether Dispose has been called
        /// </summary>
        public bool disposed;

        /// <summary>
        /// Saving mesh flag
        /// </summary>
        public bool savingMesh;

        /// <summary>
        /// Image width of depth frame
        /// </summary>
        public int width = 0;

        /// <summary>
        /// Image height of depth frame
        /// </summary>
        public int height = 0;

        /// <summary>
        /// The counter for image process failures
        /// </summary>
        public int trackingErrorCount = 0;

        /// <summary>
        /// The counter for frames that have been processed
        /// </summary>
        public int processedFrameCount = 0;

        /// <summary>
        /// Timestamp of last depth frame in milliseconds
        /// </summary>
        public long lastFrameTimestamp = 0;

        /// <summary>
        /// Timer to count FPS
        /// </summary>
        public DispatcherTimer fpsTimer;

        /// <summary>
        /// Timer stamp of last computation of FPS
        /// </summary>
        public DateTime lastFPSTimestamp;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        public KinectSensor sensor;

        /// <summary>
        /// Kinect sensor chooser object
        /// </summary>
        public KinectSensorChooser sensorChooser;

        /// <summary>
        /// The Kinect Fusion volume
        /// </summary>
        public Reconstruction volume;

        /// <summary>
        /// Intermediate storage for the depth float data converted from depth image frame
        /// </summary>
        public FusionFloatImageFrame depthFloatFrame;

        /// <summary>
        /// Per-pixel alignment values
        /// </summary>
        public FusionFloatImageFrame deltaFromReferenceFrame;

        /// <summary>
        /// minT alignment energy for frame
        /// </summary>
        public float alignmentEnergy;

        /// <summary>
        /// Shaded surface frame from shading point cloud frame
        /// </summary>
        public FusionColorImageFrame shadedSurfaceFrame;

        /// <summary>
        /// Shaded surface normals frame from shading point cloud frame
        /// </summary>
        public FusionColorImageFrame shadedSurfaceNormalsFrame;

        /// <summary>
        /// Calculated point cloud frame from image integration
        /// </summary>
        public FusionPointCloudImageFrame pointCloudFrame;

        /// <summary>
        /// Bitmap contains depth float frame data for rendering
        /// </summary>
        public WriteableBitmap depthFloatFrameBitmap;

        //// <summary>
        //// Bitmap contains delta from reference frame data for rendering
        //// </summary>
        //public WriteableBitmap deltaFromReferenceFrameBitmap;

        /// <summary>
        /// Bitmap contains the color image data
        /// </summary>
        public WriteableBitmap colorFrameBitmap;

        /// <summary>
        /// Image used when displaying the depth from the camera specified by the correction
        /// </summary>
        public WriteableBitmap depthFromCameraBitmap;
        /// <summary>
        /// Array that stores the most recent color data from the color camera
        /// </summary>
        public byte[] colorData;

        /// <summary>
        /// Stores the current image number for saving images
        /// </summary>
        public int imageNumber = 0;

        /// <summary>
        /// Bitmap contains shaded surface frame data for rendering
        /// </summary>
        public WriteableBitmap shadedSurfaceFrameBitmap;

        /// <summary>
        /// Pixel buffer of depth float frame with pixel data in float format
        /// </summary>
        public float[] depthFloatFrameDepthPixels;

        /// <summary>
        /// Pixel buffer of delta from reference frame with pixel data in float format
        /// </summary>
        //public float[] deltaFromReferenceFrameFloatPixels;

        /// <summary>
        /// Pixel buffer of depth float frame with pixel data in 32bit color
        /// </summary>
        public int[] depthFloatFramePixels;

        //// <summary>
        //// Pixel buffer of delta from reference frame in 32bit color
        //// </summary>
        //public int[] deltaFromReferenceFramePixels;

        /// <summary>
        /// Pixels buffer of shaded surface frame in 32bit color
        /// </summary>
        public int[] shadedSurfaceFramePixels;

        /// <summary>
        /// Frame data is being processed
        /// </summary>
        public bool processing = false;

        /// <summary>
        /// The transformation between the world and camera view coordinate system
        /// </summary>
        public Matrix4 worldToCameraTransform;

        /// <summary>
        /// The default transformation between the world and volume coordinate system
        /// </summary>
        public Matrix4 defaultWorldToVolumeTransform;

        /// <summary>
        /// To display shaded surface normals frame instead of shaded surface frame
        /// </summary>
        public bool displayNormals;

        /// <summary>
        /// Pause or resume image integration
        /// </summary>
        public bool pauseIntegration;

        /// <summary>
        /// Depth image is mirrored
        /// </summary>
        public bool mirrorDepth;

        /// <summary>
        /// If near mode is enabled
        /// </summary>
        public bool nearMode;

        /// <summary>
        /// Minimum depth distance threshold in meters. Depth pixels below this value will be
        /// returned as invalid (0). Min depth must be positive or 0.
        /// </summary>
        public float minDepthClip = FusionDepthProcessor.DefaultMinimumDepth;

        /// <summary>
        /// Maximum depth distance threshold in meters. Depth pixels above this value will be
        /// returned as invalid (0). Max depth must be greater than 0.
        /// </summary>
        public float maxDepthClip = FusionDepthProcessor.DefaultMaximumDepth;

        /// <summary>
        /// Image integration weight
        /// </summary>
        public short integrationWeight = FusionDepthProcessor.DefaultIntegrationWeight;

        /// <summary>
        /// The reconstruction volume voxel density in voxels per meter (vpm)
        /// 1000mm / 256vpm = ~3.9mm/voxel
        /// </summary>
        public float voxelsPerMeter = 640.0f;

        /// <summary>
        /// The reconstruction volume voxel resolution in the X axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m wide
        /// </summary>
        public int voxelsX = 512;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Y axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m high
        /// </summary>
        public int voxelsY = 384;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Z axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m deep
        /// </summary>
        public int voxelsZ = 512;

        /// <summary>
        ///  This is the matrix that converts from the color camera to the external camera if attached.
        /// </summary>
        //public Matrix4 correctionMatrix;
        //public Matrix4 otherCameraWorldToCamera;
        //public Boolean correctionSet = false;
        private CameraUtilities cameraUtils = new CameraUtilities();

        /// <summary>
        /// Parameter to translate the reconstruction based on the minimum depth setting. When set to
        /// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
        /// Setting this true in the constructor will move the volume forward along +Z away from the
        /// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
        /// by setting a non-identity world-volume transformation in the ResetReconstruction call.
        /// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
        /// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
        /// when the majority of a small volume is inside this distance.
        /// </summary>
        public bool translateResetPoseByMinDepthThreshold = true;

        public bool processingFrames = true;

        public CameraParameters camParam= new CameraParameters(.8203125f, 1.09375f, .5f, .5f);
        //public CameraParameters camParam = new CameraParameters(.7416f, 1.1976f, .5f, .5f);

        public FusionPointCloudImageFrame fpcif;
        public Byte[] distances;
        float[] depthDataBuffer;
        public MainWindow main;
        public Boolean currentlyUsingIR = false;


        #endregion

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public KinectReconstruction(MainWindow main)
        {
            this.main = main;
        }

        /// <summary>
        /// Finalizes an instance of the MainWindow class.
        /// This destructor will run only if the Dispose method does not get called.
        /// </summary>
        ~KinectReconstruction()
        {
            this.Dispose(false);
        }

        /// <summary>
        /// Property change event
        /// </summary>
        //public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Dispose resources
        /// </summary>
        public void Dispose()
        {
            this.Dispose(true);

            // This object will be cleaned up by the Dispose method.
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Frees all memory associated with the FusionImageFrame.
        /// </summary>
        /// <param name="disposing">Whether the function was called from Dispose.</param>
        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                if (disposing)
                {
                    if (null != this.depthFloatFrame)
                        this.depthFloatFrame.Dispose();

                    if (null != this.deltaFromReferenceFrame)
                        this.deltaFromReferenceFrame.Dispose();

                    if (null != this.shadedSurfaceFrame)
                        this.shadedSurfaceFrame.Dispose();

                    if (null != this.shadedSurfaceNormalsFrame)
                        this.shadedSurfaceNormalsFrame.Dispose();

                    if (null != this.pointCloudFrame)
                        this.pointCloudFrame.Dispose();

                    if (null != this.volume)
                        this.volume.Dispose();
                }
            }

            this.disposed = true;
        }

        /// <summary>
        /// Render Fusion color frame to UI
        /// </summary>
        /// <param name="colorFrame">Fusion color frame</param>
        /// <param name="colorPixels">Pixel buffer for fusion color frame</param>
        /// <param name="bitmap">Bitmap contains color frame data for rendering</param>
        /// <param name="image">UI image component to render the color frame</param>
        private static void RenderColorImage(FusionColorImageFrame colorFrame, ref int[] colorPixels, ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == colorFrame)
            {
                return;
            }

            if (null == colorPixels || colorFrame.PixelDataLength != colorPixels.Length)
            {
                // Create pixel array of correct format
                colorPixels = new int[colorFrame.PixelDataLength];
            }

            if (null == bitmap || colorFrame.Width != bitmap.Width || colorFrame.Height != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }


            // Copy pixel data to pixel buffer
            colorFrame.CopyPixelDataTo(colorPixels);

            // Write pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, colorFrame.Width, colorFrame.Height),
                        colorPixels,
                        bitmap.PixelWidth * sizeof(int),
                        0);

        }

        /// <summary>
        /// Render Fusion depth float frame to UI
        /// </summary>
        /// <param name="depthFloatFrame">Fusion depth float frame</param>
        /// <param name="depthPixels">Pixel buffer for depth float frame with pixel in depth</param>
        /// <param name="colorPixels">Pixel buffer for depth float frame with pixel in colors</param>
        /// <param name="bitmap">Bitmap contains depth float frame data for rendering</param>
        /// <param name="image">UI image component to render depth float frame to</param>
        private static void RenderDepthFloatImage(FusionFloatImageFrame depthFloatFrame, ref float[] depthPixels, ref int[] colorPixels, ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == depthFloatFrame)
            {
                return;
            }

            if (null == depthPixels || depthFloatFrame.PixelDataLength != depthPixels.Length)
            {
                // Create depth pixel array of correct format
                depthPixels = new float[depthFloatFrame.PixelDataLength];
            }

            if (null == colorPixels || depthFloatFrame.PixelDataLength != colorPixels.Length)
            {
                // Create colored pixel array of correct format
                colorPixels = new int[depthFloatFrame.PixelDataLength];
            }

            if (null == bitmap || depthFloatFrame.Width != bitmap.Width || depthFloatFrame.Height != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(depthFloatFrame.Width, depthFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }
            depthFloatFrame.CopyPixelDataTo(depthPixels);

            // Calculate color pixels based on depth of each pixel

            for (int i = 0; i < depthPixels.Length; i++)
            {
                float depth = depthPixels[i]*1000;
                //int intensity = (depth >= minRange) ? ((int)(((depth - minRange) / range) * 256.0f) % 256) : 0;

                colorPixels[i] = 0;
                colorPixels[i] += (int)(depth/256); // blue

                //intensity *= 256;
                colorPixels[i] += (int)(depth%256)*256; // green

                //intensity *= 256;
                //colorPixels[i] += intensity; // red
            }

            // Copy colored pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, depthFloatFrame.Width, depthFloatFrame.Height),
                        colorPixels,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }


        private static void RenderDepthFloatImageFromOtherCamera(FusionPointCloudImageFrame depthFloatFrame, ref float[] depthPixels, ref int[] colorPixels, ref WriteableBitmap bitmap, System.Windows.Controls.Image image)
        {
            if (null == depthFloatFrame) return;


            if (null == depthPixels || depthFloatFrame.PixelDataLength != depthPixels.Length)
            {
                // Create depth pixel array of correct format
                depthPixels = new float[depthFloatFrame.PixelDataLength];
            }

            if (null == colorPixels || depthFloatFrame.PixelDataLength != colorPixels.Length)
            {
                // Create colored pixel array of correct format
                colorPixels = new int[depthFloatFrame.PixelDataLength];
            }

            if (null == bitmap || depthFloatFrame.Width != bitmap.Width || depthFloatFrame.Height != bitmap.Height)
            {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(depthFloatFrame.Width, depthFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                image.Source = bitmap;
            }

            float[] pixels = new float[depthFloatFrame.PixelDataLength * 6];

            depthFloatFrame.CopyPixelDataTo(pixels);



            // Calculate color pixels based on depth of each pixel
            float range = 4.0f;
            float minRange = 0.0f;


            for (int i = 0; i < colorPixels.Length; i++)
            {
                float depth = pixels[i * 6] + pixels[i * 6 + 1] + pixels[i * 6 + 2];
                int intensity = (depth >= minRange) ? ((int)(((depth - minRange) / range) * 256.0f) % 256) : 0;

                colorPixels[i] = 0;
                colorPixels[i] += intensity; // blue

                intensity *= 256;
                colorPixels[i] += intensity; // green

                intensity *= 256;
                colorPixels[i] += intensity; // red
            }

            // Copy colored pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, depthFloatFrame.Width, depthFloatFrame.Height),
                        colorPixels,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }

        /// <summary>
        /// Save mesh in binary .STL file
        /// </summary>
        /// <param name="mesh">Calculated mesh object</param>
        /// <param name="writer">Binary file writer</param>
        private static void SaveBinarySTLMesh(Mesh mesh, BinaryWriter writer)
        {
            var vertices = mesh.GetVertices();
            var normals = mesh.GetNormals();
            var indices = mesh.GetTriangleIndexes();

            // Check mesh arguments
            if (0 == vertices.Count || 0 != vertices.Count % 3 || vertices.Count != indices.Count)
            {
                throw new ArgumentException(Properties.Resources.InvalidMeshArgument);
            }

            char[] header = new char[80];
            writer.Write(header);

            // Write number of triangles
            int triangles = vertices.Count / 3;
            writer.Write(triangles);

            // Sequentially write the normal, 3 vertices of the triangle and attribute, for each triangle
            for (int i = 0; i < triangles; i++)
            {
                // Write normal
                var normal = normals[i * 3];
                writer.Write(normal.X);
                writer.Write(normal.Y);
                writer.Write(normal.Z);

                // Write vertices
                for (int j = 0; j < 3; j++)
                {
                    var vertex = vertices[(i * 3) + j];
                    writer.Write(vertex.X);
                    writer.Write(vertex.Y);
                    writer.Write(vertex.Z);
                }

                ushort attribute = 0;
                writer.Write(attribute);
            }
        }

        /// <summary>
        /// Save mesh in ASCII Wavefront .OBJ file
        /// </summary>
        /// <param name="mesh">Calculated mesh object</param>
        /// <param name="writer">Stream writer</param>
        private static void SaveAsciiObjMesh(Mesh mesh, StreamWriter writer)
        {
            var vertices = mesh.GetVertices();
            var normals = mesh.GetNormals();
            var indices = mesh.GetTriangleIndexes();

            // Check mesh arguments
            if (0 == vertices.Count || 0 != vertices.Count % 3 || vertices.Count != indices.Count)
            {
                throw new ArgumentException(Properties.Resources.InvalidMeshArgument);
            }

            // Write the header lines
            writer.WriteLine("#");
            writer.WriteLine("# OBJ file created by Microsoft Kinect Fusion");
            writer.WriteLine("#");

            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (int i = 0; i < vertices.Count; i++)
            {
                var vertex = vertices[i];

                string vertexString = "v " + vertex.X.ToString(CultureInfo.CurrentCulture) + " " + vertex.Y.ToString(CultureInfo.CurrentCulture) + " " + vertex.Z.ToString(CultureInfo.CurrentCulture);

                writer.WriteLine(vertexString);
            }

            // Sequentially write the 3 normals of the triangle, for each triangle
            for (int i = 0; i < normals.Count; i++)
            {
                var normal = normals[i];

                string normalString = "vn " + normal.X.ToString(CultureInfo.CurrentCulture) + " " + normal.Y.ToString(CultureInfo.CurrentCulture) + " " + normal.Z.ToString(CultureInfo.CurrentCulture);

                writer.WriteLine(normalString);
            }

            // Sequentially write the 3 vertex indices of the triangle face, for each triangle
            // Note this is typically 1-indexed in an OBJ file when using absolute referencing!
            for (int i = 0; i < vertices.Count / 3; i++)
            {
                string baseIndex0 = ((i * 3) + 1).ToString(CultureInfo.CurrentCulture);
                string baseIndex1 = ((i * 3) + 2).ToString(CultureInfo.CurrentCulture);
                string baseIndex2 = ((i * 3) + 3).ToString(CultureInfo.CurrentCulture);

                string faceString = "f " + baseIndex0 + "//" + baseIndex0 + " " + baseIndex1 + "//" + baseIndex1 + " " + baseIndex2 + "//" + baseIndex2;
                writer.WriteLine(faceString);
            }
        }

        /// <summary>
        /// Save mesh in ASCII Polygon File format Ply file
        /// </summary>
        /// <param name="mesh">Calculated mesh object</param>
        /// <param name="writer">Stream writer</param>
        private static void SaveAsciiPlyMesh(Mesh mesh, StreamWriter writer)
        {
            var vertices = mesh.GetVertices();
            var indices = mesh.GetTriangleIndexes();

            // Check mesh arguments
            if (0 == vertices.Count || 0 != vertices.Count % 3 || vertices.Count != indices.Count)
            {
                throw new ArgumentException(Properties.Resources.InvalidMeshArgument);
            }
            MeshGeometry3D newMesh = MeshProcessor.removeDuplicateVertices(mesh);

            // Write the header lines
            writer.WriteLine("ply");
            writer.WriteLine("format ascii 1.0");
            writer.WriteLine("comment created by BLI kinect fusion program");
            writer.WriteLine("element vertex " + newMesh.Positions.Count);
            writer.WriteLine("property float x");
            writer.WriteLine("property float y");
            writer.WriteLine("property float z");
            writer.WriteLine("element face " + newMesh.TriangleIndices.Count/3);
            writer.WriteLine("property list uchar int vertex_indices");
            writer.WriteLine("end_header");


            // Sequentially write the 3 vertices of the triangle, for each triangle
            for (int i = 0; i < newMesh.Positions.Count; i++)
            {
                var vertex = newMesh.Positions[i];

                string vertexString = vertex.X.ToString(CultureInfo.CurrentCulture) + " " + vertex.Y.ToString(CultureInfo.CurrentCulture) + " " + vertex.Z.ToString(CultureInfo.CurrentCulture);

                writer.WriteLine(vertexString);
            }

            //var triangleIndices = indices.GetEnumerator();
            
            // Sequentially write the 3 vertex indices of the triangle face, for each triangle
            // Note this is typically 1-indexed in an OBJ file when using absolute referencing!
            for (int i = 0; i < newMesh.TriangleIndices.Count/3; i++)
            {

                int baseIndex0 = ((i * 3));
                int baseIndex1 = ((i * 3) + 1);
                int baseIndex2 = ((i * 3) + 2);
                string faceString = "3 " + newMesh.TriangleIndices[baseIndex0] + " " + newMesh.TriangleIndices[baseIndex1] + " " + newMesh.TriangleIndices[baseIndex2];
                writer.WriteLine(faceString);
            }
        }

        private static float ClampFloat(float value, float min, float max)
        {
            if (value < min)
                return min;
            else if (value > max)
                return max;
            else
                return value;
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        public void Start()
        {
            // Start Kinect sensor chooser
            this.sensorChooser = new KinectSensorChooser();
            //this.sensorChooserUI.KinectSensorChooser = this.sensorChooser;
            this.sensorChooser.KinectChanged += this.OnKinectSensorChanged;
            this.sensorChooser.Start();

            // Start fps timer
            this.fpsTimer = new DispatcherTimer(DispatcherPriority.Send);
            this.fpsTimer.Interval = new TimeSpan(0, 0, FpsInterval);
            this.fpsTimer.Tick += this.FpsTimerTick;
            this.fpsTimer.Start();

            // Set last fps timestamp as now
            this.lastFPSTimestamp = DateTime.Now;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        public void Stop()
        {
            // Stop timer
            if (null != this.fpsTimer)
            {
                this.fpsTimer.Stop();
                this.fpsTimer.Tick -= this.FpsTimerTick;
            }

            // Unregister Kinect sensor chooser event
            if (null != this.sensorChooser)
            {
                this.sensorChooser.KinectChanged -= this.OnKinectSensorChanged;
            }

            // Stop sensor
            if (null != this.sensor)
            {
                this.sensor.Stop();
                this.sensor.DepthFrameReady -= this.OnDepthFrameReady;
            }
        }

        /// <summary>
        /// Handler function for Kinect changed event
        /// </summary>
        /// <param name="sender">Event generator</param>
        /// <param name="e">Event parameter</param>
        private void OnKinectSensorChanged(object sender, KinectChangedEventArgs e)
        {
            // Check new sensor's status
            if (this.sensor != e.NewSensor)
            {
                // Stop old sensor
                if (null != this.sensor)
                {
                    this.sensor.Stop();
                    this.sensor.DepthFrameReady -= this.OnDepthFrameReady;
                    this.sensor.ColorFrameReady -= this.OnColorFrameReady;
                }

                this.sensor = null;

                if (null != e.NewSensor && KinectStatus.Connected == e.NewSensor.Status)
                {
                    // Start new sensor
                    this.sensor = e.NewSensor;
                    this.StartDepthStream(ImageFormat);
                    this.StartColorStream(CImageFormat);
                }
            }
        }

        /// <summary>
        /// Handler for FPS timer tick
        /// </summary>
        /// <param name="sender">Object sending the event</param>
        /// <param name="e">Event arguments</param>
        private void FpsTimerTick(object sender, EventArgs e)
        {
            if (!this.savingMesh)
            {
                if (null == this.sensor)
                {
                    // Show "No ready Kinect found!" on status bar
                    this.main.statusBarText.Text = Properties.Resources.NoReadyKinect;
                }
                else
                {
                    // Calculate time span from last calculation of FPS
                    double intervalSeconds = (DateTime.Now - this.lastFPSTimestamp).TotalSeconds;

                    // Calculate and show fps on status bar
                    this.main.statusBarText.Text = string.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        Properties.Resources.Fps,
                        (double)this.processedFrameCount / intervalSeconds);
                }
            }

            // Reset frame counter
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.Now;
        }

        /// <summary>
        /// Reset FPS timer and counter
        /// </summary>
        public void ResetFps()
        {
            // Restart fps timer
            if (null != this.fpsTimer)
            {
                this.fpsTimer.Stop();
                this.fpsTimer.Start();
            }

            // Reset frame counter
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.Now;
        }

        /// <summary>
        /// Start depth stream at specific resolution
        /// </summary>
        /// <param name="format">The resolution of image in depth stream</param>
        private void StartDepthStream(DepthImageFormat format)
        {
            try
            {
                // Enable depth stream, register event handler and start
                this.sensor.DepthStream.Enable(format);
                this.sensor.DepthFrameReady += this.OnDepthFrameReady;
                this.sensor.Start();
            }
            catch (IOException ex)
            {
                // Device is in use
                this.sensor = null;
                this.ShowStatusMessage(ex.Message);

                return;
            }
            catch (InvalidOperationException ex)
            {
                // Device is not valid, not supported or hardware feature unavailable
                this.sensor = null;
                this.ShowStatusMessage(ex.Message);

                return;
            }

            // Set Near Mode by default
            try
            {
                this.sensor.DepthStream.Range = DepthRange.Near;
                this.main.NearMode = true;
            }
            catch (InvalidOperationException)
            {
                this.ShowStatusMessage(Properties.Resources.NearModeNotSupported);
            }

            // Create volume
            if (this.RecreateReconstruction())
            {
                // Show introductory message
                this.ShowStatusMessage(Properties.Resources.IntroductoryMessage);
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void OnDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            if (!processingFrames)
                return;

            // Open depth frame
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (null != depthFrame && !this.processing)
                {
                    DepthData depthData = new DepthData();

                    // Save frame timestamp
                    depthData.FrameTimestamp = depthFrame.Timestamp;

                    // Create local depth pixels buffer
                    depthData.DepthImagePixels = new DepthImagePixel[depthFrame.PixelDataLength];

                    // Copy depth pixels to local buffer
                    depthFrame.CopyDepthImagePixelDataTo(depthData.DepthImagePixels);

                    this.width = depthFrame.Width;
                    this.height = depthFrame.Height;

                    if (null == depthFromCameraBitmap)
                    {
                        depthFromCameraBitmap = new WriteableBitmap(this.width, this.height, 96.0, 96.0, PixelFormats.Bgr32, null);
                    }

                    this.distances = new Byte[this.width * this.height * 4];

                    //Each pixel has 6 floats equal to (x , y , z, norm x, norm y, norm z)
                    this.depthDataBuffer = new float[this.width * this.height * 6];
                    
                    // Use dispatcher object to invoke ProcessDepthData function to process
                    this.main.Dispatcher.BeginInvoke(
                                        DispatcherPriority.Background,
                                        (Action<DepthData>)((d) => { this.ProcessDepthData(d); }),
                                        depthData);

                    // Mark one frame will be processed
                    this.processing = true;
                }
            }
        }


        /// <summary>
        /// Process the depth input
        /// </summary>
        /// <param name="depthData">The depth data containing depth pixels and frame timestamp</param>
        private void ProcessDepthData(DepthData depthData)
        {
            try
            {
                if (null != this.volume && !this.savingMesh)
                {
                    // Ensure frame resources are ready
                    this.AllocateFrames();

                    // Check near mode
                    this.CheckNearMode();

                    // To enable playback of a .xed file through Kinect Studio and reset of the reconstruction
                    // if the .xed loops, we test for when the frame timestamp has skipped a large number. 
                    // Note: this will potentially continually reset live reconstructions on slow machines which
                    // cannot process a live frame in less time than the reset threshold. Increase the number of
                    // milliseconds if this is a problem.
                    this.CheckResetTimeStamp(depthData.FrameTimestamp);

                    // Convert depth frame to depth float frame
                    FusionDepthProcessor.DepthToDepthFloatFrame(
                                            depthData.DepthImagePixels,
                                            this.width,
                                            this.height,
                                            this.depthFloatFrame,
                                            this.minDepthClip,
                                            this.maxDepthClip,
                                            this.mirrorDepth);

                    // Render depth float frame

                    RenderDepthFloatImage(this.depthFloatFrame, ref this.depthFloatFrameDepthPixels, ref this.depthFloatFramePixels, ref this.depthFloatFrameBitmap, this.main.depthFloatImage);

                    //RenderOtherCameraImage();
                    // Align new depth float image with reconstruction
                    bool trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                        this.depthFloatFrame,
                        FusionDepthProcessor.DefaultAlignIterationCount,
                        this.deltaFromReferenceFrame,
                        out this.alignmentEnergy,
                        this.worldToCameraTransform);

                    if (!trackingSucceeded)
                    {
                        this.trackingErrorCount++;

                        // Show tracking error on status bar
                        this.ShowStatusMessage(Properties.Resources.CameraTrackingFailed);
                    }
                    else
                    {
                        // Get updated camera transform from image alignment
                        Matrix4 calculatedCameraPos = this.volume.GetCurrentWorldToCameraTransform();

                        cameraUtils.updateCamerasToWorld(calculatedCameraPos);

                        // Clear track error count
                        this.trackingErrorCount = 0;

                        this.worldToCameraTransform = calculatedCameraPos;

                        if (!this.main.PauseIntegration)
                        {
                            this.volume.IntegrateFrame(this.depthFloatFrame, this.integrationWeight, this.worldToCameraTransform);
                        }

                    }

                    if (AutoResetReconstructionWhenLost && !trackingSucceeded && this.trackingErrorCount >= MaxTrackingErrors)
                    {
                        // Bad tracking
                        this.ShowStatusMessage(Properties.Resources.ResetVolumeAuto);

                        // Automatically Clear Volume and reset tracking if tracking fails
                        this.ResetReconstruction();
                    }

                    // Calculate the point cloud of integration

                    this.volume.CalculatePointCloud(this.pointCloudFrame, this.worldToCameraTransform);

                    // Map X axis to blue channel, Y axis to green channel and Z axiz to red channel,
                    // normalizing each to the range [0, 1].
                    Matrix4 worldToBGRTransform = new Matrix4();
                    worldToBGRTransform.M11 = this.voxelsPerMeter / this.voxelsX;
                    worldToBGRTransform.M22 = this.voxelsPerMeter / this.voxelsY;
                    worldToBGRTransform.M33 = this.voxelsPerMeter / this.voxelsZ;
                    worldToBGRTransform.M41 = 0.5f;
                    worldToBGRTransform.M42 = 0.5f;
                    worldToBGRTransform.M44 = 1.0f;

                    // Shade point cloud frame for rendering
                    FusionDepthProcessor.ShadePointCloud(this.pointCloudFrame, this.worldToCameraTransform, worldToBGRTransform, this.shadedSurfaceFrame, this.shadedSurfaceNormalsFrame);

                    // Render shaded surface frame or shaded surface normals frame
                    RenderColorImage(this.displayNormals ? this.shadedSurfaceNormalsFrame : this.shadedSurfaceFrame, ref this.shadedSurfaceFramePixels, ref this.shadedSurfaceFrameBitmap, this.main.shadedSurfaceImage);

                    if (trackingSucceeded)
                    {
                        // Increase processed frame counter
                        this.processedFrameCount++;
                    }
                }
            }
            catch (InvalidOperationException ex)
            {
                this.ShowStatusMessage(ex.Message);
            }
            finally
            {
                this.processing = false;
            }
        }


        /// <summary>
        /// Start color stream at specific resolution
        /// </summary>
        /// <param name="format">The resolution of image in color stream</param>
        private void StartColorStream(ColorImageFormat format)
        {
            try
            {
                // Enable depth stream, register event handler and start
                this.sensor.ColorStream.Enable(format);
                this.sensor.ColorFrameReady += this.OnColorFrameReady;
                if (!this.sensor.IsRunning) this.sensor.Start();
            }
            catch (IOException ex)
            {
                // Device is in use
                this.sensor = null;
                this.ShowStatusMessage(ex.Message);

                return;
            }
            catch (InvalidOperationException ex)
            {
                // Device is not valid, not supported or hardware feature unavailable
                this.sensor = null;
                this.ShowStatusMessage(ex.Message);

                return;
            }
        }

        /// <summary>
        /// Event handler for color frames. 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void OnColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            if (!processingFrames)
                return;

            // Open depth frame
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (null != colorFrame && !this.processing)
                {
                    if (null == colorData || colorData.Length != colorFrame.PixelDataLength)
                    {
                        colorData = new byte[colorFrame.PixelDataLength];
                        colorFrameBitmap = null;
                    }

                    // Copy depth pixels to local buffer
                    colorFrame.CopyPixelDataTo(colorData);
                    if (null == colorFrameBitmap)
                    {
                        if (colorFrame.BytesPerPixel == 4)
                        {
                            //colorFrameBitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                            colorFrameBitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96.0, 96.0, PixelFormats.Bgr24, null);
                        }
                        else
                        {
                            
                            colorFrameBitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96.0, 96.0, PixelFormats.Gray16, null);
                        }
                        this.main.colorImage.Source = colorFrameBitmap;
                    }
                    byte[] correctedData = new byte[colorData.Length];
                    
                    for (int i = 0; i < ColorImageHeight; i++)
                    {
                        if (colorFrame.BytesPerPixel == 4)
                        {
                            for (int j = 0; j < ColorImageWidth; j++)
                            {

                                correctedData[(i * ColorImageWidth + j) * 3] = colorData[((i + 1) * ColorImageWidth - j) * 4 - 4];
                                correctedData[(i * ColorImageWidth + j) * 3 + 1] = colorData[((i + 1) * ColorImageWidth - j) * 4 - 3];
                                correctedData[(i * ColorImageWidth + j) * 3 + 2] = colorData[((i + 1) * ColorImageWidth - j) * 4 - 2];

                            }
                        }
                        else {
                            for (int j = 0; j < ColorImageWidth; j++)
                            {
                                correctedData[(i * ColorImageWidth + j) * 2] = colorData[((i + 1) * ColorImageWidth - j) * 2 - 2];
                                correctedData[(i * ColorImageWidth + j) * 2 + 1] = colorData[((i + 1) * ColorImageWidth - j) * 2 - 1];
                            }
                        
                        }
                    }

                    colorFrameBitmap.WritePixels(new Int32Rect(0, 0, colorFrame.Width, colorFrame.Height), correctedData, colorFrame.Width * 3, 0);

                    VideoProcessor.addFrame(colorFrameBitmap);

                    if (this.currentlyUsingIR != this.main.useIR) {
                        ChangeColorImageFormat();
                    }
                }
            }
        }

        /// <summary>
        /// Allocate the frame buffers used in the process
        /// </summary>
        private void AllocateFrames()
        {
            // Allocate depth float frame
            if (null == this.depthFloatFrame || this.width != this.depthFloatFrame.Width || this.height != this.depthFloatFrame.Height)
            {
                this.depthFloatFrame = new FusionFloatImageFrame(this.width, this.height);
            }

            // Allocate delta from reference frame
            if (null == this.deltaFromReferenceFrame || this.width != this.deltaFromReferenceFrame.Width || this.height != this.deltaFromReferenceFrame.Height)
            {
                this.deltaFromReferenceFrame = new FusionFloatImageFrame(this.width, this.height);
            }

            // Allocate point cloud frame
            if (null == this.pointCloudFrame || this.width != this.pointCloudFrame.Width || this.height != this.pointCloudFrame.Height)
            {
                this.pointCloudFrame = new FusionPointCloudImageFrame(this.width, this.height);
            }

            if (null == this.fpcif || this.width != this.fpcif.Width || this.height != this.fpcif.Height)
            {
                this.fpcif = new FusionPointCloudImageFrame(this.width, this.height);
                //this.fpcif = new FusionPointCloudImageFrame(this.width, this.height, camParam);
            }

            // Allocate shaded surface frame
            if (null == this.shadedSurfaceFrame || this.width != this.shadedSurfaceFrame.Width || this.height != this.shadedSurfaceFrame.Height)
            {
                this.shadedSurfaceFrame = new FusionColorImageFrame(this.width, this.height);
            }

            // Allocate shaded surface normals frame
            if (null == this.shadedSurfaceNormalsFrame || this.width != this.shadedSurfaceNormalsFrame.Width || this.height != this.shadedSurfaceNormalsFrame.Height)
            {
                this.shadedSurfaceNormalsFrame = new FusionColorImageFrame(this.width, this.height);
            }
        }

        /// <summary>
        /// Check and enable or disable near mode
        /// </summary>
        private void CheckNearMode()
        {
            if (null != this.sensor && this.nearMode != (this.sensor.DepthStream.Range != DepthRange.Default))
            {
                this.sensor.DepthStream.Range = this.nearMode ? DepthRange.Near : DepthRange.Default;
            }
        }

        /// <summary>
        /// Check if the gap between 2 frames has reached reset time threshold. If yes, reset the reconstruction
        /// </summary>
        private void CheckResetTimeStamp(long frameTimestamp)
        {
            if (0 != this.lastFrameTimestamp)
            {
                long timeThreshold = (ReconstructionProcessor.Amp == ProcessorType) ? ResetOnTimeStampSkippedMillisecondsGPU : ResetOnTimeStampSkippedMillisecondsCPU;

                // Calculate skipped milliseconds between 2 frames
                long skippedMilliseconds = Math.Abs(frameTimestamp - this.lastFrameTimestamp);

                if (skippedMilliseconds >= timeThreshold)
                {
                    this.ShowStatusMessage(Properties.Resources.ResetVolume);
                    this.ResetReconstruction();
                }
            }

            // Set timestamp of last frame
            this.lastFrameTimestamp = frameTimestamp;
        }

        /// <summary>
        /// Reset reconstruction object to initial state
        /// </summary>
        public void ResetReconstruction()
        {
            if (null == this.sensor)
            {
                return;
            }

            // Reset tracking error counter
            this.trackingErrorCount = 0;

            // Set the world-view transform to identity, so the world origin is the initial camera location.
            this.worldToCameraTransform = Matrix4.Identity;

            // Reset volume
            if (null != this.volume)
            {
                try
                {
                    // Translate the reconstruction volume location away from the world origin by an amount equal
                    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
                    // If set false, the default world origin is set to the center of the front face of the 
                    // volume, which has the effect of locating the volume directly in front of the initial camera
                    // position with the +Z axis into the volume along the initial camera direction of view.
                    if (this.translateResetPoseByMinDepthThreshold)
                    {
                        Matrix4 worldToVolumeTransform = this.defaultWorldToVolumeTransform;

                        // Translate the volume in the Z axis by the minDepthThreshold distance
                        float minDist = (this.minDepthClip < this.maxDepthClip) ? this.minDepthClip : this.maxDepthClip;
                        worldToVolumeTransform.M43 -= minDist * this.voxelsPerMeter;

                        this.volume.ResetReconstruction(this.worldToCameraTransform, worldToVolumeTransform);
                    }
                    else
                    {
                        this.volume.ResetReconstruction(this.worldToCameraTransform);
                    }

                    if (this.main.PauseIntegration)
                    {
                        this.main.PauseIntegration = false;
                    }
                }
                catch (InvalidOperationException)
                {
                    this.ShowStatusMessage(Properties.Resources.ResetFailed);
                }
            }

            // Reset fps counter
            this.ResetFps();
        }

        /// <summary>
        /// Re-create the reconstruction object
        /// </summary>
        /// <returns>Indicate success or failure</returns>
        public bool RecreateReconstruction()
        {
            // Check if sensor has been initialized
            if (null == this.sensor)
            {
                return false;
            }

            if (null != this.volume)
            {
                this.volume.Dispose();
            }

            try
            {
                // The zero-based GPU index to choose for reconstruction processing if the 
                // ReconstructionProcessor AMP options are selected.
                // Here we automatically choose a device to use for processing by passing -1, 
                int deviceIndex = -1;

                ReconstructionParameters volParam = new ReconstructionParameters(this.voxelsPerMeter, this.voxelsX, this.voxelsY, this.voxelsZ);

                // Set the world-view transform to identity, so the world origin is the initial camera location.
                this.worldToCameraTransform = Matrix4.Identity;

                this.volume = Reconstruction.FusionCreateReconstruction(volParam, ProcessorType, deviceIndex, this.worldToCameraTransform);

                this.defaultWorldToVolumeTransform = this.volume.GetCurrentWorldToVolumeTransform();

                if (this.translateResetPoseByMinDepthThreshold)
                {
                    this.ResetReconstruction();
                }

                // Reset "Pause Integration"
                if (this.main.PauseIntegration)
                {
                    this.main.PauseIntegration = false;
                }

                return true;
            }
            catch (ArgumentException)
            {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.VolumeResolution);
            }
            catch (InvalidOperationException ex)
            {
                this.volume = null;
                this.ShowStatusMessage(ex.Message);
            }
            catch (DllNotFoundException)
            {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.MissingPrerequisite);
            }
            catch (OutOfMemoryException)
            {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.OutOfMemory);
            }

            return false;
        }

        /// <summary>
        /// Handler for click event from "Save Color" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        public void SaveColorImage()
        {
            
            FusionPointCloudImageFrame fpcif = new FusionPointCloudImageFrame(640, 480);

            if (null == this.sensor || null == colorData)
            {
                return;
            }
            imageNumber++;

            FileStream pngImage = new FileStream(imageNumber + ".png", FileMode.Create);
            PngBitmapEncoder encoder = new PngBitmapEncoder();
            encoder.Interlace = PngInterlaceOption.Off;
            encoder.Frames.Add(BitmapFrame.Create(this.colorFrameBitmap));
            encoder.Save(pngImage);

            FusionPointCloudImageFrame tempFPCIF = new FusionPointCloudImageFrame(640, 480, new CameraParameters(0.8125f, 1.0833f, .5f, .5f));

            this.volume.CalculatePointCloud(tempFPCIF, this.cameraUtils.OtherCameraToWorld);
            tempFPCIF.CopyPixelDataTo(this.depthDataBuffer);
            float rangeAdjustment = ((this.maxDepthClip - this.minDepthClip) > 0) ? (255 - 20) / (this.maxDepthClip - this.minDepthClip) : 1;

            //Min should be 20 so that is is easily distinguished from background, but the range from min to max should then span 21 - 255
            for (int i = 0; i < tempFPCIF.PixelDataLength; i++)
            {
                float correctedValue = depthDataBuffer[6 * i + 2];
                byte value = (byte)((correctedValue > 0) ? (correctedValue - this.minDepthClip) * rangeAdjustment + 20 : 0);
                this.distances[i * 4] = value;
                this.distances[i * 4 + 1] = value;
                this.distances[i * 4 + 2] = value;

            }
            depthFromCameraBitmap.WritePixels(new Int32Rect(0, 0, this.width, this.height), distances, this.width * sizeof(int), 0);

            //Simple depth
            pngImage = new FileStream(imageNumber + "_depth_simple.png", FileMode.Create);
            encoder = new PngBitmapEncoder();
            encoder.Interlace = PngInterlaceOption.Off;
            encoder.Frames.Add(BitmapFrame.Create(this.depthFromCameraBitmap));
            encoder.Save(pngImage);


            Matrix4 location = this.volume.GetCurrentWorldToCameraTransform();

            this.cameraUtils.SaveMatrixToFile(location, imageNumber);
        }
        public void ChangeColorImageFormat() { 
            if(this.main.useIR!=this.currentlyUsingIR){
                this.sensor.ColorStream.Disable();
                if(this.main.useIR){
                    this.sensor.ColorStream.Enable(IRImageFormat);
                }else{
                    this.sensor.ColorStream.Enable(CImageFormat);
                }
                currentlyUsingIR = this.main.useIR;
            }

        }

        /// <summary>
        /// Handler for click event from "Create Mesh" button
        /// </summary>
        public void CreateMesh()
        {
            if (null == this.volume)
            {
                this.ShowStatusMessage(Properties.Resources.MeshNullVolume);
                return;
            }

            this.savingMesh = true;

            // Mark the start time of saving mesh
            DateTime begining = DateTime.Now;

            try
            {
                this.ShowStatusMessage(Properties.Resources.SavingMesh);

                Mesh mesh = this.volume.CalculateMesh(1);

                SaveFileDialog dialog = new SaveFileDialog();

                if (true == this.main.stlFormat.IsChecked)
                {
                    dialog.FileName = "MeshedReconstruction.stl";
                    dialog.Filter = "STL Mesh Files|*.stl|All Files|*.*";
                }
                else if(true == this.main.objFormat.IsChecked)
                {
                    dialog.FileName = "MeshedReconstruction.obj";
                    dialog.Filter = "OBJ Mesh Files|*.obj|All Files|*.*";
                }
                else
                {
                    dialog.FileName = "MeshedReconstruction.ply";
                    dialog.Filter = "PLY Mesh Files|*.ply|All Files|*.*";
                }

                if (true == dialog.ShowDialog())
                {
                    if (true == this.main.stlFormat.IsChecked)
                    {
                        using (BinaryWriter writer = new BinaryWriter(dialog.OpenFile()))
                        {
                            SaveBinarySTLMesh(mesh, writer);
                        }
                    }
                    else if (true == this.main.objFormat.IsChecked)
                    {
                        using (StreamWriter writer = new StreamWriter(dialog.FileName))
                        {
                            SaveAsciiObjMesh(mesh, writer);
                        }
                    }
                    else {
                        using (StreamWriter writer = new StreamWriter(dialog.FileName))
                        {
                            SaveAsciiPlyMesh(mesh, writer);
                        }
                    
                    }

                    this.ShowStatusMessage(Properties.Resources.MeshSaved);
                }
                else
                {
                    this.ShowStatusMessage(Properties.Resources.MeshSaveCanceled);
                }
            }
            catch (ArgumentException)
            {
                this.ShowStatusMessage(Properties.Resources.ErrorSaveMesh);
            }
            catch (InvalidOperationException)
            {
                this.ShowStatusMessage(Properties.Resources.ErrorSaveMesh);
            }
            catch (IOException)
            {
                this.ShowStatusMessage(Properties.Resources.ErrorSaveMesh);
            }

            // Update timestamp of last frame to avoid auto reset reconstruction
            this.lastFrameTimestamp += (long)(DateTime.Now - begining).TotalMilliseconds;

            this.savingMesh = false;
        }




        /// <summary>
        /// Show exception info on status bar
        /// </summary>
        /// <param name="message">Message to show on status bar</param>
        private void ShowStatusMessage(string message)
        {
            this.main.Dispatcher.BeginInvoke((Action)(() =>
            {
                this.ResetFps();
                this.main.statusBarText.Text = message;
            }));
        }

        public void updateDisplayNormals(bool value) {
            this.displayNormals = value;
            this.main.PropertyChangedEvent("DisplayNormals");
        }
        public void updateNearMode(bool value) {
            this.nearMode = value;
            this.main.PropertyChangedEvent("NearMode");
        }
        public void updatePauseIntegration(bool value) {
            this.pauseIntegration = value;
            this.main.PropertyChangedEvent("PauseIntegration");
        }
        public void updateMirrorDepth(bool value)
        {
            this.mirrorDepth = value;
            this.main.PropertyChangedEvent("MirrorDepth");
            this.ResetReconstruction();
        }

        /// <summary>
        /// Binding property to min clip depth slider
        /// </summary>
        public void updateMinDepthClip(float value)
        {
            this.minDepthClip = value;
            this.main.PropertyChangedEvent("MinDepthClip");
        }

        /// <summary>
        /// Binding property to max clip depth slider
        /// </summary>
        public void updateMaxDepthClip(float value)
        {
            this.maxDepthClip = (float)value;
            this.main.PropertyChangedEvent("MaxDepthClip");
        }

        /// <summary>
        /// Binding property to integration weight slider
        /// </summary>
        public void updateIntegrationWeight(double value)
        {
            this.integrationWeight = (short)(value + 0.5);
            this.main.PropertyChangedEvent("IntegrationWeight");
        }

        /// <summary>
        /// Binding property to voxels per meter slider
        /// </summary>
        public void updateVoxelsPerMeter(double value)
        {
            this.voxelsPerMeter = (float)value;
            this.main.PropertyChangedEvent("VoxelsPerMeter");
        }

        /// <summary>
        /// Binding property to X-axis volume resolution slider
        /// </summary>
        public void updateVoxelsX(double value)
        {
            this.voxelsX = (int)(value + 0.5);
            this.main.PropertyChangedEvent("VoxelsX");
        }

        /// <summary>
        /// Binding property to Y-axis volume resolution slider
        /// </summary>
        public void updateVoxelsY(double value)
        {
            this.voxelsY = (int)(value + 0.5);
            this.main.PropertyChangedEvent("VoxelsY");
        }

        /// <summary>
        /// Binding property to Z-axis volume resolution slider
        /// </summary>
        public void updateVoxelsZ(double value)
        {
            this.voxelsZ = (int)(value + 0.5);
            this.main.PropertyChangedEvent("VoxelsZ");
        }
        public void SetCorrectionMatrix() {
            this.cameraUtils.SetCorrectionMatrix();
        }
    }
}