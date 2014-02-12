//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace BLIKinect
{
    using System;
    using System.ComponentModel;
    using System.Globalization;
    using System.Windows;
    using System.Windows.Data;
    using System.Windows.Threading;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        private KinectReconstruction reconProcessor;
        public Boolean useIR = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            this.reconProcessor = new KinectReconstruction(this);
            this.InitializeComponent();
        }



        #region Binding properties

        /// <summary>
        /// Binding property to check box "Display Surface Normals"
        /// </summary>
        public bool DisplayNormals
        {
            get{return this.reconProcessor.displayNormals;}
            set{this.reconProcessor.updateDisplayNormals(value);}
        }

        /// <summary>
        /// Binding property to check box "Near Mode"
        /// </summary>
        public bool NearMode
        {
            get{return this.reconProcessor.nearMode;}
            set{this.reconProcessor.updateNearMode(value);}
        }

        /// <summary>
        /// Binding property to check box "Pause Integration"
        /// </summary>
        public bool PauseIntegration
        {
            get{return this.reconProcessor.pauseIntegration;}
            set{this.reconProcessor.updatePauseIntegration(value);}
        }

        /// <summary>
        /// Binding property to check box "Mirror Depth"
        /// </summary>
        public bool MirrorDepth
        {
            get{return this.reconProcessor.mirrorDepth;}
            set{this.reconProcessor.updateMirrorDepth(value);}
        }

        /// <summary>
        /// Binding property to min clip depth slider
        /// </summary>
        public double MinDepthClip
        {
            get{return (double)this.reconProcessor.minDepthClip;}
            set{this.reconProcessor.updateMinDepthClip((float)value);}
        }

        /// <summary>
        /// Binding property to max clip depth slider
        /// </summary>
        public double MaxDepthClip
        {
            get{return (double)this.reconProcessor.maxDepthClip;}
            set{this.reconProcessor.updateMaxDepthClip((float)value);}
        }

        /// <summary>
        /// Binding property to integration weight slider
        /// </summary>
        public double IntegrationWeight
        {
            get{return (double)this.reconProcessor.integrationWeight;}
            set{this.reconProcessor.updateIntegrationWeight(value);}
        }

        /// <summary>
        /// Binding property to voxels per meter slider
        /// </summary>
        public double VoxelsPerMeter
        {
            get{return (double)this.reconProcessor.voxelsPerMeter;}
            set{this.reconProcessor.updateVoxelsPerMeter(value);}
        }

        /// <summary>
        /// Binding property to X-axis volume resolution slider
        /// </summary>
        public double VoxelsX
        {
            get{return (double)this.reconProcessor.voxelsX;}
            set{this.reconProcessor.updateVoxelsX(value);}
        }

        /// <summary>
        /// Binding property to Y-axis volume resolution slider
        /// </summary>
        public double VoxelsY
        {
            get { return (double)this.reconProcessor.voxelsY; }
            set { this.reconProcessor.updateVoxelsY(value); }
        }

        /// <summary>
        /// Binding property to Z-axis volume resolution slider
        /// </summary>
        public double VoxelsZ
        {
            get { return (double)this.reconProcessor.voxelsZ; }
            set { this.reconProcessor.updateVoxelsZ(value); }
        }

        #endregion

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            this.reconProcessor.Start();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">Object sending the event</param>
        /// <param name="e">Event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.reconProcessor.Stop();
        }

        /// <summary>
        /// Handler for click event from "Reset Reconstruction" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        private void ResetReconstructionButtonClick(object sender, RoutedEventArgs e)
        {
            if (null == this.reconProcessor.sensor)
            {
                return;
            }

            this.reconProcessor.ResetReconstruction();
            this.ShowStatusMessage(Properties.Resources.ResetVolume);
        }

        /// <summary>
        /// Handler for click event from "Save Color" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        private void SaveColorButtonClick(object sender, RoutedEventArgs e)
        {
            this.reconProcessor.SaveColorImage();
            this.ShowStatusMessage(Properties.Resources.SaveColor);
        }

        /// <summary>
        /// Handler for click event from "Save Color" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        private void SetCorrectionMatrix(object sender, RoutedEventArgs e)
        {
            this.reconProcessor.processingFrames = false;
            this.reconProcessor.SetCorrectionMatrix();
            this.ShowStatusMessage("Updated Correction Matrix");
            this.reconProcessor.processingFrames = true;
        }

        /// <summary>
        /// Handler for click event from "Create Mesh" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        private void CreateMeshButtonClick(object sender, RoutedEventArgs e)
        {
            this.reconProcessor.CreateMesh();
        }

        /// <summary>
        /// Handler for volume setting changing event
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event argument</param>
        private void VolumeSettingsChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            this.reconProcessor.RecreateReconstruction();
        }

        /// <summary>
        /// Show exception info on status bar
        /// </summary>
        /// <param name="message">Message to show on status bar</param>
        private void ShowStatusMessage(string message)
        {
            this.Dispatcher.BeginInvoke((Action)(() =>
            {
                this.reconProcessor.ResetFps();
                this.statusBarText.Text = message;
            }));
        }

        public void PropertyChangedEvent(String eventArgs) {
            if (null != this.PropertyChanged)
            {
                this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs(eventArgs));
            }
        }

        private void ColorFormatSelect(object sender, RoutedEventArgs e)
        {
            RadioButton source = sender as RadioButton;
            if (source.Content.ToString()=="RGB")
            {
                this.useIR = false;
            }
            else {
                this.useIR = true;
            }
        }

        private void RecordToggle(object sender, RoutedEventArgs e)
        {
            Button btn = (Button)sender;
            if (VideoProcessor.isRecording())
            {
                VideoProcessor.saveVideo();
                btn.Content = "Record Video";
            }
            else {
                VideoProcessor.startRecording();
                btn.Content = "Stop Recording";
            }
        }
    }

    /// <summary>
    /// Convert depth to UI text
    /// </summary>
    public class DepthToTextConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return ((double)value).ToString("0.00", CultureInfo.CurrentCulture) + "m";
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

}
