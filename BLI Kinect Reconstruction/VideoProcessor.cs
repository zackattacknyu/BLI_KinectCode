using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using AviFile;
using System.Drawing;
using System.Windows.Media.Imaging;
using System.IO;

namespace BLIKinect
{
    static class VideoProcessor
    {
        public static int count = 1;
        public static Stopwatch sw = new Stopwatch();
        public static List<Bitmap> images = new List<Bitmap>();

        public static void startRecording(){
            sw.Start();
        }
        public static void stopRecording() {
            sw.Stop();
        }
        public static Boolean isRecording() {
            return sw.IsRunning;
        }
        public static void addFrame(WriteableBitmap inputframe) {
            if (sw.IsRunning) { 
                images.Add(BitmapFromWriteableBitmap(inputframe));
            }
        }
        public static void saveVideo() {
            //create a new AVI file


            if (sw.IsRunning)
                sw.Stop();
            long totalMilliseconds = sw.ElapsedMilliseconds;

            double FrameRate = 30;//Default frames per second if it cannot be calculated
            if (totalMilliseconds > 0) {
                FrameRate = (images.Count * 1000) / totalMilliseconds;
            }
            AviManager aviManager = new AviManager(@"Recording_"+count+".avi", false);
            //add a new video stream and one frame to the new file
            VideoStream aviStream = aviManager.AddVideoStream(false, FrameRate, images[0]);

            for (int n = 1; n < images.Count; n++)
            {
                aviStream.AddFrame(images[n]);
            }
            aviManager.Close();
            images.Clear();
            sw.Reset();
            count++;
        }
        private static System.Drawing.Bitmap BitmapFromWriteableBitmap(WriteableBitmap writeBmp)
        {
            System.Drawing.Bitmap bmp;
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create((BitmapSource)writeBmp));
                enc.Save(outStream);
                bmp = new System.Drawing.Bitmap(outStream);
            }
            return bmp;
        }

    }
}
