

namespace BLIKinect
{
    using Microsoft.Kinect;
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;
    using System.IO;
    using System.Windows.Forms;
    using System.Windows.Data;
    using System.Diagnostics;

    class CameraUtilities
    {
        private volatile Boolean correctionSet = false;


        private Matrix4 correctionMatrix;

        private Matrix4 otherCameraWorldToCamera;

        private Matrix4 otherCameraCameraToWorld;

        private Matrix4 worldToCamera;

        private Matrix4 cameraToWorld;

        //translation associated with moving from depth camera to color camera. This can be adjusted if a better calibration is determined.
        private const float xOffset = 0.0220f;
        private const float yOffset = -0.00213f;
        private const float zOffset = -0.00218f;

        /// <summary>
        /// Calculates the camera to world from a given world to Camera. It is important to note that microsoft uses column row format whereas matlab uses row column.
        /// </summary>
        /// <param name="worldToCamera"></param>
        /// <returns></returns>
        public Matrix4 calculateCameraToWorld(Matrix4 worldToCamera) {
            Matrix4 location = worldToCamera;

            //Calculate the transpose for the inverse of the world to camera matrix
            float invX = -(location.M11 * location.M41 + location.M12 * location.M42 + location.M13 * location.M43);
            float invY = -(location.M21 * location.M41 + location.M22 * location.M42 + location.M23 * location.M43);
            float invZ = -(location.M31 * location.M41 + location.M32 * location.M42 + location.M33 * location.M43);

            //We don't need to changed the positions from M11:M13:M31:M33 because Microsoft uses column:row format and I am using 
            //row:column so they are essential transposed already which the the inverse of the rotation matrix
            location.M14 = invX;
            location.M24 = invY;
            location.M34 = invZ;
            location.M41 = 0;
            location.M42 = 0;
            location.M43 = 0;

            return location;
        }

        /// <summary>
        /// This method is used to correct the translation of the color camera given the know location and rotation for the depth camera.
        /// </summary>
        /// <param name="depthCameraToWorld">This is the depth cameras camera to world matrix.</param>
        /// <returns>The color camera's camera to world matrix</returns>
        public Matrix4 correctForDepthToColor(Matrix4 depthCameraToWorld) {
            Matrix4 colorCameraToWorld = depthCameraToWorld;

            float xCorrection = xOffset * depthCameraToWorld.M11 + yOffset * depthCameraToWorld.M12 + zOffset * depthCameraToWorld.M13;
            float yCorrection = xOffset * depthCameraToWorld.M21 + yOffset * depthCameraToWorld.M22 + zOffset * depthCameraToWorld.M23;
            float zCorrection = xOffset * depthCameraToWorld.M31 + yOffset * depthCameraToWorld.M32 + zOffset * depthCameraToWorld.M33;

            colorCameraToWorld.M14 += xCorrection;
            colorCameraToWorld.M24 += yCorrection;
            colorCameraToWorld.M34 += zCorrection;

            return colorCameraToWorld;
        }

        /// <summary>
        /// This method take a current world to camera matrix and determines the camera to world matrices for both the color camera and the depth camera.
        /// </summary>
        /// <param name="worldToDepthCamera">Pass in the current world to depth camera matrix. This is directly provided by the kinect sensor.</param>
        public void updateCamerasToWorld(Matrix4 worldToDepthCamera) {
            this.worldToCamera = worldToDepthCamera;

            Matrix4 depthCameraToWorld = calculateCameraToWorld(worldToCamera);

            this.cameraToWorld = correctForDepthToColor(depthCameraToWorld);

            //If a correction matrix has been selected, use it to generate the files needed for the reconstruction
            if (correctionSet)
            {
                Matrix4 correctionAfterDToC = correctForDepthToColor(correctionMatrix);
                this.otherCameraCameraToWorld = matMult(depthCameraToWorld,correctionAfterDToC);
            }
        }


        /// <summary>
        /// Handler for click event from "Save Color" button
        /// </summary>
        /// <param name="sender">Event sender</param>
        /// <param name="e">Event arguments</param>
        public void SetCorrectionMatrix()
        {
            float[,] values = new float[4, 4];
            String[] rows = new String[4];
            
            OpenFileDialog ofDlg = new OpenFileDialog();


            if (ofDlg.ShowDialog() == System.Windows.Forms.DialogResult.OK)
            {

                
                using (TextReader tr = File.OpenText(ofDlg.FileName))
                {
                    for (int i = 0; i < 4; i++)
                    {
                        rows[i] = tr.ReadLine();
                    }
                }
            }else{return;}

            for (int i = 0; i < 4; i++)
            {
                String[] split = rows[i].Split(new char[] { ',' });
                for (int j = 0; j < 4; j++)
                {
                    values[i, j] = float.Parse(split[j]);
                }
            }

            correctionMatrix.M11 = values[0, 0];
            correctionMatrix.M12 = values[0, 1];
            correctionMatrix.M13 = values[0, 2];
            correctionMatrix.M14 = values[0, 3];
            correctionMatrix.M21 = values[1, 0];
            correctionMatrix.M22 = values[1, 1];
            correctionMatrix.M23 = values[1, 2];
            correctionMatrix.M24 = values[1, 3];
            correctionMatrix.M31 = values[2, 0];
            correctionMatrix.M32 = values[2, 1];
            correctionMatrix.M33 = values[2, 2];
            correctionMatrix.M34 = values[2, 3];
            correctionMatrix.M41 = values[3, 0];
            correctionMatrix.M42 = values[3, 1];
            correctionMatrix.M43 = values[3, 2];
            correctionMatrix.M44 = values[3, 3];

            this.correctionSet = true;
        }

        /// <summary>
        /// This just multiplies to matrices. Since order matters for matrix multiplications you must pass the matrices in order left and right.
        /// </summary>
        /// <param name="left">The left matrix in the multiplication</param>
        /// <param name="right">The right matrix in the muliplication</param>
        /// <returns>The product of two 4x4 matrices</returns>
        public Matrix4 matMult(Matrix4 left, Matrix4 right)
        {
            Matrix4 result = new Matrix4();

            float[][] rows = new float[4][];
            rows[0] = new float[] { left.M11, left.M12, left.M13, left.M14 };
            rows[1] = new float[] { left.M21, left.M22, left.M23, left.M24 };
            rows[2] = new float[] { left.M31, left.M32, left.M33, left.M34 };
            rows[3] = new float[] { left.M41, left.M42, left.M43, left.M44 };

            float[][] cols = new float[4][];
            cols[0] = new float[] { right.M11, right.M21, right.M31, right.M41 };
            cols[1] = new float[] { right.M12, right.M22, right.M32, right.M42 };
            cols[2] = new float[] { right.M13, right.M23, right.M33, right.M43 };
            cols[3] = new float[] { right.M14, right.M24, right.M34, right.M44 };

            result.M11 = vectMult(rows[0], cols[0]);
            result.M12 = vectMult(rows[0], cols[1]);
            result.M13 = vectMult(rows[0], cols[2]);
            result.M14 = vectMult(rows[0], cols[3]);
            result.M21 = vectMult(rows[1], cols[0]);
            result.M22 = vectMult(rows[1], cols[1]);
            result.M23 = vectMult(rows[1], cols[2]);
            result.M24 = vectMult(rows[1], cols[3]);
            result.M31 = vectMult(rows[2], cols[0]);
            result.M32 = vectMult(rows[2], cols[1]);
            result.M33 = vectMult(rows[2], cols[2]);
            result.M34 = vectMult(rows[2], cols[3]);
            result.M41 = vectMult(rows[3], cols[0]);
            result.M42 = vectMult(rows[3], cols[1]);
            result.M43 = vectMult(rows[3], cols[2]);
            result.M44 = vectMult(rows[3], cols[3]);

            return result;
        }

        /// <summary>
        /// This calculates the dot product of two vectors. It is used in the matrix multiplication. The vectors must be the came length.
        /// </summary>
        /// <param name="a">First vector</param>
        /// <param name="b">Second Vector</param>
        /// <returns>Returns the scalar associated with the dot product of the two input vectors.</returns>
        public float vectMult(float[] a, float[] b)
        {
            float value = 0;
            for (int i = 0; i < a.Length; i++)
            {
                value += a[i] * b[i];
            }
            return value;

        }
        /// <summary>
        /// Helper method for writing the matrix to a text file. The file should be named according to the image that was used to create it.
        /// </summary>
        /// <param name="worldToCamera">This is whatever world to camera matrix you want to write to file.</param>
        /// <param name="imageNumber">The image number is used to name the file so we know which image the matrix corresponds to.</param>
        public void SaveMatrixToFile(Matrix4 worldToCamera, int imageNumber)
        {
            updateCamerasToWorld(worldToCamera);
            //The 525 480 and 640 are the focal length in pixels along with the height and width. The actual values don't matter, but the ratio does. These are used for texturing.
            writeMatrixToReconstructFile(this.cameraToWorld, imageNumber + ".txt", "525 480 640");

            //If a correction matrix has been selected, use it to generate the files needed for the reconstruction
            if (correctionSet)
            {
                writeMatrixToReconstructFile(this.otherCameraCameraToWorld, "matrix" + imageNumber + ".txt", "525 480 640");//Depending on the third party camera, you may have to change these parameters.
                writeMatrixToFile(this.worldToCamera, "WorldToCamera_" + imageNumber + ".txt");
            }
        }
        /// <summary>
        /// Actual
        /// </summary>
        /// <param name="matrix"></param>
        /// <param name="filename"></param>
        /// <param name="cameraIntrinsics"></param>
        private void writeMatrixToReconstructFile(Matrix4 matrix,String filename, String cameraIntrinsics) {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("TVector");
            sb.AppendLine(matrix.M14.ToString());
            sb.AppendLine(matrix.M24.ToString());
            sb.AppendLine(matrix.M34.ToString());
            sb.AppendLine();
            sb.AppendLine("RMatrix");
            sb.AppendLine(matrix.M11.ToString() + "\t" + matrix.M12.ToString() + "\t" + matrix.M13.ToString());
            sb.AppendLine(matrix.M21.ToString() + "\t" + matrix.M22.ToString() + "\t" + matrix.M23.ToString());
            sb.AppendLine(matrix.M31.ToString() + "\t" + matrix.M32.ToString() + "\t" + matrix.M33.ToString());
            sb.AppendLine();
            sb.AppendLine("Camera Intrinsics: focal height width");
            sb.AppendLine(cameraIntrinsics);
            sb.AppendLine();

            using (StreamWriter outfile = new StreamWriter(filename, false))
            {
                outfile.Write(sb.ToString());
            }
        }
        /// <summary>
        /// Getter for checking the status of the correction matrix. 
        /// </summary>
        public Boolean CorrectionSet {
            get{return this.correctionSet;}
        }
        /// <summary>
        /// This writes the matrix to file in a way that can be easily read in by matlab's 
        /// </summary>
        /// <param name="matrix">This is the matrix to write to file</param>
        /// <param name="filename">This is what to name the file</param>
        private void writeMatrixToFile(Matrix4 matrix,String filename) {
            StringBuilder sb = new StringBuilder();
            sb = new StringBuilder();
            sb.AppendLine(matrix.M11.ToString() + "\t" + matrix.M21.ToString() + "\t" + matrix.M31.ToString() + "\t" + matrix.M41.ToString());
            sb.AppendLine(matrix.M12.ToString() + "\t" + matrix.M22.ToString() + "\t" + matrix.M32.ToString() + "\t" + matrix.M42.ToString());
            sb.AppendLine(matrix.M13.ToString() + "\t" + matrix.M23.ToString() + "\t" + matrix.M33.ToString() + "\t" + matrix.M43.ToString());
            sb.AppendLine(matrix.M14.ToString() + "\t" + matrix.M24.ToString() + "\t" + matrix.M34.ToString() + "\t" + matrix.M44.ToString());

            using (StreamWriter outfile = new StreamWriter(filename, false))
            {
                outfile.Write(sb.ToString());
            }
        }
        public Matrix4 OtherCameraToWorld { get { return otherCameraCameraToWorld; } }
    }
}