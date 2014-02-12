using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect.Toolkit.Fusion;
using System.Windows.Media.Media3D;
using System.Windows.Media;

namespace BLIKinect
{
    class MeshProcessor
    {
        public static MeshGeometry3D removeDuplicateVertices(Mesh oldMesh) {
            MeshGeometry3D newMesh = new MeshGeometry3D();
            Point3DCollection points = new Point3DCollection();
            Int32Collection newIndices = new Int32Collection();
            
            var vertices = oldMesh.GetVertices();


            var distinctVertices = vertices.Distinct();
            //List<Vector3> vectorArray = new List<Vector3>();

            Dictionary<Vector3, Int32> dict = new Dictionary<Vector3, int>();

            int count = 0;
            foreach(Vector3 location in distinctVertices){

                dict.Add(location, count);
                count++;
                points.Add(new Point3D(location.X, location.Y, location.Z));
            }

            var indices = oldMesh.GetTriangleIndexes();
            foreach (int i in indices) {
                int newIndex = dict[vertices.ElementAt(i)];
                newIndices.Add(newIndex);
            
            }
            Int32Collection cleanedIndices = new Int32Collection();
            for (int i = 0; i < newIndices.Count / 3; i++) {
                if (newIndices[i * 3] == newIndices[i * 3 + 1] || newIndices[i * 3] == newIndices[i * 3 + 2] || newIndices[i * 3 + 1] == newIndices[i * 3 + 2])
                {
                    continue;
                }
                else {
                    cleanedIndices.Add(newIndices[i * 3]);
                    cleanedIndices.Add(newIndices[i * 3+1]);
                    cleanedIndices.Add(newIndices[i * 3+2]);
                }
            
            }

            newMesh.Positions = points;
            newMesh.TriangleIndices = cleanedIndices;
            return newMesh;
        }

    }
}
