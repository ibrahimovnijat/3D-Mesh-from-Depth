#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

using namespace std;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; //0.01f; // 1cm
	
	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;
	
	// TODO: Determine number of valid faces
	unsigned int nFaces = 0;

	vector<string> faceIdVec;
	for (unsigned int i = 0; i < height-1; i++) {
		for (unsigned int j = 0; j < width-1; j++) {
			unsigned int vertex1 = i * width + j;		// get correct vertice indexes for each square box
			unsigned int vertex2 = vertex1 + 1;
			unsigned int vertex3 = (i+1) * width + j;
			unsigned int vertex4 = vertex3 + 1;
			float dist1 = 0.0f, dist2 = 0.0f, dist3 = 0.0f;

			Vector3f vec1 = Vector3f(vertices[vertex1].position[0], vertices[vertex1].position[1], vertices[vertex1].position[2]);
			Vector3f vec2 = Vector3f(vertices[vertex2].position[0], vertices[vertex2].position[1], vertices[vertex2].position[2]);
			Vector3f vec3 = Vector3f(vertices[vertex3].position[0], vertices[vertex3].position[1], vertices[vertex3].position[2]);
			Vector3f vec4 = Vector3f(vertices[vertex4].position[0], vertices[vertex4].position[1], vertices[vertex4].position[2]);
			
			//check if all edges of those two faces (for a single square) are less than threshold
			// 1st face (formed with vertex 1, 3 and 2, note that order matter)
			dist1 = (vec1 - vec3).norm();
			dist2 = (vec3 - vec2).norm();
			dist3 = (vec1 - vec2).norm();

			//check if all edges of the face is less than threshold a.k.a 0.01m
			stringstream ss1;
			if ((dist1 < edgeThreshold) && (dist2 < edgeThreshold) && (dist3 < edgeThreshold)) {
				nFaces += 1; 
				ss1 << "3" << " " << to_string(vertex1) << " " << to_string(vertex3) << " " << to_string(vertex2);
				faceIdVec.push_back(ss1.str());
			}
			// 2nd face (formed with vertex 2, 3 and 4, note that order matters)
			dist1 = (vec2 - vec3).norm();
			dist2 = (vec3 - vec4).norm();
			dist3 = (vec4 - vec2).norm();
			stringstream ss2;
			if ((dist1 < edgeThreshold) && (dist2 < edgeThreshold) && (dist3 < edgeThreshold)) {  
				nFaces += 1;
				ss2 << "3" << " " << to_string(vertex3) << " " << to_string(vertex4) << " " << to_string(vertex2);
				faceIdVec.push_back(ss2.str());
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << "# numVertices numFaces numEdges" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (unsigned int i = 0; i < nVertices; i++) {
		if ((vertices[i].position[0] == MINF) && (vertices[i].position[1] == MINF) && (vertices[i].position[2] == MINF)) {
			outFile << 0.0f << " " <<  0.0f << " " << 0.0f << " " << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1]<< " " << (int)vertices[i].color[2] << " " <<  (int)vertices[i].color[3] << endl;
		}
		else {
			outFile << (float)vertices[i].position[0] << " " <<  (float)vertices[i].position[1] << " " << (float)vertices[i].position[2] << " " <<(int)vertices[i].color[0] << " " << (int)vertices[i].color[1]<< " " << (int)vertices[i].color[2] << " " <<  (int)vertices[i].color[3] << endl; 
		}
	}

	// TODO: save valid faces
	std::cout << "# list of faces" << std::endl;
	std::cout << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	//create valid faces and add to file
	for (unsigned int i = 0; i < faceIdVec.size(); i++) {
		outFile << faceIdVec[i] << endl;
	}
	// close file
	outFile.close();
	return true;
}


int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	// std:string filenameIn = "../../Data/rgbd_dataset_freiburg2_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn)){
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	int sensorFrame = 0;
	while (sensor.ProcessNextFrame())
	{	
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();
		
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();		
		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		//---------------- Start here -------------
		uint16_t image_width = sensor.GetDepthImageWidth();
		uint16_t image_height = sensor.GetDepthImageHeight();

		unsigned int vertex_idx = -1;
		for (uint16_t r = 0; r < image_height; r++) {	
			for (uint16_t c = 0; c < image_width; c++) {
				vertex_idx += 1;
				float vertex_depth_val = depthMap[vertex_idx]; //depth pixel value
				if (vertex_depth_val == MINF) {
					vertices[vertex_idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[vertex_idx].color = Vector4uc(0,0,0,0);
				} else {
					Vector3f point_camera_coord = depthIntrinsicsInv * Vector3f(c, r, 1) * vertex_depth_val;
					Vector4f point_world_coord = trajectoryInv * depthExtrinsicsInv * Vector4f(point_camera_coord[0], point_camera_coord[1], point_camera_coord[2], 1.0f);
					vertices[vertex_idx].position = point_world_coord;  // copy pos vals to vertices array
					for (int i = 0; i < 4; i++){
						vertices[vertex_idx].color[i] = (unsigned char)colorMap[4*vertex_idx + i]; //copy rgba vals to vertices
					}
				}
			}
		}
		//-------------End here --------------

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		} else {
			std::cout << "File write successfull!" << std::endl;
		}
		// free mem
		delete[] vertices;
	}
	return 0;
}
