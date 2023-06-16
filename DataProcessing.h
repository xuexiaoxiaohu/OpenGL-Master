#pragma once
// QT
#include <QVector>
#include <QVector3D>
// PCL
#include <pcl/io/ply_io.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/vtk_lib_io.h>
//VTK
#include <vtkTriangleFilter.h>
#include <vtkFillHolesFilter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkImageStencil.h>
#include <vtkMarchingCubes.h>
#include <vtkImageGaussianSmooth.h>

class DataProcessing {
public:
	DataProcessing() {};
	~DataProcessing() {};

	void loadPointData(QString path);
	void addNormal(pcl::PolygonMesh &inMesh);
	void getMaxMinPoint(std::vector<QVector3D> data);
	void poly2tri(std::string src, std::string dst);

	int getNearestVertexIndex(QVector3D worldPos, std::vector<QVector3D> glVtx);
	std::vector<int> getIndicesFromRadiusSearch(pcl::PolygonMesh mesh, pcl::PointXYZ searchPoint, float radius = 4);
	void eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete);
	void fillMesh(pcl::PolygonMesh& mesh);
	void getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh, float radius);
	//Remeshing: Isotropic Explicit Remeshing
	void isoExpRemeshing(const char* srcPath,const char* dstPath);
	QVector<QVector3D>	pointData;
	std::vector<float> glMeshData;
	QVector3D maxPoint,minPoint;

	//lr 
	void getRenderData(pcl::PolygonMesh& mesh);
	void get_Normal(QVector3D p1, QVector3D p2, QVector3D p3, double& a, double& b, double& c);
	void getClipPlaneMesh(pcl::PolygonMesh& mesh, double a, double b, double c, QVector3D p);
	void getClipPlane_X0Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_X1Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Y0Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Y1Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Z0Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void getClipPlane_Z1Mesh(pcl::PolygonMesh& mesh,  QVector3D p);
	void clipPlaneMesh(pcl::PolygonMesh& mesh, double a, double b, double c, QVector3D p);
	QVector<QVector3D>	worldPos;
	void polyClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos);
	void boxClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos, double rayStart[3]);
	void polyLineClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos);

};