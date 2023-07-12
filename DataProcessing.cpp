#include "DataProcessing.h"
#include <QApplication>
#include "Macro.h"
#include <qDebug>
#include <QFile>
#include <QDataStream>
#include <vtkMetaImageWriter.h>
#include <vtkClipPolyData.h>
#include <vtkPlane.h>
#include <vtkSelectPolyData.h>
#include <vtkPlanes.h>
#include <vtkFrustumSource.h>
#include <vtkIdFilter.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkCellLocator.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCellData.h>
#include <vtkPolyLine.h>
#include <vtkPolyPlane.h>

void DataProcessing::loadPointData(QString path) {
	QFile file(path);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))	return;

	QTextStream in(&file);
	while (!in.atEnd()) {
		QString line = in.readLine();
		QStringList parts = line.split(" ");
		QVector3D vector(parts[0].toFloat(), parts[1].toFloat(), parts[2].toFloat());
		pointData.append(vector);
	}
}

void DataProcessing::getMaxMinPoint(std::vector<QVector3D> data) {
	maxPoint = minPoint = {data[0].x() ,data[0].y() ,data[0].z()};
	for (int i = 0; i < data.size(); i++) {
		if (maxPoint.x() < data[i].x())	maxPoint.setX(data[i].x());
		if (maxPoint.y() < data[i].y())	maxPoint.setY(data[i].y());
		if (maxPoint.z() < data[i].z())	maxPoint.setZ(data[i].z());

		if (minPoint.x() > data[i].x())	minPoint.setX(data[i].x());
		if (minPoint.y() > data[i].y())	minPoint.setY(data[i].y());
		if (minPoint.z() > data[i].z())	minPoint.setZ(data[i].z());
	}
}

void DataProcessing::addNormal(pcl::PolygonMesh &mesh) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	std::vector<pcl::Indices> k_indices;
	std::vector<std::vector<float>> k_sqr_distances;
	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(cloud);
	search.nearestKSearch(*cloud, pcl::Indices(), NUM_NEIGHS, k_indices, k_sqr_distances);

	pcl::PointCloud<pcl::Normal>::Ptr normalsPtr(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsRefinedPtr(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	for (size_t i = 0; i < cloud->size(); ++i) {
		pcl::Normal normal;
		ne.computePointNormal(*cloud, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint((*cloud)[i], (*cloud).sensor_origin_[0], (*cloud).sensor_origin_[1],
			(*cloud).sensor_origin_[2], normal.normal_x, normal.normal_y, normal.normal_z);
		normalsPtr->emplace_back(normal);
	}

	pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(normalsPtr);
	nr.setMaxIterations(ITERATS);
	nr.setConvergenceThreshold(0.1);
	nr.filter(*normalsRefinedPtr);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normalsRefinedPtr, *cloud_with_normals);
	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*cloud_with_normals, outputCloud);
	mesh.cloud = outputCloud;
}

int DataProcessing::getNearestVertexIndex(QVector3D worldPos, std::vector<QVector3D> glVextex) {
	int index = -1;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < glVextex.size(); i++) {
		float dist = (glVextex[i] - worldPos).length();
		if (dist < minDist) {
			index = i;
			minDist = dist;
		}
	}
	return index;
}
std::vector<int> DataProcessing::getIndicesFromRadiusSearch(pcl::PolygonMesh mesh, pcl::PointXYZ searchPoint, float radius) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	std::vector<int> k_indices;
	std::vector<float> k_sqr_dists;

	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	kdtree->setInputCloud(cloud);
	kdtree->radiusSearch(searchPoint, radius, k_indices, k_sqr_dists);

	return k_indices;// 包含了最近点的索引
}
void DataProcessing::eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete) {
	std::vector<pcl::Vertices>& polygons = mesh.polygons;
	for (int i = 0; i < polygons.size(); ++i) {
		pcl::Vertices& vertices = polygons[i];
		bool isRemove = false;
		for (int j = 0; j < verticesToDelete.size(); ++j) {
			int index = verticesToDelete[j];
			if (std::find(vertices.vertices.begin(), vertices.vertices.end(), index) != vertices.vertices.end()) {
				isRemove = true;
				break;
			}
		}
		if (isRemove) {
			polygons.erase(polygons.begin() + i); 
			--i;
		}
	}
	mesh.polygons = polygons;
}
void DataProcessing::fillMesh(pcl::PolygonMesh& mesh) {
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkFillHolesFilter> fillHoles = vtkSmartPointer<vtkFillHolesFilter>::New();
	pcl::io::mesh2vtk(mesh, polydata);
	fillHoles->SetInputData(polydata);
	fillHoles->SetHoleSize(MAX_HOLE_SIZE);
	fillHoles->Update();
	pcl::io::vtk2mesh(fillHoles->GetOutput(), mesh);
}

void DataProcessing::getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh, float radius){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	std::vector<QVector3D> meshVertices;
	for (const auto& point:cloud){
		meshVertices.emplace_back(point.x, point.y, point.z);
	}
	int nearestIndex = getNearestVertexIndex(worldPos, meshVertices);
	if (nearestIndex != -1) {
		pcl::PointXYZ nrstVertex;
		nrstVertex.x = meshVertices[nearestIndex].x();
		nrstVertex.y = meshVertices[nearestIndex].y();
		nrstVertex.z = meshVertices[nearestIndex].z();

		pcl::Indices verticesNeedDelete = getIndicesFromRadiusSearch(mesh, nrstVertex, radius);
		eraseMesh(mesh, verticesNeedDelete);
		fillMesh(mesh);
		glMeshData.clear();
		pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);
		for (std::size_t i = 0; i < mesh.polygons.size(); i++) {
			for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++) {
				pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
				glMeshData.emplace_back(point.x);
				glMeshData.emplace_back(point.y);
				glMeshData.emplace_back(point.z);
				glMeshData.emplace_back(point.normal_x);
				glMeshData.emplace_back(point.normal_y);
				glMeshData.emplace_back(point.normal_z);
			}
		}
	}
}
void DataProcessing::poly2tri(std::string src, std::string dst) {
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(src.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> filter = vtkSmartPointer<vtkTriangleFilter>::New();
	filter->SetInputData(reader->GetOutput());

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetFileName(dst.c_str());
	writer->SetInputConnection(filter->GetOutputPort());
	writer->SetFileTypeToASCII();
	writer->Update();
}
	
void DataProcessing::isoExpRemeshing(const char* srcPath, const char* dstPath) {
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(srcPath);
	reader->Update();
	auto polyData = reader->GetOutput();

	vtkSmartPointer<vtkImageData> whiteImage = vtkSmartPointer<vtkImageData>::New();
	double bounds[6];
	polyData->GetBounds(bounds);
	double spacing[3] = {0.5,0.5,0.5};
	int dimension[3];
	for (int i = 0; i < 3; i++) {
		dimension[i] = static_cast<int>(ceil((bounds[i * 2 + 1] - bounds[i * 2]) / spacing[i]));
	}
	whiteImage->SetDimensions(dimension);
	whiteImage->SetExtent(0, dimension[0] - 1, 0, dimension[1] - 1, 0, dimension[2] - 1);

	double origin[3];
	origin[0] = bounds[0] + spacing[0] / 2;
	origin[1] = bounds[2] + spacing[1] / 2;
	origin[2] = bounds[4] + spacing[2] / 2;
	whiteImage->SetOrigin(origin);
	whiteImage->SetSpacing(spacing);
	whiteImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

	unsigned char inval = 255, outval = 0;
	vtkIdType count = whiteImage->GetNumberOfPoints();
	for (vtkIdType i = 0; i < count; ++i) {
		whiteImage->GetPointData()->GetScalars()->SetTuple1(i, inval);
	}
	


	vtkSmartPointer<vtkPolyDataToImageStencil> pol2stenc = vtkSmartPointer<vtkPolyDataToImageStencil>::New();
	pol2stenc->SetInputData(polyData);
	pol2stenc->SetOutputOrigin(origin);
	pol2stenc->SetOutputSpacing(spacing);
	pol2stenc->SetOutputWholeExtent(whiteImage->GetExtent());
	pol2stenc->Update();

	vtkSmartPointer<vtkImageStencil> imageStenc = vtkSmartPointer<vtkImageStencil>::New();
	imageStenc->SetInputData(whiteImage);
	imageStenc->SetStencilConnection(pol2stenc->GetOutputPort());
	imageStenc->ReverseStencilOff();
	imageStenc->SetBackgroundValue(outval);
	imageStenc->Update();


	vtkSmartPointer<vtkImageGaussianSmooth> gaussianSmooth = vtkSmartPointer<vtkImageGaussianSmooth>::New();
	gaussianSmooth->SetInputConnection(imageStenc->GetOutputPort());
	gaussianSmooth->SetDimensionality(3);
	gaussianSmooth->SetRadiusFactor(5);
	gaussianSmooth->SetStandardDeviation(1);
	gaussianSmooth->Update();




	vtkSmartPointer<vtkMarchingCubes>marchingcube = vtkSmartPointer<vtkMarchingCubes>::New();
	marchingcube->SetInputData(gaussianSmooth->GetOutput());
	marchingcube->SetValue(0, 70);
	marchingcube->ComputeNormalsOn();
	marchingcube->Update();

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetFileName(dstPath);
	writer->SetInputData(marchingcube->GetOutput());
	writer->SetFileTypeToASCII();
	writer->Update();
}



// lr 
void DataProcessing::getRenderData(pcl::PolygonMesh& mesh) {
	glMeshData.clear();
	pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);
	for (std::size_t i = 0; i < mesh.polygons.size(); i++) {
		for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++) {
			pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
			glMeshData.emplace_back(point.x);
			glMeshData.emplace_back(point.y);
			glMeshData.emplace_back(point.z);
			glMeshData.emplace_back(point.normal_x);
			glMeshData.emplace_back(point.normal_y);
			glMeshData.emplace_back(point.normal_z);
		}
	}
}


// 已知三点坐标，求法向量
void DataProcessing::get_Normal(QVector3D p1, QVector3D p2, QVector3D p3, double& a, double& b, double& c)

{
	//qDebug() << "before cal a: " << a;
	a = ((p2.y() - p1.y()) * (p3.z() - p1.z()) - (p2.z() - p1.z()) * (p3.y() - p1.y()));
	//qDebug() << "after cal a: " << a;
	b = ((p2.z() - p1.z()) * (p3.x() - p1.x()) - (p2.x() - p1.x()) * (p3.z() - p1.z()));
	c = ((p2.x() - p1.x()) * (p3.y() - p1.y()) - (p2.y() - p1.y()) * (p3.x() - p1.x()));
}




void DataProcessing::getClipPlaneMesh(pcl::PolygonMesh& mesh, double a, double b, double c, QVector3D p)
{
	clipPlaneMesh(mesh, a, b, c, p);
	getRenderData(mesh);

}


void DataProcessing::getClipPlane_X0Mesh(pcl::PolygonMesh& mesh,  QVector3D p)
{
	clipPlaneMesh(mesh, 1.0, 0.0, 0.0, p);// 
	getRenderData(mesh);

}
void DataProcessing::getClipPlane_X1Mesh(pcl::PolygonMesh& mesh, QVector3D p)
{
	clipPlaneMesh(mesh, -1.0, 0.0, 0.0, p);// 
	getRenderData(mesh);

}
void DataProcessing::getClipPlane_Y0Mesh(pcl::PolygonMesh& mesh, QVector3D p)
{
	clipPlaneMesh(mesh, 0.0, 1.0, 0.0, p);// 
	getRenderData(mesh);

}
void DataProcessing::getClipPlane_Y1Mesh(pcl::PolygonMesh& mesh, QVector3D p)
{
	clipPlaneMesh(mesh, 0.0, -1.0, 0.0, p);// 
	getRenderData(mesh);

}
void DataProcessing::getClipPlane_Z0Mesh(pcl::PolygonMesh& mesh, QVector3D p)
{
	clipPlaneMesh(mesh, 0.0, 0.0, 1.0, p);// 
	getRenderData(mesh);

}
void DataProcessing::getClipPlane_Z1Mesh(pcl::PolygonMesh& mesh, QVector3D p)
{
	clipPlaneMesh(mesh, 0.0, 0.0, -1.0, p);// 
	getRenderData(mesh);

}
void DataProcessing::clipPlaneMesh(pcl::PolygonMesh& mesh, double a, double b, double c, QVector3D p)
{

	//qDebug() << "before clip , mesh size " << mesh.polygons.size();
	//qDebug() << "before clip , a,b, c " << a<< " "<<b<<" "<<c;
	vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata1);
	//qDebug() << "before clip , polydata1 size " << polydata1->GetNumberOfPoints();
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetInputData(polydata1);

	vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();


	plane->SetOrigin(p.x(), p.y(), p.z());
	plane->SetNormal(a, b, c);
	

	clipper->SetClipFunction(plane);

	clipper->InsideOutOn(); 
	clipper->Update();
	vtkSmartPointer<vtkPolyData> output = clipper->GetOutput();

	vtkSmartPointer<vtkClipPolyData> clipper2 = vtkSmartPointer<vtkClipPolyData>::New();
	clipper2->SetInputData(polydata1);

	clipper2->SetClipFunction(plane);

	clipper2->InsideOutOff();
	clipper2->Update();
	vtkSmartPointer<vtkPolyData> output2 = clipper2->GetOutput();

	//qDebug() << "after clip , vtkouput size " << output->GetNumberOfPoints();
	if (output->GetNumberOfPoints() > output2->GetNumberOfPoints())
	{
		pcl::io::vtk2mesh(output, mesh);
	}
	else
	{
		pcl::io::vtk2mesh(output2, mesh);
	}
	
	
}

void DataProcessing::polyClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos)
{
	vtkNew<vtkPoints> selectionPoints;

	for (int i = 0; i < worldPos.size(); i++)
	{
		selectionPoints->InsertPoint(i, worldPos[i].x(),worldPos[i].y(),worldPos[i].z());
		//cout << i<<" " << worldPos[i].x()<<" " << worldPos[i].y()<<" " << worldPos[i].z()<<" " << endl;
	}

	vtkSmartPointer<vtkPolyData> polydata3 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata3);
	vtkSmartPointer<vtkSelectPolyData> selectPolyData = vtkSmartPointer<vtkSelectPolyData>::New();
	selectPolyData->SetInputData(polydata3);
	selectPolyData->SetLoop(selectionPoints);
	selectPolyData->GenerateUnselectedOutputOn();
	selectPolyData->Update();
	pcl::io::vtk2mesh(selectPolyData->GetUnselectedOutput(), mesh);//多点删除

}


void DataProcessing::polyExtract(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos)
{
	vtkNew<vtkPoints> selectionPoints;

	for (int i = 0; i < worldPos.size(); i++)
	{
		selectionPoints->InsertPoint(i, worldPos[i].x(), worldPos[i].y(), worldPos[i].z());
		//cout << i<<" " << worldPos[i].x()<<" " << worldPos[i].y()<<" " << worldPos[i].z()<<" " << endl;
	}

	vtkSmartPointer<vtkPolyData> polydata3 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata3);
	vtkSmartPointer<vtkSelectPolyData> selectPolyData = vtkSmartPointer<vtkSelectPolyData>::New();
	selectPolyData->SetInputData(polydata3);
	selectPolyData->SetLoop(selectionPoints);
	selectPolyData->GenerateUnselectedOutputOn();
	selectPolyData->Update();
	pcl::io::vtk2mesh(selectPolyData->GetOutput(), mesh); //多点提取

}


void DataProcessing::polyLineClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos)
{
	vtkSmartPointer<vtkPoints>polyline_pts = vtkSmartPointer<vtkPoints>::New();

	for (int i = 0; i < worldPos.size(); i++)
	{
		double pt[3] = { worldPos[i].x(),worldPos[i].y(),worldPos[i].z() };
		polyline_pts->InsertNextPoint(pt);
	}
	//double pt[3] = { worldPos[0].x(),worldPos[0].y(),worldPos[0].z() }; // 是为了开头结尾相接
	//polyline_pts->InsertNextPoint(pt);
	vtkSmartPointer<vtkPolyLine> polyline = vtkSmartPointer<vtkPolyLine>::New();
	polyline->GetPointIds()->SetNumberOfIds(worldPos.size());
	//polyline->GetPointIds()->SetNumberOfIds(worldPos.size()+1);
	for (unsigned int i = 0; i < worldPos.size(); i++)
	//for (unsigned int i = 0; i < worldPos.size()+1; i++)
	{
		polyline->GetPointIds()->SetId(i, i);
	}
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(polyline);
	vtkSmartPointer<vtkPolyData> polylineData = vtkSmartPointer<vtkPolyData>::New();
	polylineData->SetPoints(polyline_pts);
	polylineData->SetLines(cells);

	// 沿折线进行裁剪

	vtkSmartPointer<vtkPolyLine> PolyLine = vtkSmartPointer<vtkPolyLine>::New();
	PolyLine = vtkPolyLine::SafeDownCast(polylineData->GetCell(0));
	
	//clip
	vtkSmartPointer<vtkPolyPlane>polyPlane = vtkSmartPointer<vtkPolyPlane>::New();
	polyPlane->SetPolyLine(PolyLine);
	vtkSmartPointer<vtkPolyData> polydata4 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata4);

	vtkSmartPointer<vtkClipPolyData>clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetInputData(polydata4);
	clipper->SetClipFunction(polyPlane);
	/*clipper->InsideOutOn();*/
	clipper->Update();

	/*vtkSmartPointer<vtkClipPolyData>clipper2 = vtkSmartPointer<vtkClipPolyData>::New();
	clipper2->SetInputData(polydata4);
	clipper2->SetClipFunction(polyPlane);
	clipper2->InsideOutOff();
	clipper2->Update();

	if (clipper->GetOutput()->GetNumberOfPoints() > clipper2->GetOutput()->GetNumberOfPoints())
	{
		cout << "clipper->GetOutput()->GetNumberOfPoints(): " << clipper->GetOutput()->GetNumberOfPoints() << endl;
		pcl::io::vtk2mesh(clipper->GetOutput(), mesh);
	}
	else
	{
		cout << "clipper2->GetOutput()->GetNumberOfPoints(): " << clipper2->GetOutput()->GetNumberOfPoints() << endl;
		pcl::io::vtk2mesh(clipper2->GetOutput(), mesh);
	}*/
	pcl::io::vtk2mesh(clipper->GetOutput(), mesh);
	

}



void DataProcessing::boxClip(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos, double rayStart[3])
{
	
	double p11[3] = { worldPos[0].x(),worldPos[0].y(),worldPos[0].z() };
	double p21[3] = { worldPos[1].x(),worldPos[1].y(),worldPos[1].z() };
	double p31[3] = { worldPos[2].x(),worldPos[2].y(),worldPos[2].z() };
	double p41[4] = { worldPos[3].x(),worldPos[3].y(),worldPos[3].z() };
	double* points[4] = { p11,p21,p31,p41 };

	vtkSmartPointer<vtkPoints> polydataPoints = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < 4; i++)
	{
		polydataPoints->InsertNextPoint(points[i]);
	}
	vtkSmartPointer<vtkIdList> lineIds = vtkSmartPointer<vtkIdList>::New();
	lineIds->SetNumberOfIds(5);
	lineIds->SetId(0, 0);
	lineIds->SetId(1, 1);
	lineIds->SetId(2, 2);
	lineIds->SetId(3, 3);
	lineIds->SetId(4, 0);
	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
	lines->InsertNextCell(lineIds);
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(polydataPoints);
	polydata->SetLines(lines);
	double bounds[6];
	polydata->GetBounds(bounds);
	/*std::cout << "X range: " << bounds[0] << " - " << bounds[1] << std::endl;
	std::cout << "Y range: " << bounds[2] << " - " << bounds[3] << std::endl;
	std::cout << "Z range: " << bounds[4] << " - " << bounds[5] << std::endl;
	std::cout << "***" << std::endl;*/
	double xmin = bounds[0];
	double xmax = bounds[1];
	double ymin = bounds[2];
	double ymax = bounds[3];
	double zmin = bounds[4];
	double zmax = bounds[5];

	// 创建裁剪平面的法向量和截距
	double planes[24] = {
		1, 0, 0, -xmin, // left
		-1, 0, 0, xmax, // right
		0, 1, 0, -ymin, // bottom
		0, -1, 0, ymax, // top
		0, 0, 1, -zmin, // near
		0, 0, -1, zmax  // far
	};


	// 创建vtkPlanes对象，并将裁剪平面的参数设置给它
	vtkSmartPointer<vtkPlanes> clippingPlanes = vtkSmartPointer<vtkPlanes>::New();
	clippingPlanes->SetFrustumPlanes(planes);

	// 使用vtkFrustumSource类创建视锥体，并将裁剪平面设置给它
	vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
	frustumSource->SetPlanes(clippingPlanes);
	frustumSource->Update();


	vtkPlanes* frustum = frustumSource->GetPlanes();

	//提前标记几何数据的CellId
	vtkIdFilter* idFilter = vtkIdFilter::New();
	vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata2);
	/* std::cout << "初始模型点个数： " << polydata2->GetNumberOfPoints() << std::endl;
	 std::cout << "初始模型面片个数： " << polydata2->GetNumberOfCells() << std::endl;*/

	idFilter->SetInputData(polydata2);
	// idFilter->SetCellIdsArrayName("OriginalCellId");
	idFilter->Update();
	//提取视锥体内的模型
	vtkExtractPolyDataGeometry* extract = vtkExtractPolyDataGeometry::New();
	extract->SetInputConnection(idFilter->GetOutputPort());
	extract->SetImplicitFunction(frustum);
	extract->Update();
	if (!extract->GetOutput()->GetPolys())
	{
		std::cout << "faild!" << std::endl;
		return;
	}



	//创建面片定位器
	vtkCellLocator* locator = vtkCellLocator::New();
	locator->SetDataSet(extract->GetOutput());
	locator->BuildLocator();
	//----------利用光线投射的方法寻找更靠近摄像机的面片------------

	/*double rayStart[3] = { this->camera->eye[0],this->camera->eye[1],this->camera->eye[2] };*///光线起点坐标：设置为摄像机位置
	double rayDirection[3];			//光线方向向量：设置为框选数据包围盒的中心
	extract->GetOutput()->GetCenter(rayDirection);
	//std::cout << "center of box : " << rayDirection[0] << " " << rayDirection[1] << " " << rayDirection[2] << std::endl;
	//std::cout << " ray start " << rayStart[0] << " " << rayStart[1] << " " << rayStart[2] << std::endl;
	double xyz[3];
	double t;
	double pcoords[3];
	int subId;
	vtkIdType cellId = -1;			//记录光线击中的面片Id号


	locator->IntersectWithLine(rayStart, rayDirection, 0.0001, t, xyz, pcoords, subId, cellId);
	//-----------利用找到的面片获取相连的面
	vtkPolyDataConnectivityFilter* connectivity = vtkPolyDataConnectivityFilter::New();
	connectivity->SetInputConnection(extract->GetOutputPort());
	connectivity->SetExtractionModeToCellSeededRegions();
	connectivity->InitializeSeedList();
	connectivity->AddSeed(cellId);
	connectivity->Update();

	// 感兴趣框删除
	vtkIdTypeArray* ids = dynamic_cast<vtkIdTypeArray*>(connectivity->GetOutput()->GetCellData()->GetArray(0));
	polydata2->BuildLinks();
	if (!ids) return;

	for (int i = 0; i < ids->GetNumberOfValues(); i++) {
		vtkIdType id = ids->GetValue(i);
		polydata2->DeleteCell(id);
	}

	polydata2->RemoveDeletedCells();
	polydata2->Modified();
	pcl::io::vtk2mesh(polydata2, mesh);

	
}

void DataProcessing::boxExtract(pcl::PolygonMesh& mesh, QVector<QVector3D> worldPos, double rayStart[3])
{

	double p11[3] = { worldPos[0].x(),worldPos[0].y(),worldPos[0].z() };
	double p21[3] = { worldPos[1].x(),worldPos[1].y(),worldPos[1].z() };
	double p31[3] = { worldPos[2].x(),worldPos[2].y(),worldPos[2].z() };
	double p41[4] = { worldPos[3].x(),worldPos[3].y(),worldPos[3].z() };
	double* points[4] = { p11,p21,p31,p41 };

	vtkSmartPointer<vtkPoints> polydataPoints = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < 4; i++)
	{
		polydataPoints->InsertNextPoint(points[i]);
	}
	vtkSmartPointer<vtkIdList> lineIds = vtkSmartPointer<vtkIdList>::New();
	lineIds->SetNumberOfIds(5);
	lineIds->SetId(0, 0);
	lineIds->SetId(1, 1);
	lineIds->SetId(2, 2);
	lineIds->SetId(3, 3);
	lineIds->SetId(4, 0);
	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
	lines->InsertNextCell(lineIds);
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(polydataPoints);
	polydata->SetLines(lines);
	double bounds[6];
	polydata->GetBounds(bounds);
	/*std::cout << "X range: " << bounds[0] << " - " << bounds[1] << std::endl;
	std::cout << "Y range: " << bounds[2] << " - " << bounds[3] << std::endl;
	std::cout << "Z range: " << bounds[4] << " - " << bounds[5] << std::endl;
	std::cout << "***" << std::endl;*/
	double xmin = bounds[0];
	double xmax = bounds[1];
	double ymin = bounds[2];
	double ymax = bounds[3];
	double zmin = bounds[4];
	double zmax = bounds[5];

	// 创建裁剪平面的法向量和截距
	double planes[24] = {
		1, 0, 0, -xmin, // left
		-1, 0, 0, xmax, // right
		0, 1, 0, -ymin, // bottom
		0, -1, 0, ymax, // top
		0, 0, 1, -zmin, // near
		0, 0, -1, zmax  // far
	};


	// 创建vtkPlanes对象，并将裁剪平面的参数设置给它
	vtkSmartPointer<vtkPlanes> clippingPlanes = vtkSmartPointer<vtkPlanes>::New();
	clippingPlanes->SetFrustumPlanes(planes);

	// 使用vtkFrustumSource类创建视锥体，并将裁剪平面设置给它
	vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
	frustumSource->SetPlanes(clippingPlanes);
	frustumSource->Update();


	vtkPlanes* frustum = frustumSource->GetPlanes();

	//提前标记几何数据的CellId
	vtkIdFilter* idFilter = vtkIdFilter::New();
	vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
	pcl::io::mesh2vtk(mesh, polydata2);
	/* std::cout << "初始模型点个数： " << polydata2->GetNumberOfPoints() << std::endl;
	 std::cout << "初始模型面片个数： " << polydata2->GetNumberOfCells() << std::endl;*/

	idFilter->SetInputData(polydata2);
	// idFilter->SetCellIdsArrayName("OriginalCellId");
	idFilter->Update();
	//提取视锥体内的模型
	vtkExtractPolyDataGeometry* extract = vtkExtractPolyDataGeometry::New();
	extract->SetInputConnection(idFilter->GetOutputPort());
	extract->SetImplicitFunction(frustum);
	extract->Update();
	if (!extract->GetOutput()->GetPolys())
	{
		std::cout << "faild!" << std::endl;
		return;
	}



	//创建面片定位器
	vtkCellLocator* locator = vtkCellLocator::New();
	locator->SetDataSet(extract->GetOutput());
	locator->BuildLocator();
	//----------利用光线投射的方法寻找更靠近摄像机的面片------------

	/*double rayStart[3] = { this->camera->eye[0],this->camera->eye[1],this->camera->eye[2] };*///光线起点坐标：设置为摄像机位置
	double rayDirection[3];			//光线方向向量：设置为框选数据包围盒的中心
	extract->GetOutput()->GetCenter(rayDirection);
	//std::cout << "center of box : " << rayDirection[0] << " " << rayDirection[1] << " " << rayDirection[2] << std::endl;
	//std::cout << " ray start " << rayStart[0] << " " << rayStart[1] << " " << rayStart[2] << std::endl;
	double xyz[3];
	double t;
	double pcoords[3];
	int subId;
	vtkIdType cellId = -1;			//记录光线击中的面片Id号


	locator->IntersectWithLine(rayStart, rayDirection, 0.0001, t, xyz, pcoords, subId, cellId);
	//-----------利用找到的面片获取相连的面
	vtkPolyDataConnectivityFilter* connectivity = vtkPolyDataConnectivityFilter::New();
	connectivity->SetInputConnection(extract->GetOutputPort());
	connectivity->SetExtractionModeToCellSeededRegions();
	connectivity->InitializeSeedList();
	connectivity->AddSeed(cellId);
	connectivity->Update();

	//// 感兴趣框提取
	vtkSmartPointer<vtkPolyData> polydata3 = vtkSmartPointer<vtkPolyData>::New();
	polydata3 = connectivity->GetOutput();
	pcl::io::vtk2mesh(polydata3, mesh);
}
