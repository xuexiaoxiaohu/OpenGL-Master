#include "MainWindow.h"
#include <filesystem>
#include <QFileDialog>
#include "MyGLWidget.h"
#include "Macro.h"
#include <QPixmap>
#include <QPainter>
#include <thread>
MainWindow::MainWindow(QWidget *parent)
    :QMainWindow(parent)
    ,isRenderRunning(true)
    ,isPlyFile(false){
	ui.setupUi(this);
    addOpengGLWidget();

    connect(ui.openPushBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(ui.stopPushBtn, SIGNAL(clicked()), this, SLOT(stopRendering()));
    connect(ui.grayScaleSlider, SIGNAL(valueChanged(int)), this, SLOT(setGrayValue(int)));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(RepaintUI()));
    connect(ui.checkBox, SIGNAL(clicked()), this, SLOT(choosePlyFile()));
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCursor()));
    timer->start(100);

    driver = new NDIDriver("COM3");
    surface = new SurfaceReconsturction();
    pointProc = new DataProcessing();
    meshProc = new DataProcessing();
}
void MainWindow::choosePlyFile() {
    isPlyFile = true;
}
void MainWindow::setGrayValue(int) {
    meshGLWidget->grayValue = ui.grayScaleSlider->value() / 10.0f;
    meshGLWidget->repaint();
}
void MainWindow::updateCursor() {
    meshGLWidget->geometry().contains(this->mapFromGlobal(QCursor::pos())) ? meshGLWidget->isMouseBrush = true:false;
}
MainWindow::~MainWindow(){
    delete driver;
    delete surface;
    delete pointGLWidget;
    delete meshGLWidget;
    delete pointProc;
    delete meshProc;
}

void MainWindow::addOpengGLWidget(){
    pointGLWidget = new MyGLWidget(this, PointType);
    pointGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(pointGLWidget);

    meshGLWidget = new MyGLWidget(this, MeshType);
    meshGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(meshGLWidget);
}
void MainWindow::RepaintUI() {
    pointGLWidget->repaint();
    meshGLWidget->repaint();
}
void MainWindow::openFile(){
    if (isPlyFile)
    {
        fileName = QFileDialog::getOpenFileName(this, "Open Scan Data", "./Release/Data", "Scan Data(*.ply)");
        
    }
    else
    {
        fileName = QFileDialog::getOpenFileName(this, "Open Scan Data", "./Release/Data", "Scan Data(*.txt)");
        if (fileName.isEmpty()) return;
        ui.fileLineEdit->setText(fileName);
        pointProc->loadPointData(fileName);
    }
   



}
void MainWindow::startRendering(){
    isRenderRunning = true;
    auto collectDataFunc = [=]() {
        if (isPlyFile)
        {
            pcl::PolygonMesh mesh;
            pcl::io::loadPLYFile(fileName.toStdString().c_str(), mesh);
            meshProc->addNormal(mesh);

            pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
            pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);

            std::vector<GLfloat> glMesh;
            for (int i = 0; i < mesh.polygons.size(); i++)
                for (int j = 0; j < mesh.polygons[i].vertices.size(); j++) {
                    pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
                    glMesh.emplace_back(point.x);
                    glMesh.emplace_back(point.y);
                    glMesh.emplace_back(point.z);
                    glMesh.emplace_back(point.normal_x);
                    glMesh.emplace_back(point.normal_y);
                    glMesh.emplace_back(point.normal_z);
                }
            QVector3D center = { -4.68985001, 72.03362747, 92.42464668 };
            //float radius = (pointProc->maxPoint - center).length();
            float radius = 45.0;
            meshGLWidget->setAdaptivePara(center, radius);
            meshGLWidget->setMesh(mesh);
            meshGLWidget->setImageData(glMesh);
 

            emit signal_glUpdate();
        }
        else
        {
            // Get data From NDI device.
            if (ui.fileLineEdit->text().contains("txt") == false) {
                while (!driver->set_tracking_status(true)) Sleep(10);

                while (isRenderRunning) {
                    static float lastTx = 0, lastTy = 0, lastTz = 0;
                    if (auto data = driver->get_data(); data.has_value()) {
                        auto& items = data.value();
                        for (auto& item : items) {
                            if (!item.transform.isMissing()) {
                                auto point = item.transform;
                                if (point.toolHandle == 11)
                                    if ((abs(lastTx - point.tx) > DELTA) && (abs(lastTy - point.ty) > DELTA)
                                        && (abs(lastTz - point.tz) > DELTA))
                                        if ((abs(point.tx) < VOLUME_MAX) && (abs(point.tz) < VOLUME_MAX)) {
                                            pointProc->pointData.push_back(QVector3D{ (float)point.tx ,(float)point.ty ,(float)point.tz });
                                            lastTx = point.tx; lastTy = point.ty; lastTz = point.tz;
                                        }
                                std::cout << item.timespec_s << " ," << item.frameNumber << " , " << point.toolHandle << ",  "
                                    << point.status << " , " << point.q0 << " , " << point.qx << " , " << point.qy << " , "
                                    << point.qz << ",  " << point.tx << ",  " << point.ty << ",  " << point.tz << std::endl;
                            }
                        }
                    }
                    enclosureDataProcessing();
                }
            }
            else {
                enclosureDataProcessing();
            }
        }

    };
    
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}

void MainWindow::stopRendering() {
    driver->set_tracking_status(false);
    isRenderRunning = false;

}
void MainWindow::enclosureDataProcessing(){
    std::vector<QVector3D> rawData;
    for (int i = 0; i < pointProc->pointData.size(); i++) {
        if (isRenderRunning){
            rawData.emplace_back(pointProc->pointData[i]);
            pointProc->getMaxMinPoint(rawData);

            QVector3D center = (pointProc->maxPoint + pointProc->minPoint) / 2.0f;
            //QVector3D center = { -4.68985001, 72.03362747, 92.42464668 };
            float radius = (pointProc->maxPoint - center).length();
            //float radius = 45.0;

            // Point
            std::vector<GLfloat> glPoint;
            for (int i = 0; i < rawData.size(); i++) {
                glPoint.emplace_back(pointProc->pointData[i].x());
                glPoint.emplace_back(pointProc->pointData[i].y());
                glPoint.emplace_back(pointProc->pointData[i].z());
            }
            pointGLWidget->setAdaptivePara(center, radius);
            pointGLWidget->setImageData(glPoint);

            // Mesh 
            if ((rawData.size() >= MIN_PTS_SIZE_REQD)) {
                int rawDataSize = rawData.size(), pointDataSize = pointProc->pointData.size();
                if (((rawData.size() % MESH_GRTH_SIZE) == 0) || (abs(rawDataSize - pointDataSize) <= 0)) {
                    surface->construction(rawData);

                    std::filesystem::path parentPath = std::filesystem::current_path();
                    std::string srcPath ="C:\\Project\\OpenGL-Rendering-Master-Build\\result.ply"; // 多边形网格
                    std::string dstPath = "C:\\Project\\OpenGL-Rendering-Master-Build\\triangleResult.ply"; // 三角形网格
                 /*   std::string srcPath = parentPath.string() + "/result.ply";
                    std::string dstPath = parentPath.string() + "/triangleResult.ply";*/
                    pcl::PolygonMesh mesh;
                    //meshProc->isoExpRemeshing(srcPath.c_str(), dstPath.c_str());// 均匀化会出现孔洞

                    
                    //std::string dstPath = "C:\\Users\\11024\\Desktop\\11\\mesh-mixed.ply"; // 三角形网格
                    meshProc->poly2tri(srcPath.c_str(), dstPath.c_str());// 仅仅将多边形网格变成三角形网格


                    pcl::io::loadPLYFile(dstPath.c_str(), mesh);
                    meshProc->addNormal(mesh);

                    pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
                    pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);

                    std::vector<GLfloat> glMesh;
                    for (int i = 0; i < mesh.polygons.size(); i++) 
                        for (int j = 0; j < mesh.polygons[i].vertices.size(); j++) {
                            pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
                            glMesh.emplace_back(point.x);
                            glMesh.emplace_back(point.y);
                            glMesh.emplace_back(point.z);
                            glMesh.emplace_back(point.normal_x);
                            glMesh.emplace_back(point.normal_y);
                            glMesh.emplace_back(point.normal_z);
                        }
                    meshGLWidget->setAdaptivePara(center, radius);
                    meshGLWidget->setMesh(mesh);
                    meshGLWidget->setImageData(glMesh);
                }
            }
            emit signal_glUpdate();
        }
    }  
}