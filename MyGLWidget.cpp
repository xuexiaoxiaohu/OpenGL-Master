//OpenGL
#include <glut.h>
// Qt
#include <QCoreApplication>
#include <QMessageBox>
#include <QPixmap>
#include <QPainter>
#include <QTimer>
// Custom
#include "MyGLWidget.h"
#include <Macro.h>
// lr 
#include <vtkImplicitSelectionLoop.h>
#include <vtkSelectPolyData.h>
#include <vtkPlanes.h>
#include <vtkFrustumSource.h>
#include <vtkIdFilter.h>
#include <vtkExtractPolyDataGeometry.h>
#include <vtkCellLocator.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkProperty.h>

MyGLWidget::MyGLWidget(QWidget* parent,int dataType)
    : rotationAngle(0.0f)
    , dataType(dataType)
    , isShiftPressed(false)
    , grayValue(0.5f)
    , brushPosition(0.0f,0.0f,-0.5f)
    , brushSize(4)
    , isMouseBrush(false)
{
    camera = new Camera();
    glDataProc = new DataProcessing();
    pShader = new QOpenGLShaderProgram();
    mShader = new QOpenGLShaderProgram();

    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
    
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCursor()));
    timer->start(100);
}
void MyGLWidget::updateCursor() {
    if (isShiftPressed && isMouseBrush){
        QPixmap pixmap(brushSize, brushSize);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        //painter.setPen(Qt::red);
        painter.setBrush(Qt::red);
        painter.drawEllipse(0, 0, brushSize, brushSize);
        QCursor cursor(pixmap);
        setCursor(cursor);
    }else{
        setCursor(Qt::ArrowCursor);
    }
}
MyGLWidget::~MyGLWidget(){
    delete pShader;
    delete mShader;
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);
}
void MyGLWidget::setMesh(pcl::PolygonMesh mesh) {
    this->mesh = mesh;
}
void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::setAdaptivePara(QVector3D center, float radius){
    camera->center = center;
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.01f, 20 * radius);
}

void MyGLWidget::initializeGL(){
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    QString qAppDir = QCoreApplication::applicationDirPath();
    QString pointVert = qAppDir + "/Shader/point.vert", pointFrag = qAppDir + "/Shader/point.frag";
    pShader->addShaderFromSourceFile(QOpenGLShader::Vertex, pointVert);
    pShader->addShaderFromSourceFile(QOpenGLShader::Fragment, pointFrag);
    pShader->link();

    QString meshVert = qAppDir + "/Shader/mesh.vert", meshFrag = qAppDir + "/Shader/mesh.frag";
    mShader->addShaderFromSourceFile(QOpenGLShader::Vertex, meshVert);
    mShader->addShaderFromSourceFile(QOpenGLShader::Fragment, meshFrag);
    mShader->link();
}
// PaintGL
void MyGLWidget::paintGL(){
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (DataType::PointType == dataType) {
        glPointSize(1.0f);
        glGenVertexArrays(1, &pVAO);
        glBindVertexArray(pVAO);
        glGenBuffers(1, &pVBO);
        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        
        pShader->bind();
        pShader->setUniformValue("model", model);
        pShader->setUniformValue("view", camera->getViewMatrix());
        pShader->setUniformValue("proj", proj);

        glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
    }else{
        glGenVertexArrays(1, &mVAO);
        glBindVertexArray(mVAO);
        glGenBuffers(1, &mVBO);
        glBindBuffer(GL_ARRAY_BUFFER, mVBO);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        mShader->bind();
        mShader->setUniformValue("viewPos", QVector3D(0.0f, 0.0f, 3.0f));

        QVector3D grayValue3D(grayValue, grayValue,grayValue);
        QVector3D dirLight1(1.0f, 1.0f, 3.0f), dirLight2(1.0f, 1.0f, -3.0f);

        mShader->setUniformValue("mtrl.ambient", grayValue3D);
        mShader->setUniformValue("mtrl.diffuse", grayValue3D);
        mShader->setUniformValue("mtrl.specular", grayValue3D);
        mShader->setUniformValue("mtrl.shininess", 16.0f);

        mShader->setUniformValue("dirLight1.ambient", grayValue3D);
        mShader->setUniformValue("dirLight1.diffuse", grayValue3D);
        mShader->setUniformValue("dirLight1.specular", grayValue3D);
        mShader->setUniformValue("dirLight1.direction", dirLight1);

        mShader->setUniformValue("dirLight2.ambient", grayValue3D);
        mShader->setUniformValue("dirLight2.diffuse", grayValue3D);
        mShader->setUniformValue("dirLight2.specular", grayValue3D);
        mShader->setUniformValue("dirLight2.direction", dirLight2);

        mShader->setUniformValue("model", model);
        mShader->setUniformValue("view", camera->getViewMatrix());
        mShader->setUniformValue("proj", proj);  

        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);    
    }
}
void MyGLWidget::resizeGL(int width, int height){
    glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    QPoint currentMousePos = event->pos();
    QVector3D diff = QVector3D(currentMousePos - lastMousePos);
    rotationAngle += (diff.length() / 2.0f);
    QVector3D rotationAxis = QVector3D(diff.y(), diff.x(), 0.0f).normalized();
    model.setToIdentity();
    if (event->buttons() & Qt::LeftButton) {
        if (isShiftPressed) {
            int deltaX = currentMousePos.x() - lastMousePos.x();
            int deltaY = currentMousePos.y() - lastMousePos.y();
            int  distance = sqrt(deltaX * deltaX + deltaY * deltaY);
            if (distance >= 2.0) {
                lastMousePos = currentMousePos;
                GLdouble wx = 0, wy = 0, wz = 0;
                convScreen2World(currentMousePos, wx, wy, wz);
                glDataProc->getErasedMesh(QVector3D(wx, wy, wz), mesh, 0.1 * brushSize);
                setImageData(glDataProc->glMeshData);
            }
        }else{
           model.translate(camera->center);
           model.rotate(rotationAngle, rotationAxis);
           model.translate(-camera->center);
        }
    }
    if (event->buttons() & Qt::RightButton) {
        model.translate(currentMousePos.x()/20, currentMousePos.y()/20);
    }
    repaint();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (event->buttons() & Qt::LeftButton) 
        lastMousePos = event->pos();

    // lr
    if (isPolyClipped && event->button() == Qt::LeftButton)
    {
        //��¼��ǰ����λ�ã�
        QPoint pos = event->pos();
        m_points.append(pos);
    }
    // lr
    if (isBoxClipped && event->button() == Qt::LeftButton)
    {
        QPoint pos = event->pos();
        m_points.append(pos);
    }
    //lr 
    if (idSliceClipped && event->button() == Qt::LeftButton)
    {
        QPoint pos = event->pos();
        m_points.append(pos);
    }
}
void MyGLWidget::mouseReleaseEvent(QMouseEvent* event) {

}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() & Qt::Key_Shift) {
        isShiftPressed = true;
    }
    if (event->key() == Qt::Key_A){
        isPolyClipped = !isPolyClipped;
        if (!isPolyClipped){
            /*vtkNew<vtkPoints> selectionPoints;
            GLdouble wx, wy, wz;
            for (int i = 0; i < m_points.size(); i++)
            {
                convScreen2World(m_points[i], wx, wy, wz);  
                selectionPoints->InsertPoint(i, wx, wy, wz);
            }
            vtkSmartPointer<vtkPolyData> polydata3 = vtkSmartPointer<vtkPolyData>::New();
            pcl::io::mesh2vtk(this->mesh, polydata3);
            vtkSmartPointer<vtkSelectPolyData> selectPolyData = vtkSmartPointer<vtkSelectPolyData>::New();
            selectPolyData->SetInputData(polydata3);
            selectPolyData->SetLoop(selectionPoints);
            selectPolyData->GenerateUnselectedOutputOn();
            selectPolyData->Update();
            pcl::io::vtk2mesh(selectPolyData->GetUnselectedOutput(), mesh);*/

            // 

            GLdouble wx, wy, wz;
            QVector<QVector3D>	worldPos;
            for (int i = 0; i < m_points.size(); i++)
            {
                convScreen2World(m_points[i], wx, wy, wz);
                worldPos.push_back(QVector3D(wx, wy, wz));
            }
            glDataProc->polyClip(mesh, worldPos);

            //qDebug() << "after box choose  , mesh size " << this->mesh.polygons.size();
            glDataProc->getRenderData(mesh);
            setImageData(glDataProc->glMeshData);
            repaint();
            m_points.clear();
        }

    }

    if (event->key() == Qt::Key_B)
    {
        isBoxClipped = !isBoxClipped;
        if (!isBoxClipped)
        {
            // compute box vertex
            QPoint p1(m_points[0].x(), m_points[0].y());
            QPoint p2(m_points[0].x(), m_points[1].y());
            QPoint p3(m_points[1].x(), m_points[1].y());
            QPoint p4(m_points[1].x(), m_points[0].y());
            GLdouble wx1, wy1, wz1, wx2, wy2, wz2, wx3, wy3, wz3, wx4, wy4, wz4;
            convScreen2World(p1, wx1, wy1, wz1);
            convScreen2World(p2, wx2, wy2, wz2);
            convScreen2World(p3, wx3, wy3, wz3);
            convScreen2World(p4, wx4, wy4, wz4);


            QVector<QVector3D>	worldPos;
            worldPos.push_back(QVector3D(wx1, wy1, wz1));
            worldPos.push_back(QVector3D(wx2, wy2, wz2));
            worldPos.push_back(QVector3D(wx3, wy3, wz3));
            worldPos.push_back(QVector3D(wx4, wy4, wz4));

           // double p11[3] = { wx1,wy1,wz1 };
           // double p21[3] = { wx2,wy2,wz2 };
           // double p31[3] = { wx3,wy3,wz3 };
           // double p41[3] = { wx4,wy4,wz4 };
           // double* points[4] = { p11,p21,p31,p41 };
           // 
           // vtkSmartPointer<vtkPoints> polydataPoints = vtkSmartPointer<vtkPoints>::New();
           // for (int i = 0; i < 4; i++)
           // {
           //     polydataPoints->InsertNextPoint(points[i]);
           // }
           // vtkSmartPointer<vtkIdList> lineIds = vtkSmartPointer<vtkIdList>::New();
           // lineIds->SetNumberOfIds(5);
           // lineIds->SetId(0, 0);
           // lineIds->SetId(1, 1);
           // lineIds->SetId(2, 2);
           // lineIds->SetId(3, 3);
           // lineIds->SetId(4, 0);
           // vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
           // lines->InsertNextCell(lineIds);
           // vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
           // polydata->SetPoints(polydataPoints);
           // polydata->SetLines(lines);
           // double bounds[6];
           // polydata->GetBounds(bounds);
           // /*std::cout << "X range: " << bounds[0] << " - " << bounds[1] << std::endl;
           // std::cout << "Y range: " << bounds[2] << " - " << bounds[3] << std::endl;
           // std::cout << "Z range: " << bounds[4] << " - " << bounds[5] << std::endl;
           // std::cout << "***" << std::endl;*/
           // double xmin = bounds[0];
           // double xmax = bounds[1];
           // double ymin = bounds[2];
           // double ymax = bounds[3];
           // double zmin = bounds[4];
           // double zmax = bounds[5];

           // // 创建裁剪平面的法向量和截距
           // double planes[24] = {
           //     1, 0, 0, -xmin, // left
           //     -1, 0, 0, xmax, // right
           //     0, 1, 0, -ymin, // bottom
           //     0, -1, 0, ymax, // top
           //     0, 0, 1, -zmin, // near
           //     0, 0, -1, zmax  // far
           // };


           // // 创建vtkPlanes对象，并将裁剪平面的参数设置给它
           // vtkSmartPointer<vtkPlanes> clippingPlanes = vtkSmartPointer<vtkPlanes>::New();
           // clippingPlanes->SetFrustumPlanes(planes);

           // // 使用vtkFrustumSource类创建视锥体，并将裁剪平面设置给它
           // vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
           // frustumSource->SetPlanes(clippingPlanes);
           // frustumSource->Update();


           // vtkPlanes* frustum = frustumSource->GetPlanes();

           // //提前标记几何数据的CellId
           // vtkIdFilter* idFilter = vtkIdFilter::New();
           // vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
           // pcl::io::mesh2vtk(this->mesh, polydata2);
           ///* std::cout << "初始模型点个数： " << polydata2->GetNumberOfPoints() << std::endl;
           // std::cout << "初始模型面片个数： " << polydata2->GetNumberOfCells() << std::endl;*/

           // idFilter->SetInputData(polydata2);
           // // idFilter->SetCellIdsArrayName("OriginalCellId");
           // idFilter->Update();
           // //提取视锥体内的模型
           // vtkExtractPolyDataGeometry* extract = vtkExtractPolyDataGeometry::New();
           // extract->SetInputConnection(idFilter->GetOutputPort());
           // extract->SetImplicitFunction(frustum);
           // extract->Update();
           // if (!extract->GetOutput()->GetPolys())
           // {
           //     std::cout << "faild!" << std::endl;
           //     return;
           // }



           // //创建面片定位器
           // vtkCellLocator* locator = vtkCellLocator::New();
           // locator->SetDataSet(extract->GetOutput());
           // locator->BuildLocator();
           // //----------利用光线投射的方法寻找更靠近摄像机的面片------------

           // double rayStart[3] = { this->camera->eye[0],this->camera->eye[1],this->camera->eye[2] };//光线起点坐标：设置为摄像机位置
           // double rayDirection[3];			//光线方向向量：设置为框选数据包围盒的中心
           // extract->GetOutput()->GetCenter(rayDirection);
           // //std::cout << "center of box : " << rayDirection[0] << " " << rayDirection[1] << " " << rayDirection[2] << std::endl;
           // //std::cout << " ray start " << rayStart[0] << " " << rayStart[1] << " " << rayStart[2] << std::endl;
           // double xyz[3];
           // double t;
           // double pcoords[3];
           // int subId;
           // vtkIdType cellId = -1;			//记录光线击中的面片Id号


           // locator->IntersectWithLine(rayStart, rayDirection, 0.0001, t, xyz, pcoords, subId, cellId);
           // //-----------利用找到的面片获取相连的面
           // vtkPolyDataConnectivityFilter* connectivity = vtkPolyDataConnectivityFilter::New();
           // connectivity->SetInputConnection(extract->GetOutputPort());
           // connectivity->SetExtractionModeToCellSeededRegions();
           // connectivity->InitializeSeedList();
           // connectivity->AddSeed(cellId);
           // connectivity->Update();

           // vtkIdTypeArray* ids = dynamic_cast<vtkIdTypeArray*>(connectivity->GetOutput()->GetCellData()->GetArray(0));
           // polydata2->BuildLinks();
           // if (!ids) return;
           // 
           // for (int i = 0; i < ids->GetNumberOfValues(); i++){
           //     vtkIdType id = ids->GetValue(i);
           //     polydata2->DeleteCell(id);
           // }

           // polydata2->RemoveDeletedCells();
           // polydata2->Modified();

           // pcl::io::vtk2mesh(polydata2, mesh);


            double rayStart[3] = { this->camera->eye[0],this->camera->eye[1],this->camera->eye[2] };//光线起点坐标：
            glDataProc->boxClip(mesh, worldPos, rayStart);
            glDataProc->getRenderData(mesh);
            setImageData(glDataProc->glMeshData);
            repaint();
            m_points.clear();

        }
    }

    if (event->key() == Qt::Key_C)
    {
        idSliceClipped = !idSliceClipped;
        if (!idSliceClipped){

            QPoint p1(m_points[0].x(), m_points[0].y());
            GLdouble wx1, wy1, wz1, wx2, wy2, wz2, wx3, wy3, wz3;
            convScreen2World(p1, wx1, wy1, wz1);
            double a = 1.0, b = 0.0, c = 0.0;       
            glDataProc->getClipPlaneMesh(mesh, a, b, c, QVector3D((float)wx1, (float)wy1, (float)wz1));
            setImageData(glDataProc->glMeshData);
            repaint();
            m_points.clear();

        }
    }

}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() & Qt::Key_Shift) 
        isShiftPressed = false;
}
void MyGLWidget::wheelEvent(QWheelEvent* event) {
    QPoint offset = event->angleDelta();
    if (isShiftPressed){
        brushParam += (offset.y() * SCROLL_SEN);
        if (abs(brushParam) <= 0.0001) brushParam = 0.0f;
        if (brushParam > BRUSH_PARAM_MAX)  brushParam = BRUSH_PARAM_MAX;
        if (brushParam < BRUSH_PARAM_MIN) brushParam = BRUSH_PARAM_MIN;
        brushSize = PARAM_A * brushParam + PARAM_B;
    }else{
        camera->mouseScroll(offset.y());
        repaint();
    }
}
void MyGLWidget::convScreen2World(QPoint screenPoint, GLdouble& wx, GLdouble& wy, GLdouble& wz) {
    int viewport[4] = { 0, 0, SCR_WIDTH, SCR_HEIGHT };
    double mvArray[16], pArray[16];

    QMatrix4x4 mvMat = (camera->getViewMatrix()) * model;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mvArray[i * 4 + j] = mvMat(j, i);
            pArray[i * 4 + j] = proj(j, i);
        }
    }

    GLfloat depth;
    makeCurrent();
    glReadPixels(screenPoint.x(), viewport[3] - screenPoint.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    gluUnProject(screenPoint.x(), viewport[3] - screenPoint.y(), depth, mvArray, pArray, viewport, &wx, &wy, &wz);
}
