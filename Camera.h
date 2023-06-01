#pragma once
#include <QVector3D>

class Camera{
public:
	Camera(
		QVector3D eye = QVector3D(0.0f, 0.0f, 3.0f),
		QVector3D center = QVector3D(0.0f, 0.0f, 0.0f),
		QVector3D up = QVector3D(0.0f, 1.0f, 0.0f));

	QMatrix4x4 getViewMatrix(); 
	void mouseScroll(float yoffset);

	QVector3D eye,center,up;
	float scrollSen;
};

