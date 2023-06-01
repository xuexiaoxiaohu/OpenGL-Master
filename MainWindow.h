#pragma once
#include <QMainWindow>
#include "ui_MainWindow.h"
#include <QTimer>
#include "MyGLWidget.h"
#include "NDIDriver.h"
#include "DataProcessing.h"
#include "SurfaceReconsturction.h"
#include <QMutex>
class MainWindow : public QMainWindow{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

public slots:
	void openFile();
	void startRendering();
	void stopRendering();
	void RepaintUI();
	void updateCursor();
	void setGrayValue(int);
	void choosePlyFile();

signals:
	void signal_glUpdate();

private:
	Ui::MainWindowClass ui;
	void addOpengGLWidget();
	void  enclosureDataProcessing();
	NDIDriver* driver;
	MyGLWidget* pointGLWidget, *meshGLWidget;
	bool isRenderRunning;
	SurfaceReconsturction* surface;
	DataProcessing* pointProc, *meshProc;
	bool isPlyFile;
	QString fileName;
};	
