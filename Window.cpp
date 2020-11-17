#include <iostream>

#include <QFileDialog>

#include "Window.h"


// constructor for control button
CtrlButton::CtrlButton(QString label, QWidget *parent) 
    : QPushButton(label, parent)
{
    setFixedSize(70, 30);
}

Window::Window(QWidget *parent) 
    : QMainWindow(parent)
{
    // OpenGL widget for BVH viewing
    bvhWidget = new BVHWidget(this);

    // timer widget for animation
    timer = new QTimer(this);
    timer->setInterval(50); // in ms
    // connect the widget to the update slot of an openGL widget
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(incrementFrame()));
    // connect the signal from a loaded BVH file to update the timer
    QObject::connect(bvhWidget, SIGNAL(setTimerInterval(int)), this, SLOT(updateTimerInterval(int)));

    //
    // Window layout
    //

    // add file menu to the main window's menu bar
    fileMenu = menuBar()->addMenu(tr("&File"));
    // create actions for BVH files 
    open = new QAction(tr("&Open"), this);
    open->setStatusTip(tr("Open a BVH file"));
    save = new QAction(tr("&Save"), this);
    save->setStatusTip(tr("Save a BVH file"));
    // connect them to a slot
    // connect open 
    QObject::connect(open, SIGNAL(triggered()), this, SLOT(loadFileDialog()));
    QObject::connect(this, SIGNAL(selectedBVH(QString)), bvhWidget, SLOT(loadBVHFile(QString)));
    // connect save
    // add the actions to the file menu
    fileMenu->addAction(open);
    fileMenu->addAction(save);

    // create the box widget to contain control widgets (play, pause, frame select...)
    ctrlBox = new QGroupBox;
    ctrlLayout = new QGridLayout;
    // create the control widgets
    play = new CtrlButton("&Play", this);
    stop = new CtrlButton("&Stop", this);
    start = new CtrlButton("&Start", this);
    ffwrd = new CtrlButton("&Ffwrd", this);
    rewnd = new CtrlButton("&Rewnd", this);
    end = new CtrlButton("&End", this);
    frames = new QSlider(Qt::Horizontal, this);
    frames->setMaximum(0);
    // add them to the box widget 
    ctrlLayout->addWidget(start, 0, 0);
    ctrlLayout->addWidget(rewnd, 0, 1);
    ctrlLayout->addWidget(play, 0, 2);
    ctrlLayout->addWidget(stop, 0, 3);
    ctrlLayout->addWidget(ffwrd, 0, 4);
    ctrlLayout->addWidget(end, 0, 5);
    ctrlLayout->addWidget(frames, 1, 0, 1, 6);
    // connect the control widgets
    // connect start to load first frame
    QObject::connect(start, SIGNAL(pressed()), this, SLOT(goToFirstFrame()));
    // connect play to start playing the animation (start the timer)
    QObject::connect(play, SIGNAL(pressed()), timer, SLOT(start()));
    // connect stop to pause playing the animation
    QObject::connect(stop, SIGNAL(pressed()), timer, SLOT(stop()));
    // connect ffwrd to increase animation play speed
    QObject::connect(ffwrd, SIGNAL(pressed()), bvhWidget, SLOT(changePlayDir()));
    // connect rewnd to decrease animation play speed
    QObject::connect(ffwrd, SIGNAL(pressed()), bvhWidget, SLOT(changePlayDir()));
    // connect end to load last frame
    QObject::connect(end, SIGNAL(pressed()), this, SLOT(goToLastFrame()));
    // connect slider to load a new frame on value change (must change value range upon loading a BVH file)
    QObject::connect(bvhWidget, SIGNAL(setMaxFrame(int)), this, SLOT(updateFrameSlider(int)));
    // the slider controls the animation so connect it to the animate slot
    QObject::connect(frames, SIGNAL(valueChanged(int)), bvhWidget, SLOT(animateFigure(int)));
    ctrlBox->setLayout(ctrlLayout);
    // create the dock widget containing the controls
    ctrlDock = new QDockWidget(tr("Animation Controls"), this);
    ctrlDock->setAllowedAreas(Qt::BottomDockWidgetArea);
    ctrlDock->setWidget(ctrlBox);
    //ctrlDock->setFeatures(QDockWidget::DockWidgetMovable  QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::BottomDockWidgetArea, ctrlDock);

    // central widget where we render
    setCentralWidget(bvhWidget);

    // initialise the status bar message, also creates it
    statusBar()->showMessage(tr("Ready"));
}

// delete all widgets
Window::~Window()
{
    // there must be a better way of doing this...
    delete bvhWidget;
    delete fileMenu;
    delete open;
    delete save;
    delete ctrlDock;
    delete ctrlBox;
    delete ctrlLayout;
    delete play;
    delete stop;
    delete ffwrd;
    delete rewnd;
    delete end;
    delete frames;
    delete timer;
}

// opens up a file browser dialog and emits a signal to the GL widget if a BVH file is chosen
void Window::loadFileDialog()
{
    // open dialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("&Load BVH"), "./", tr("BVH (*.bvh)"));
    
    // check name chosen is not empty
    if (!fileName.isEmpty())
        emit selectedBVH(fileName);
}

// slot that set the slider's maximum to the number of frames in a loaded BVH file
void Window::updateFrameSlider(int nFrames)
{
    goToFirstFrame();
    frames->setMaximum(nFrames);
}

// slot that updates the timer's time interval 
void Window::updateTimerInterval(int interval)
{
    timer->setInterval(interval);
}

// connected to the timer timeout signal, increments the slider value by 1
void Window::incrementFrame()
{
    // get current value in slider
    int curr = frames->value();
    // set the slider to current + the play direction, automatically clamped
    frames->setValue(curr + bvhWidget->playDir);
}

// reset the slider to initial value
void Window::goToFirstFrame()
{
    frames->setValue(frames->minimum());
}

// set the slider to maximum value
void Window::goToLastFrame()
{
    frames->setValue(frames->maximum());
}

