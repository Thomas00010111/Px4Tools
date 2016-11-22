#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_mainwindow.h"
#include "mainapplication.h"
#include "vicon.h"

class MainApplication;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setController(MainApplication *controller);
    void setViconController(Vicon *controller);

private slots:
//    void on_SendPositionBtn_clicked();

    void on_stopBtn_clicked();

    void on_ExitBtn_clicked();

//    void on_sendPositionBtn_clicked();

    void on_stopSendingPosition_clicked();

    void on_startloop_clicked();

    void on_sendMocapBtn_toggled(bool checked);

    void on_sendMocapBtn_clicked();

    void on_SendPositionBtn_toggled(bool checked);

    void on_sendPositionBtn_toggled(bool checked);

    void closeEvent (QCloseEvent *);

    void on_pushButton_toggled(bool checked);

    void on_map2PositionBtn_toggled(bool checked);

    void on_btnGazebo2VisionEstimate_toggled(bool checked);

    void on_vicon2VisionBtn_toggled(bool checked);

    void on_setIpViconBtn_clicked();

    void on_orbSlam2Btn_toggled(bool checked);

    void on_StartSequenceButton_toggled(bool checked);

    void on_killButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    MainApplication *controller;
    Vicon *vicon_controller;
    void setColorButton(QPushButton *button, bool checked);
    void closeMainForm();
};

#endif // MAINWINDOW_H


