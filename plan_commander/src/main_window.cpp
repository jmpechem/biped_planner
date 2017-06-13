/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/plan_commander/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace plan_commander {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
	, isConnected(false)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(poseInfoUpdated()),this, SLOT(updateposeinfo()));


    QObject::connect(ui.button_pcd,  SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_online,  SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_heightcut,  SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_noisecancel,  SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_gridmap,  SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_obstacle_finder, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_obstacle_potential_field, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_step_detect, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_plan_root, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_spline_root, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));
    QObject::connect(ui.button_footstep, SIGNAL(clicked()),this,SLOT(stateButtonClicked()));

    ui.edit_spline_deg->setText("3");
    ui.edit_spline_nodenum->setText("5");
    ui.edit_spline_noderes->setText("1000");


    ui.edit_heightlimit->setText("1");
    ui.edit_filtersize->setText("0.03");
    ui.edit_gridsize->setText("0.05");

    ui.edit_slope->setText("10");
    ui.edit_relstep->setText("0.1");
    ui.edit_obstacle_radius->setText("0.1");

    ui.edit_seg_num->setText("3");
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			isConnected = true;
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
			isConnected = true;
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "plan_commander");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "plan_commander");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::stateButtonClicked(){
   QString objName = sender()->objectName();
   std::string state;
   int id = 0;
   float val[3] = {0.0,0.0,0.0};
   if(objName.compare("button_pcd")==0){
        state = "load_pcd";        
     }
   else if(objName.compare("button_online")==0){
        state = "load_onlinedata";
     }
   else if(objName.compare("button_heightcut")==0){
        state = "height_cut";
        QString tmp = ui.edit_heightlimit->text();
        val[0] = tmp.toFloat();
     }
   else if(objName.compare("button_noisecancel")==0){
        state = "noise_preprocessing";
        QString tmp = ui.edit_filtersize->text();
        val[0] = tmp.toFloat();
     }
   else if(objName.compare("button_gridmap")==0){
        state = "grid_map";
        QString tmp = ui.edit_gridsize->text();
        val[0] = tmp.toFloat();
     }
   else if(objName.compare("button_obstacle_finder")==0){
       state = "find_obstacle";
       QString tmp = ui.edit_slope->text();
       val[0] = tmp.toFloat();
       tmp = ui.edit_relstep->text();
       val[1] = tmp.toFloat();
     }
   else if(objName.compare("button_obstacle_potential_field")==0){
       state = "obstacle_potential_field";
       QString tmp = ui.edit_obstacle_radius->text();
       val[0] = tmp.toFloat();
       tmp = ui.edit_gridsize->text();
       val[1] = tmp.toFloat();
     }
   else if(objName.compare("button_step_detect")==0){
       state = "step_detect";
       QString tmp = ui.edit_seg_num->text();
       id = tmp.toInt();
     }
   else if(objName.compare("button_plan_root")==0){
       state = "plan_root";
     }
   else if(objName.compare("button_spline_root")==0){
       state = "spline_root";
       QString tmp = ui.edit_spline_deg->text();
       val[0] = tmp.toFloat();
       tmp = ui.edit_spline_nodenum->text();
       val[1] = tmp.toFloat();
       tmp = ui.edit_spline_noderes->text();
       val[2] = tmp.toFloat();
     }
   else if(objName.compare("button_footstep")==0){
       state = "footstep_plan";
     }


   qnode.send_transition(state,id,val[0],val[1],val[2]);

}
void MainWindow::updateposeinfo()
{
  if(qnode.pose_recv_msg.id == 0 && qnode.pose_recv_msg.state == "init_pose")
  {
      ui.edit_init_pose->setText("(" + QString::number(qnode.pose_recv_msg.val1,'f',2) + ", " + QString::number(qnode.pose_recv_msg.val2,'f',2) + ", " + QString::number(rad2deg(qnode.pose_recv_msg.val3),'f',2) + ")");
  }
  else if(qnode.pose_recv_msg.id == 1  && qnode.pose_recv_msg.state == "goal_pose")
  {
      ui.edit_goal_pose->setText("(" + QString::number(qnode.pose_recv_msg.val1,'f',2) + ", " + QString::number(qnode.pose_recv_msg.val2,'f',2) + ", " + QString::number(rad2deg(qnode.pose_recv_msg.val3),'f',2) + ")");
  }
  else if(qnode.pose_recv_msg.id == 2 && qnode.pose_recv_msg.state == "init_pose") // for localization mode(initial point updated mode)
  {
      ui.edit_init_pose->setText("(" + QString::number(qnode.pose_recv_msg.val1,'f',2) + ", " + QString::number(qnode.pose_recv_msg.val2,'f',2) + ", " + QString::number(rad2deg(qnode.pose_recv_msg.val3),'f',2) + ")");
  }

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace plan_commander

