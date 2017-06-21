/**
 * @file /include/plan_commander/main_window.hpp
 *
 * @brief Qt based gui for plan_commander.
 *
 * @date November 2010
 **/
#ifndef plan_commander_MAIN_WINDOW_H
#define plan_commander_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QPushButton>
#include <QLineEdit>
/*****************************************************************************
** Namespace
*****************************************************************************/
#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

namespace plan_commander {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);


    /******************************************
    ** Manual connections
    *******************************************/
        void stateButtonClicked();
        void updateposeinfo();
        void updatesegmentationinfo();
        void add_fetch(QModelIndex index);
        void remove_fetch(QModelIndex index);
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	bool isConnected;
	QStringListModel *add_model;
	QStringListModel *remove_model;
	QString buffer_seg;
	int buffer_seg_num;
	QStringList add_List;
	QStringList remove_List;
};

}  // namespace plan_commander

#endif // plan_commander_MAIN_WINDOW_H
