/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pushButton_LaserBeam;
    QPushButton *pushButton_LaserPlane;
    QPushButton *pushButton_LaserVerification;
    QGroupBox *groupBox_Laser;
    QRadioButton *radioButton_L1;
    QRadioButton *radioButton_L2;
    QRadioButton *radioButton_L3;
    QRadioButton *radioButton_L4;
    QGroupBox *groupBox_Distance;
    QRadioButton *radioButton_D1;
    QRadioButton *radioButton_D2;
    QGroupBox *groupBox_SerialNum;
    QLineEdit *lineEdit_L4;
    QLineEdit *lineEdit_L3;
    QLineEdit *lineEdit_L2;
    QLineEdit *lineEdit_L1;
    QGroupBox *groupBox_SerialNum_2;
    QLineEdit *lineEdit_F4;
    QLineEdit *lineEdit_F3;
    QLineEdit *lineEdit_F2;
    QLineEdit *lineEdit_F1;
    QMenuBar *menuBar;
    QMenu *menuLaser_Alignment_Tool;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(832, 379);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButton_LaserBeam = new QPushButton(centralWidget);
        pushButton_LaserBeam->setObjectName(QStringLiteral("pushButton_LaserBeam"));
        pushButton_LaserBeam->setGeometry(QRect(70, 250, 190, 40));
        QFont font;
        font.setPointSize(12);
        pushButton_LaserBeam->setFont(font);
        pushButton_LaserPlane = new QPushButton(centralWidget);
        pushButton_LaserPlane->setObjectName(QStringLiteral("pushButton_LaserPlane"));
        pushButton_LaserPlane->setGeometry(QRect(290, 250, 190, 40));
        pushButton_LaserPlane->setFont(font);
        pushButton_LaserVerification = new QPushButton(centralWidget);
        pushButton_LaserVerification->setObjectName(QStringLiteral("pushButton_LaserVerification"));
        pushButton_LaserVerification->setEnabled(true);
        pushButton_LaserVerification->setGeometry(QRect(640, 250, 150, 40));
        pushButton_LaserVerification->setFont(font);
        groupBox_Laser = new QGroupBox(centralWidget);
        groupBox_Laser->setObjectName(QStringLiteral("groupBox_Laser"));
        groupBox_Laser->setGeometry(QRect(70, 20, 180, 210));
        groupBox_Laser->setFont(font);
        radioButton_L1 = new QRadioButton(groupBox_Laser);
        radioButton_L1->setObjectName(QStringLiteral("radioButton_L1"));
        radioButton_L1->setGeometry(QRect(10, 40, 100, 21));
        radioButton_L2 = new QRadioButton(groupBox_Laser);
        radioButton_L2->setObjectName(QStringLiteral("radioButton_L2"));
        radioButton_L2->setGeometry(QRect(10, 80, 100, 21));
        radioButton_L3 = new QRadioButton(groupBox_Laser);
        radioButton_L3->setObjectName(QStringLiteral("radioButton_L3"));
        radioButton_L3->setGeometry(QRect(10, 120, 100, 21));
        radioButton_L4 = new QRadioButton(groupBox_Laser);
        radioButton_L4->setObjectName(QStringLiteral("radioButton_L4"));
        radioButton_L4->setGeometry(QRect(10, 160, 100, 21));
        groupBox_Distance = new QGroupBox(centralWidget);
        groupBox_Distance->setObjectName(QStringLiteral("groupBox_Distance"));
        groupBox_Distance->setGeometry(QRect(640, 20, 150, 121));
        groupBox_Distance->setFont(font);
        radioButton_D1 = new QRadioButton(groupBox_Distance);
        radioButton_D1->setObjectName(QStringLiteral("radioButton_D1"));
        radioButton_D1->setGeometry(QRect(20, 40, 100, 21));
        radioButton_D2 = new QRadioButton(groupBox_Distance);
        radioButton_D2->setObjectName(QStringLiteral("radioButton_D2"));
        radioButton_D2->setGeometry(QRect(20, 80, 100, 21));
        groupBox_SerialNum = new QGroupBox(centralWidget);
        groupBox_SerialNum->setObjectName(QStringLiteral("groupBox_SerialNum"));
        groupBox_SerialNum->setGeometry(QRect(250, 20, 210, 210));
        groupBox_SerialNum->setFont(font);
        lineEdit_L4 = new QLineEdit(groupBox_SerialNum);
        lineEdit_L4->setObjectName(QStringLiteral("lineEdit_L4"));
        lineEdit_L4->setGeometry(QRect(30, 160, 150, 25));
        lineEdit_L3 = new QLineEdit(groupBox_SerialNum);
        lineEdit_L3->setObjectName(QStringLiteral("lineEdit_L3"));
        lineEdit_L3->setGeometry(QRect(30, 120, 150, 25));
        lineEdit_L2 = new QLineEdit(groupBox_SerialNum);
        lineEdit_L2->setObjectName(QStringLiteral("lineEdit_L2"));
        lineEdit_L2->setGeometry(QRect(30, 80, 150, 25));
        lineEdit_L1 = new QLineEdit(groupBox_SerialNum);
        lineEdit_L1->setObjectName(QStringLiteral("lineEdit_L1"));
        lineEdit_L1->setGeometry(QRect(30, 40, 150, 25));
        groupBox_SerialNum_2 = new QGroupBox(centralWidget);
        groupBox_SerialNum_2->setObjectName(QStringLiteral("groupBox_SerialNum_2"));
        groupBox_SerialNum_2->setGeometry(QRect(460, 20, 131, 210));
        groupBox_SerialNum_2->setFont(font);
        lineEdit_F4 = new QLineEdit(groupBox_SerialNum_2);
        lineEdit_F4->setObjectName(QStringLiteral("lineEdit_F4"));
        lineEdit_F4->setGeometry(QRect(30, 160, 71, 25));
        lineEdit_F3 = new QLineEdit(groupBox_SerialNum_2);
        lineEdit_F3->setObjectName(QStringLiteral("lineEdit_F3"));
        lineEdit_F3->setGeometry(QRect(30, 120, 71, 25));
        lineEdit_F2 = new QLineEdit(groupBox_SerialNum_2);
        lineEdit_F2->setObjectName(QStringLiteral("lineEdit_F2"));
        lineEdit_F2->setGeometry(QRect(30, 80, 71, 25));
        lineEdit_F1 = new QLineEdit(groupBox_SerialNum_2);
        lineEdit_F1->setObjectName(QStringLiteral("lineEdit_F1"));
        lineEdit_F1->setGeometry(QRect(30, 40, 71, 25));
        MainWindow->setCentralWidget(centralWidget);
        pushButton_LaserBeam->raise();
        pushButton_LaserPlane->raise();
        groupBox_Laser->raise();
        groupBox_Distance->raise();
        groupBox_SerialNum->raise();
        groupBox_SerialNum_2->raise();
        pushButton_LaserVerification->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setEnabled(true);
        menuBar->setGeometry(QRect(0, 0, 832, 20));
        menuLaser_Alignment_Tool = new QMenu(menuBar);
        menuLaser_Alignment_Tool->setObjectName(QStringLiteral("menuLaser_Alignment_Tool"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuLaser_Alignment_Tool->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        pushButton_LaserBeam->setText(QApplication::translate("MainWindow", "LaserBeamAlignment", Q_NULLPTR));
        pushButton_LaserPlane->setText(QApplication::translate("MainWindow", "LaserPlaneAlignment", Q_NULLPTR));
        pushButton_LaserVerification->setText(QApplication::translate("MainWindow", "LaserVerification", Q_NULLPTR));
        groupBox_Laser->setTitle(QApplication::translate("MainWindow", "Laser Select", Q_NULLPTR));
        radioButton_L1->setText(QApplication::translate("MainWindow", "Laser1", Q_NULLPTR));
        radioButton_L2->setText(QApplication::translate("MainWindow", "Laser2", Q_NULLPTR));
        radioButton_L3->setText(QApplication::translate("MainWindow", "Laser3", Q_NULLPTR));
        radioButton_L4->setText(QApplication::translate("MainWindow", "Laser4", Q_NULLPTR));
        groupBox_Distance->setTitle(QApplication::translate("MainWindow", "Distance Select", Q_NULLPTR));
        radioButton_D1->setText(QApplication::translate("MainWindow", "D1", Q_NULLPTR));
        radioButton_D2->setText(QApplication::translate("MainWindow", "D2", Q_NULLPTR));
        groupBox_SerialNum->setTitle(QApplication::translate("MainWindow", "Laser Serial Number", Q_NULLPTR));
        groupBox_SerialNum_2->setTitle(QApplication::translate("MainWindow", "Fixture Number", Q_NULLPTR));
        menuLaser_Alignment_Tool->setTitle(QApplication::translate("MainWindow", "Laser Assembly Tool", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
