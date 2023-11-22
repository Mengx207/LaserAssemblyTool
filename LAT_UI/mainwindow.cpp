#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QScreen>
#include <QWidget>
#include <QApplication>
#include <iostream>
#include <stdlib.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    move(QGuiApplication::screens().at(0)->geometry().bottomRight() - frameGeometry().bottomRight());
//    move(200,200);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_LaserBeam_clicked()
{
    char buf[128];

    if (ui->radioButton_L1->isChecked())
    {
        QString serialnum_L1 = ui->lineEdit_L1->QLineEdit::text();
        QByteArray ba_L1 = serialnum_L1.toLocal8Bit();
        const char *c_str_L1 = ba_L1.data();

        QString serialnum_F1 = ui->lineEdit_F1->QLineEdit::text();
        QByteArray ba_F1 = serialnum_F1.toLocal8Bit();
        const char *c_str_F1 = ba_F1.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 1 %s d1",c_str_L1);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 1 %s d2",c_str_L1);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 1 %s %s",c_str_L1,c_str_F1);
            system(buf);
        }
        qDebug() << "Laser1 Beam";
    }
    if (ui->radioButton_L2->isChecked())
    {
        QString serialnum_L2 = ui->lineEdit_L2->QLineEdit::text();
        QByteArray ba_L2 = serialnum_L2.toLocal8Bit();
        const char *c_str_L2 = ba_L2.data();

        QString serialnum_F2 = ui->lineEdit_F2->QLineEdit::text();
        QByteArray ba_F2 = serialnum_F2.toLocal8Bit();
        const char *c_str_F2 = ba_F2.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 2 %s d1",c_str_L2);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 2 %s d2",c_str_L2);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 2 %s %s",c_str_L2,c_str_F2);
            system(buf);
        }
        qDebug() << "Laser2 Beam";
    }
    if (ui->radioButton_L3->isChecked())
    {
        QString serialnum_L3 = ui->lineEdit_L3->QLineEdit::text();
        QByteArray ba_L3 = serialnum_L3.toLocal8Bit();
        const char *c_str_L3 = ba_L3.data();

        QString serialnum_F3 = ui->lineEdit_F3->QLineEdit::text();
        QByteArray ba_F3 = serialnum_F3.toLocal8Bit();
        const char *c_str_F3 = ba_F3.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 3 %s d1",c_str_L3);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 3 %s d2",c_str_L3);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 3 %s %s",c_str_L3,c_str_F3);
            system(buf);
        }
        qDebug() << "Laser3 Beam";
    }
    if (ui->radioButton_L4->isChecked())
    {
        QString serialnum_L4 = ui->lineEdit_L4->QLineEdit::text();
        QByteArray ba_L4 = serialnum_L4.toLocal8Bit();
        const char *c_str_L4 = ba_L4.data();

        QString serialnum_F4 = ui->lineEdit_F4->QLineEdit::text();
        QByteArray ba_F4 = serialnum_F4.toLocal8Bit();
        const char *c_str_F4 = ba_F4.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 4 %s d1",c_str_L4);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 4 %s d2",c_str_L4);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide 4 %s %s",c_str_L4,c_str_F4);
            system(buf);
        }
        qDebug() << "Laser4 Beam";
    }

}

void MainWindow::on_pushButton_LaserPlane_clicked()
{
    char buf[128];

    if (ui->radioButton_L1->isChecked())
    {
        QString serialnum_L1 = ui->lineEdit_L1->QLineEdit::text();
        QByteArray ba_L1 = serialnum_L1.toLocal8Bit();
        const char *c_str_L1 = ba_L1.data();

        QString serialnum_F1 = ui->lineEdit_F1->QLineEdit::text();
        QByteArray ba_F1 = serialnum_F1.toLocal8Bit();
        const char *c_str_F1 = ba_F1.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 1 %s d1",c_str_L1);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 1 %s d2",c_str_L1);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 1 %s %s",c_str_L1,c_str_F1);
            system(buf);
        }
        qDebug() << "Laser1 Beam";
    }
    if (ui->radioButton_L2->isChecked())
    {
        QString serialnum_L2 = ui->lineEdit_L2->QLineEdit::text();
        QByteArray ba_L2 = serialnum_L2.toLocal8Bit();
        const char *c_str_L2 = ba_L2.data();

        QString serialnum_F2 = ui->lineEdit_F2->QLineEdit::text();
        QByteArray ba_F2 = serialnum_F2.toLocal8Bit();
        const char *c_str_F2 = ba_F2.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 2 %s d1",c_str_L2);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 2 %s d2",c_str_L2);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 2 %s %s",c_str_L2,c_str_F2);
            system(buf);
        }
        qDebug() << "Laser2 Beam";
    }
    if (ui->radioButton_L3->isChecked())
    {
        QString serialnum_L3 = ui->lineEdit_L3->QLineEdit::text();
        QByteArray ba_L3 = serialnum_L3.toLocal8Bit();
        const char *c_str_L3 = ba_L3.data();

        QString serialnum_F3 = ui->lineEdit_F3->QLineEdit::text();
        QByteArray ba_F3 = serialnum_F3.toLocal8Bit();
        const char *c_str_F3 = ba_F3.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 3 %s d1",c_str_L3);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 3 %s d2",c_str_L3);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 3 %s %s",c_str_L3,c_str_F3);
            system(buf);
        }
        qDebug() << "Laser3 Beam";
    }
    if (ui->radioButton_L4->isChecked())
    {
        QString serialnum_L4 = ui->lineEdit_L4->QLineEdit::text();
        QByteArray ba_L4 = serialnum_L4.toLocal8Bit();
        const char *c_str_L4 = ba_L4.data();

        QString serialnum_F4 = ui->lineEdit_F4->QLineEdit::text();
        QByteArray ba_F4 = serialnum_F4.toLocal8Bit();
        const char *c_str_F4 = ba_F4.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 4 %s d1",c_str_L4);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 4 %s d2",c_str_L4);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide 4 %s %s",c_str_L4,c_str_F4);
            system(buf);
        }
        qDebug() << "Laser4 Beam";
    }

}

void MainWindow::on_pushButton_LaserVerification_clicked()
{
    if (ui->radioButton_L1->isChecked())
    {system("cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification 1");}
    else if (ui->radioButton_L2->isChecked())
    {system("cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification 2");}
    else if (ui->radioButton_L3->isChecked())
    {system("cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification 3");}
    else if (ui->radioButton_L4->isChecked())
    {system("cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification 4");}
}
