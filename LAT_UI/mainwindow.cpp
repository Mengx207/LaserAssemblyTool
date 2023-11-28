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
    QString version;
    if (ui->radioButton_V3->isChecked())
    {
        version = "V3";
    }
    else if (ui->radioButton_V4->isChecked())
    {
        version = "V4";
    }
    QByteArray ba_V = version.toLocal8Bit();
    const char *c_str_V = ba_V.data();

    if (ui->radioButton_L1->isChecked())
    {
        QString serialnum_L1 = ui->lineEdit_L1->QLineEdit::text();
        QByteArray ba_L1 = serialnum_L1.toLocal8Bit();
        const char *c_str_L1 = ba_L1.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 1 %s d1",c_str_V,c_str_L1);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 1 %s d2",c_str_V,c_str_L1);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 1",c_str_V);
            system(buf);
        }
        qDebug() << "Laser1 Beam";
    }

    if (ui->radioButton_L2->isChecked())
    {
        QString serialnum_L2 = ui->lineEdit_L2->QLineEdit::text();
        QByteArray ba_L2 = serialnum_L2.toLocal8Bit();
        const char *c_str_L2 = ba_L2.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 2 %s d1",c_str_V,c_str_L2);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 2 %s d2",c_str_V,c_str_L2);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 2 %s",c_str_V,c_str_L2);
            system(buf);
        }
        qDebug() << "Laser2 Beam";
    }

    if (ui->radioButton_L3->isChecked())
    {
        QString serialnum_L3 = ui->lineEdit_L3->QLineEdit::text();
        QByteArray ba_L3 = serialnum_L3.toLocal8Bit();
        const char *c_str_L3 = ba_L3.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 3 %s d1",c_str_V,c_str_L3);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 3 %s d2",c_str_V,c_str_L3);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 3 %s",c_str_V,c_str_L3);
            system(buf);
        }
        qDebug() << "Laser3 Beam";
    }

    if (ui->radioButton_L4->isChecked())
    {
        QString serialnum_L4 = ui->lineEdit_L4->QLineEdit::text();
        QByteArray ba_L4 = serialnum_L4.toLocal8Bit();
        const char *c_str_L4 = ba_L4.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 4 %s d1",c_str_V,c_str_L4);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 4 %s d2",c_str_V,c_str_L4);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserbeamguide %s 4 %s",c_str_V,c_str_L4);
            system(buf);
        }
        qDebug() << "Laser4 Beam";
    }

}

void MainWindow::on_pushButton_LaserPlane_clicked()
{
    char buf[128];

    QString version;
    if (ui->radioButton_V3->isChecked())
    {
        version = "V3";
    }
    else if (ui->radioButton_V4->isChecked())
    {
        version = "V4";
    }
    QByteArray ba_V = version.toLocal8Bit();
    const char *c_str_V = ba_V.data();

    if (ui->radioButton_L1->isChecked())
    {
        QString serialnum_L1 = ui->lineEdit_L1->QLineEdit::text();
        QByteArray ba_L1 = serialnum_L1.toLocal8Bit();
        const char *c_str_L1 = ba_L1.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 1 %s d1",c_str_V,c_str_L1);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 1 %s d2",c_str_V,c_str_L1);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 1 %s",c_str_V,c_str_L1);
            system(buf);
        }
        qDebug() << "Laser1 Beam";
    }
    if (ui->radioButton_L2->isChecked())
    {
        QString serialnum_L2 = ui->lineEdit_L2->QLineEdit::text();
        QByteArray ba_L2 = serialnum_L2.toLocal8Bit();
        const char *c_str_L2 = ba_L2.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 2 %s d1",c_str_V,c_str_L2);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 2 %s d2",c_str_V,c_str_L2);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 2 %s",c_str_V,c_str_L2);
            system(buf);
        }
        qDebug() << "Laser2 Beam";
    }
    if (ui->radioButton_L3->isChecked())
    {
        QString serialnum_L3 = ui->lineEdit_L3->QLineEdit::text();
        QByteArray ba_L3 = serialnum_L3.toLocal8Bit();
        const char *c_str_L3 = ba_L3.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 3 %s d1",c_str_V,c_str_L3);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 3 %s d2",c_str_V,c_str_L3);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 3 %s",c_str_V,c_str_L3);
            system(buf);
        }
        qDebug() << "Laser3 Beam";
    }
    if (ui->radioButton_L4->isChecked())
    {
        QString serialnum_L4 = ui->lineEdit_L4->QLineEdit::text();
        QByteArray ba_L4 = serialnum_L4.toLocal8Bit();
        const char *c_str_L4 = ba_L4.data();

        if(ui->radioButton_D1->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 4 %s d1",c_str_V,c_str_L4);
            system(buf);
        }
        else if(ui->radioButton_D2->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 4 %s d2",c_str_V,c_str_L4);
            system(buf);
        }
        else
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserplaneguide %s 4 %s",c_str_V,c_str_L4);
            system(buf);
        }
        qDebug() << "Laser4 Beam";
    }

}

void MainWindow::on_pushButton_LaserVerification_clicked()
{
    char buf[128];

    if (ui->radioButton_L1->isChecked())
    {
        QString serialnum_L1 = ui->lineEdit_L1->QLineEdit::text();
        QByteArray ba_L1 = serialnum_L1.toLocal8Bit();
        const char *c_str_L1 = ba_L1.data();

        if(ui->radioButton_V3->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V3 1 %s",c_str_L1);
            system(buf);
        }
        else if(ui->radioButton_V4->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V4 1 %s",c_str_L1);
            system(buf);
        }
    }
    else if (ui->radioButton_L2->isChecked())
    {
        QString serialnum_L2 = ui->lineEdit_L2->QLineEdit::text();
        QByteArray ba_L2 = serialnum_L2.toLocal8Bit();
        const char *c_str_L2 = ba_L2.data();

        if(ui->radioButton_V3->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V3 2 %s",c_str_L2);
            system(buf);
        }
        else if(ui->radioButton_V4->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V4 2 %s",c_str_L2);
            system(buf);
        }
    }
    else if (ui->radioButton_L3->isChecked())
    {
        QString serialnum_L3 = ui->lineEdit_L3->QLineEdit::text();
        QByteArray ba_L3 = serialnum_L3.toLocal8Bit();
        const char *c_str_L3 = ba_L3.data();

        if(ui->radioButton_V3->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V3 3 %s",c_str_L3);
            system(buf);
        }
        else if(ui->radioButton_V4->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V4 3 %s",c_str_L3);
            system(buf);
        }
    }
    else if (ui->radioButton_L4->isChecked())
    {
        QString serialnum_L4 = ui->lineEdit_L4->QLineEdit::text();
        QByteArray ba_L4 = serialnum_L4.toLocal8Bit();
        const char *c_str_L4 = ba_L4.data();

        if(ui->radioButton_V3->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V3 4 %s",c_str_L4);
            system(buf);
        }
        else if(ui->radioButton_V4->isChecked())
        {
            snprintf(buf,sizeof(buf),"cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && ./laserverification V4 4 %s",c_str_L4);
            system(buf);
        }
    }
}
