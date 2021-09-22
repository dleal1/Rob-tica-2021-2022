/********************************************************************************
** Form generated from reading UI file 'counterDlg.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COUNTERDLG_H
#define UI_COUNTERDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QLCDNumber *lcdNumber;
    QPushButton *getPeriodo;
    QPushButton *setPeriodo;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *getTimeElapsed;
    QPushButton *button;
    QLineEdit *pantalla;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName(QString::fromUtf8("Counter"));
        Counter->resize(439, 315);
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setGeometry(QRect(50, 40, 301, 91));
        getPeriodo = new QPushButton(Counter);
        getPeriodo->setObjectName(QString::fromUtf8("getPeriodo"));
        getPeriodo->setGeometry(QRect(20, 220, 121, 61));
        setPeriodo = new QPushButton(Counter);
        setPeriodo->setObjectName(QString::fromUtf8("setPeriodo"));
        setPeriodo->setGeometry(QRect(290, 220, 121, 61));
        layoutWidget = new QWidget(Counter);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(150, 220, 131, 61));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        getTimeElapsed = new QPushButton(layoutWidget);
        getTimeElapsed->setObjectName(QString::fromUtf8("getTimeElapsed"));

        verticalLayout->addWidget(getTimeElapsed);

        button = new QPushButton(layoutWidget);
        button->setObjectName(QString::fromUtf8("button"));

        verticalLayout->addWidget(button);

        pantalla = new QLineEdit(Counter);
        pantalla->setObjectName(QString::fromUtf8("pantalla"));
        pantalla->setGeometry(QRect(50, 150, 301, 51));

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QApplication::translate("Counter", "Counter", nullptr));
        getPeriodo->setText(QApplication::translate("Counter", "Obtener Periodo", nullptr));
        setPeriodo->setText(QApplication::translate("Counter", "Cambio Periodo", nullptr));
        getTimeElapsed->setText(QApplication::translate("Counter", "Tiempo ejecucion", nullptr));
        button->setText(QApplication::translate("Counter", "STOP", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
