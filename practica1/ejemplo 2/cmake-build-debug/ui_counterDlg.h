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
#include <QtWidgets/QSplitter>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Counter
{
public:
    QLCDNumber *lcdNumber;
    QLineEdit *pantalla;
    QSplitter *splitter_2;
    QPushButton *getPeriodo;
    QSplitter *splitter;
    QPushButton *setPeriodo;
    QPushButton *button;
    QPushButton *getTimeElapsed;

    void setupUi(QWidget *Counter)
    {
        if (Counter->objectName().isEmpty())
            Counter->setObjectName(QString::fromUtf8("Counter"));
        Counter->resize(410, 314);
        lcdNumber = new QLCDNumber(Counter);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setGeometry(QRect(50, 40, 301, 91));
        pantalla = new QLineEdit(Counter);
        pantalla->setObjectName(QString::fromUtf8("pantalla"));
        pantalla->setGeometry(QRect(50, 160, 311, 51));
        splitter_2 = new QSplitter(Counter);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setGeometry(QRect(10, 230, 381, 61));
        splitter_2->setOrientation(Qt::Horizontal);
        getPeriodo = new QPushButton(splitter_2);
        getPeriodo->setObjectName(QString::fromUtf8("getPeriodo"));
        splitter_2->addWidget(getPeriodo);
        splitter = new QSplitter(splitter_2);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Vertical);
        setPeriodo = new QPushButton(splitter);
        setPeriodo->setObjectName(QString::fromUtf8("setPeriodo"));
        splitter->addWidget(setPeriodo);
        button = new QPushButton(splitter);
        button->setObjectName(QString::fromUtf8("button"));
        splitter->addWidget(button);
        splitter_2->addWidget(splitter);
        getTimeElapsed = new QPushButton(splitter_2);
        getTimeElapsed->setObjectName(QString::fromUtf8("getTimeElapsed"));
        splitter_2->addWidget(getTimeElapsed);

        retranslateUi(Counter);

        QMetaObject::connectSlotsByName(Counter);
    } // setupUi

    void retranslateUi(QWidget *Counter)
    {
        Counter->setWindowTitle(QApplication::translate("Counter", "Counter", nullptr));
        getPeriodo->setText(QApplication::translate("Counter", "Obtener per\303\255odo", nullptr));
        setPeriodo->setText(QApplication::translate("Counter", "Cambio per\303\255odo", nullptr));
        button->setText(QApplication::translate("Counter", "STOP", nullptr));
        getTimeElapsed->setText(QApplication::translate("Counter", "Tiempo ejecuci\303\263n", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Counter: public Ui_Counter {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COUNTERDLG_H
