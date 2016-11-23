#ifndef DASHBOARD_WINDOW_H
#define DASHBOARD_WINDOW_H
#include <QtWidgets>
#include <QPushButton>
#include <QString>
#include <QGridLayout>
#include <QLabel>

#include <QtCharts/QChartView>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QValueAxis>


class Dashboard : public QWidget
{
    Q_OBJECT

public:
    Dashboard(int argc, char** argv, QWidget *parent = 0);

private:

    QtCharts::QBarSet *l1Snr;
    QtCharts::QBarSet *l2Snr;

    QtCharts::QBarSeries *snrBars;
    QtCharts::QChart *chart;
    QtCharts::QChartView *chartView;

    QLabel *label_nav_newPosition;
    QLabel *label_nav_newClockThisSolution;
    QLabel *label_nav_newHorizontalThisSolution;
    QLabel *label_nav_newHeightThisSolution;
    QLabel *label_nav_usesLeastSquares;
    QLabel *label_nav_usesFilteredL1;
    QLabel *label_nav_isDifferential;
    QLabel *label_nav_isPhase;
    QLabel *label_nav_isFixedInteger;
    QLabel *label_nav_isOmnistar;
    QLabel *label_nav_isStatic;
    QLabel *label_nav_isNetworkRTK;
    QLabel *label_nav_isLocationRTK;
    QLabel *label_nav_isBeaconGPS;
    QLabel *label_nav_initCounter;

    QLabel *label_nav_newPosition_value;
    QLabel *label_nav_newClockThisSolution_value;
    QLabel *label_nav_newHorizontalThisSolution_value;
    QLabel *label_nav_newHeightThisSolution_value;
    QLabel *label_nav_usesLeastSquares_value;
    QLabel *label_nav_usesFilteredL1_value;
    QLabel *label_nav_isDifferential_value;
    QLabel *label_nav_isPhase_value;
    QLabel *label_nav_isFixedInteger_value;
    QLabel *label_nav_isOmnistar_value;
    QLabel *label_nav_isStatic_value;
    QLabel *label_nav_isNetworkRTK_value;
    QLabel *label_nav_isLocationRTK_value;
    QLabel *label_nav_isBeaconGPS_value;
    QLabel *label_nav_initCounter_value;

    QGridLayout *navLayout;
    QWidget *navWidget;

    QTabWidget *mainTabs;
    QGridLayout *mainLayout;


    void setup_nav_panel();
};//end class Dashboard
#endif

