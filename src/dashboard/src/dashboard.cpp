#include "dashboard/dashboard.h"

Dashboard::Dashboard(int argc, char **argv, QWidget *parent)
    : QWidget(parent) {
    
    setup_nav_panel();
    navWidget = new QWidget();
    navWidget->setLayout(navLayout);

    mainLayout = new QGridLayout();
    mainLayout->setContentsMargins(0, 0, 0, 0);

    mainTabs = new QTabWidget();
    mainTabs->addTab(navWidget, "GPS");

    mainLayout->addWidget(mainTabs, 0, 0);

    setLayout(mainLayout);

    setWindowTitle(tr("Corvette Dashboard"));

    show();
}

void Dashboard::setup_nav_panel() {
    /** Set up the Controls **/
    l1Snr = new QtCharts::QBarSet("L1");
    l2Snr = new QtCharts::QBarSet("L2");

    *l1Snr << 1 << 2 << 3 << 4 << 5 << 6;
    *l2Snr << 6 << 5 << 4 << 3 << 2 << 1;

    snrBars = new QtCharts::QBarSeries();
    snrBars->append(l1Snr);
    snrBars->append(l2Snr);

    chart = new QtCharts::QChart();
    chart->addSeries(snrBars);


    QStringList categories;
    categories << "PRN1" << "PRN2" << "PRN3" << "PRN4" << "PRN5" << "PRN6";

    QtCharts::QBarCategoryAxis *axisX = new QtCharts::QBarCategoryAxis();
    axisX->append(categories);
    chart->setAxisX(axisX, snrBars);

    QtCharts::QValueAxis *axisY = new QtCharts::QValueAxis();
    chart->setAxisY(axisY, snrBars);
    axisY->setRange(0, 60);

    chart->legend()->setVisible(false);
    chart->setContentsMargins(0, 0, 0, 0);

    chartView = new QtCharts::QChartView(chart);
    chartView->setContentsMargins(0, 0, 0, 0);



    label_nav_newPosition = new QLabel("New Position");
    label_nav_newClockThisSolution = new QLabel("New Clock");
    label_nav_newHorizontalThisSolution = new QLabel("New Horz");
    label_nav_newHeightThisSolution = new QLabel("New Height");
    label_nav_usesLeastSquares = new QLabel("Uses Least Squares");
    label_nav_usesFilteredL1 = new QLabel("Uses Filtered L1");
    label_nav_isDifferential = new QLabel("Is Differential");
    label_nav_isPhase = new QLabel("Is Phase");
    label_nav_isFixedInteger = new QLabel("Is Fixed Int");
    label_nav_isOmnistar = new QLabel("Is Omnistar");
    label_nav_isStatic = new QLabel("Is Static");
    label_nav_isNetworkRTK = new QLabel("Is Network RTK");
    label_nav_isLocationRTK = new QLabel("Is Location RTK");
    label_nav_isBeaconGPS = new QLabel("Is Beacon DGPS");
    label_nav_initCounter = new QLabel("Init Counter");

    label_nav_newPosition_value = new QLabel("True");
    label_nav_newClockThisSolution_value = new QLabel("True");
    label_nav_newHorizontalThisSolution_value = new QLabel("True");
    label_nav_newHeightThisSolution_value = new QLabel("True");
    label_nav_usesLeastSquares_value = new QLabel("True");
    label_nav_usesFilteredL1_value = new QLabel("False");
    label_nav_isDifferential_value = new QLabel("False");
    label_nav_isPhase_value = new QLabel("True");
    label_nav_isFixedInteger_value = new QLabel("True");
    label_nav_isOmnistar_value = new QLabel("False");
    label_nav_isStatic_value = new QLabel("False");
    label_nav_isNetworkRTK_value = new QLabel("True");
    label_nav_isLocationRTK_value = new QLabel("False");
    label_nav_isBeaconGPS_value = new QLabel("True");
    label_nav_initCounter_value = new QLabel("0");


    navLayout = new QGridLayout();
    navLayout->addWidget(chartView, 0, 0, 1, 2);

    navLayout->addWidget(label_nav_newPosition, 1, 0, 1, 1);
    navLayout->addWidget(label_nav_newPosition_value, 1, 1, 1, 1);

    navLayout->addWidget(label_nav_newClockThisSolution, 2, 0, 1, 1);
    navLayout->addWidget(label_nav_newClockThisSolution_value, 2, 1, 1, 1);

    navLayout->addWidget(label_nav_newHorizontalThisSolution, 3, 0, 1, 1);
    navLayout->addWidget(label_nav_newHorizontalThisSolution_value, 3, 1, 1, 1);

    navLayout->addWidget(label_nav_newHeightThisSolution, 4, 0, 1, 1);
    navLayout->addWidget(label_nav_newHeightThisSolution_value, 4, 1, 1, 1);

    navLayout->addWidget(label_nav_usesLeastSquares, 5, 0, 1, 1);
    navLayout->addWidget(label_nav_usesLeastSquares_value, 5, 1, 1, 1);

    navLayout->addWidget(label_nav_usesFilteredL1, 6, 0, 1, 1);
    navLayout->addWidget(label_nav_usesFilteredL1_value, 6, 1, 1, 1);

    navLayout->addWidget(label_nav_isDifferential, 7, 0, 1, 1);
    navLayout->addWidget(label_nav_isDifferential_value, 7, 1, 1, 1);

    navLayout->addWidget(label_nav_isPhase, 8, 0, 1, 1);
    navLayout->addWidget(label_nav_isPhase_value, 8, 1, 1, 1);

    navLayout->addWidget(label_nav_isFixedInteger, 9, 0, 1, 1);
    navLayout->addWidget(label_nav_isFixedInteger_value, 9, 1, 1, 1);

    navLayout->addWidget(label_nav_isOmnistar, 10, 0, 1, 1);
    navLayout->addWidget(label_nav_isOmnistar_value, 10, 1, 1, 1);

    navLayout->addWidget(label_nav_isStatic, 11, 0, 1, 1);
    navLayout->addWidget(label_nav_isStatic_value, 11, 1, 1, 1);

    navLayout->addWidget(label_nav_isNetworkRTK, 12, 0, 1, 1);
    navLayout->addWidget(label_nav_isNetworkRTK_value, 12, 1, 1, 1);

    navLayout->addWidget(label_nav_isLocationRTK, 13, 0, 1, 1);
    navLayout->addWidget(label_nav_isLocationRTK_value, 13, 1, 1, 1);

    navLayout->addWidget(label_nav_isBeaconGPS, 14, 0, 1, 1);
    navLayout->addWidget(label_nav_isBeaconGPS_value, 14, 1, 1, 1);

    navLayout->addWidget(label_nav_initCounter, 15, 0, 1, 1);
    navLayout->addWidget(label_nav_initCounter_value, 15, 1, 1, 1);
}

